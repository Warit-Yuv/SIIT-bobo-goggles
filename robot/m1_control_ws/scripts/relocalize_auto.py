#!/usr/bin/env python3
"""Find where the robot is on the map, automatically.

    ./relocalize_auto.py                 # search the whole map
    ./relocalize_auto.py --hint 1.5 2.0  # search near a guess (faster, safer)
    ./relocalize_auto.py --dry-run       # report the best pose, do not send it

WHY THIS EXISTS
---------------
AMCL needs a starting pose. Without one it sits at whatever `initial_pose` says
- usually (0, 0, 0) - and the robot appears somewhere it is not. The documented
fix is "click 2D Pose Estimate in RViz", which is fine when you are watching the
map and can see roughly where the robot is, and useless when you are not.

This does the search for you. It takes the current laser scan, slides it over
the saved map at every plausible position and heading, and scores each placement
by how well the scan lands on walls. The best score is published as the initial
pose, and AMCL refines from there.

HOW IT SCORES
-------------
A distance transform of the map gives, for every cell, the distance to the
nearest occupied cell. A scan endpoint that lands on a wall scores 1; one that
lands in the middle of free space scores near 0. Summing over all beams and
normalising gives a match quality in 0..1 that is directly comparable between
poses, so the search can report not just the winner but how much better it is
than the runner-up.

That margin matters. A best score that barely beats the second-best means the
scan fits two places equally well - two similar corridors, or a room with a lot
of symmetry - and picking the wrong one is worse than refusing. The tool says so
rather than guessing.

WHY NOT USE THE PARTICLE FILTER FOR THIS
----------------------------------------
AMCL does have a global-localisation mode, but it needs the robot to drive
around to gather evidence, and its recovery behaviour is tuned for a robot that
is already roughly localised. A brute-force match over a saved map answers the
question in about a second, with an explicit confidence number, and does not
disturb the filter if the answer is unclear.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np


def build_likelihood_field(grid, unknown_as_free=True):
    """Distance (in cells) from every cell to the nearest occupied cell."""
    import cv2

    # cv2.distanceTransform measures distance to the nearest ZERO pixel, so
    # obstacles must be 0 and everything else non-zero.
    h, w = grid.shape
    binimg = np.full((h, w), 255, dtype=np.uint8)
    binimg[grid >= 50] = 0                    # occupied -> 0
    if not unknown_as_free:
        binimg[grid < 0] = 0                  # unknown counts as an obstacle

    dist = cv2.distanceTransform(binimg, cv2.DIST_L2, 5)
    return dist.astype(np.float32)


def score_poses(scan_xy, dist_field, res, origin, poses, sigma_cells=1.0,
                min_inside=0.55):
    """Score each (x, y, yaw) by how well the scan lands on walls.

    scan_xy  : (N, 2) laser endpoints in the robot frame, metres
    poses    : (M, 3) candidate poses (x, y, yaw)
    returns  : (M,) scores in 0..1

    Normalisation is by the TOTAL number of beams, not by the number that
    happen to land inside the map. That detail decides whether the search works
    at all.

    Averaging only over in-map beams makes poses near the map boundary look
    excellent: a pose in the corner has a handful of beams inside, and if those
    few graze a wall the average is near 1.0 while the correct pose - with every
    beam accounted for and a few legitimately missing due to map noise - scores
    lower. Measured on this map, every top candidate was pinned to a map edge,
    and the "confidence margin" between first and second place was 0.002, which
    is noise.

    Counting out-of-map beams as zero match fixes it: a pose can only score high
    if it explains the whole scan, which is what being in the right place
    actually means.
    """
    h, w = dist_field.shape
    inv2s2 = 1.0 / (2.0 * sigma_cells * sigma_cells)
    n_beams = len(scan_xy)

    out = np.zeros(len(poses), dtype=np.float32)
    xs = scan_xy[:, 0]
    ys = scan_xy[:, 1]

    # Vectorised over positions for a fixed yaw: the scan only rotates once per
    # heading, so the per-yaw rotation is done once and reused for every
    # translation. This is what makes a full-map search take about a second
    # instead of a minute.
    for yaw in np.unique(poses[:, 2]):
        sel = poses[:, 2] == yaw
        c, s = math.cos(yaw), math.sin(yaw)
        rx = xs * c - ys * s
        ry = xs * s + ys * c

        # (K, N) world coordinates for the K candidate positions at this yaw
        wx = poses[sel, 0][:, None] + rx[None, :]
        wy = poses[sel, 1][:, None] + ry[None, :]

        px = ((wx - origin[0]) / res).astype(np.int32)
        py = ((wy - origin[1]) / res).astype(np.int32)

        inside = (px >= 0) & (px < w) & (py >= 0) & (py < h)
        d = dist_field[np.clip(py, 0, h - 1), np.clip(px, 0, w - 1)]
        lik = np.where(inside, np.exp(-(d * d) * inv2s2), 0.0)

        # Total-beam normalisation: a beam outside the map scores zero, so a
        # pose that leaves half the scan unexplained cannot win.
        sc = lik.sum(axis=1) / max(1, n_beams)

        # Reject poses that barely see the map at all - their score is computed
        # from too little evidence to mean anything.
        sc = np.where(inside.mean(axis=1) >= min_inside, sc, 0.0)
        out[sel] = sc

    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hint', nargs=2, type=float, metavar=('X', 'Y'),
                    help='search within --radius of this point')
    ap.add_argument('--radius', type=float, default=1.5,
                    help='search radius around --hint, metres (default 1.5)')
    ap.add_argument('--pos-step', type=float, default=0.10,
                    help='position grid spacing, metres (default 0.10)')
    ap.add_argument('--yaw-step', type=float, default=10.0,
                    help='heading grid spacing, degrees (default 10)')
    ap.add_argument('--dry-run', action='store_true',
                    help='report the best pose without publishing it')
    ap.add_argument('--top', type=int, default=3,
                    help='show this many top candidates (default 3)')
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node('relocalize_auto')

    map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
    scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)

    store = {}

    def map_cb(msg):
        store['map'] = msg

    def scan_cb(msg):
        store['scan'] = msg

    node.create_subscription(OccupancyGrid, '/map', map_cb, map_qos)
    node.create_subscription(LaserScan, '/scan', scan_cb, scan_qos)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    print('waiting for /map and /scan ...')
    t0 = time.monotonic()
    while (('map' not in store or 'scan' not in store)
           and time.monotonic() - t0 < 20.0):
        rclpy.spin_once(node, timeout_sec=0.1)

    if 'map' not in store:
        print('ERROR: no /map. Start localisation first:')
        print('    ./m1_map.sh localize NAME')
        return 1
    if 'scan' not in store:
        print('ERROR: no /scan. Is the robot stack running?')
        return 1

    m = store['map']
    s = store['scan']
    info = m.info
    w, h = info.width, info.height
    res = info.resolution
    origin = (info.origin.position.x, info.origin.position.y)
    print(f'map  : {w}x{h} @ {res * 100:.0f} cm, origin '
          f'({origin[0]:.2f}, {origin[1]:.2f})')

    grid = np.array(m.data, dtype=np.int16).reshape(h, w)
    occupied = int((grid >= 50).sum())
    print(f'       {occupied} occupied cells')

    # ---- scan endpoints in the robot frame -------------------------------
    ranges = np.asarray(s.ranges, dtype=np.float32)
    angles = s.angle_min + np.arange(len(ranges)) * s.angle_increment
    rmax = min(s.range_max, 6.0)      # longer returns mostly exit the room
    ok = np.isfinite(ranges) & (ranges > max(0.10, s.range_min)) & (ranges < rmax)
    n_valid = int(ok.sum())
    if n_valid < 20:
        print(f'ERROR: only {n_valid} usable beams in the scan')
        return 1
    scan_xy = np.stack([ranges[ok] * np.cos(angles[ok]),
                        ranges[ok] * np.sin(angles[ok])], axis=1)
    print(f'scan : {n_valid} usable beams')

    # ---- search grid -----------------------------------------------------
    if args.hint:
        cx, cy = args.hint
        print(f'search: within {args.radius} m of ({cx:.2f}, {cy:.2f})')
    else:
        cx = origin[0] + w * res / 2.0
        cy = origin[1] + h * res / 2.0
        args.radius = max(w * res, h * res)
        print(f'search: whole map ({args.radius:.1f} m)')

    step = args.pos_step
    n = int(args.radius / step)
    gx = np.arange(-n, n + 1) * step + cx
    gy = np.arange(-n, n + 1) * step + cy
    yaws = np.deg2rad(np.arange(0.0, 360.0, args.yaw_step))

    # Keep candidates inside the mapped area; outside there is no information.
    X, Y, T = np.meshgrid(gx, gy, yaws, indexing='ij')
    cand = np.stack([X.ravel(), Y.ravel(), T.ravel()], axis=1)
    px = (cand[:, 0] - origin[0]) / res
    py = (cand[:, 1] - origin[1]) / res
    cand = cand[(px >= 0) & (px < w) & (py >= 0) & (py < h)]
    print(f'poses : {len(cand)} candidates '
          f'({step * 100:.0f} cm x {args.yaw_step:.0f} deg)')

    dist = build_likelihood_field(grid)
    print('scoring ...')
    t0 = time.monotonic()
    scores = score_poses(scan_xy, dist, res, origin, cand)
    elapsed = time.monotonic() - t0
    print(f'       {len(cand)} poses in {elapsed:.2f}s')

    order = np.argsort(scores)[::-1][:max(1, args.top)]
    best = cand[order[0]]
    best_score = float(scores[order[0]])
    second = float(scores[order[1]]) if len(order) > 1 else 0.0
    margin = best_score - second

    print()
    print('=== best matches ===')
    for rank, i in enumerate(order, 1):
        p = cand[i]
        print(f'  {rank}. x={p[0]:+7.3f}  y={p[1]:+7.3f}  '
              f'yaw={math.degrees(p[2]):+7.1f} deg   score {scores[i]:.3f}')

    print()
    print(f'confidence: best {best_score:.3f}, runner-up {second:.3f}, '
          f'margin {margin:.3f}')

    if best_score < 0.25:
        print('  WEAK: the scan does not fit the map well anywhere.')
        print('  Either the robot is outside the mapped area, or the room has')
        print('  changed, or the map is bad. Re-map before trusting this.')
    elif margin < 0.02:
        print('  AMBIGUOUS: two places fit almost equally well.')
        print('  This room is symmetric enough that one scan cannot decide.')
        print('  Use --hint X Y with a rough guess, or click 2D Pose Estimate.')
    else:
        print('  GOOD: one placement is clearly better than the rest.')

    if args.dry_run:
        print()
        print('--dry-run: not publishing.')
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # ---- publish ---------------------------------------------------------
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(best[0])
    msg.pose.pose.position.y = float(best[1])
    msg.pose.pose.position.z = 0.0
    yaw = float(best[2])
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # Covariance reflects the search resolution, not the match quality: AMCL
    # should refine within about one grid step, and the particle filter is told
    # to spread a little wider when the match was weak.
    spread = (args.pos_step * 2.0) ** 2 * (1.0 + (1.0 - best_score) * 4.0)
    yaw_spread = math.radians(args.yaw_step * 1.5) ** 2
    msg.pose.covariance[0] = spread
    msg.pose.covariance[7] = spread
    msg.pose.covariance[35] = yaw_spread

    # Publish a few times: AMCL may not be subscribed yet at the instant this
    # finishes, and a single message would be silently dropped.
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print()
    print(f'published /initialpose: x={best[0]:+.3f} y={best[1]:+.3f} '
          f'yaw={math.degrees(yaw):+.1f} deg')
    print('AMCL should converge within a second or two.')
    print('Check with:  ./m1_map.sh status')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
