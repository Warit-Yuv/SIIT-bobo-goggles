"""Checks that head pose is recovered with the right signs and units.

    python3 scripts/test_gaze_pose.py

Head rotation is what makes vertical gaze work at all: the iris barely moves
up and down inside the eye, so an iris-only vertical signal is mostly noise.
Measured on the hardware, the fitted vertical gain came back clamped at its
0.25 minimum - the fit saying "there is no usable relationship here".

So this checks the three things that were wrong before:

  * neutral must be 0 deg, not 180
  * up must be positive pitch, down negative
  * moving closer or further must NOT look like tilting the head

Synthetic landmarks are projected from a known rotation, so the expected
answer is known exactly rather than eyeballed.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

import gaze_hud_node as g                                        # noqa: E402

FOCAL = 640.0
K = np.array([[FOCAL, 0.0, 320.0],
              [0.0, FOCAL, 240.0],
              [0.0, 0.0, 1.0]], dtype=np.float64)


def rot_x(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


class Point:
    def __init__(self, x, y):
        self.x, self.y = x, y


class Landmarks:
    """Only the six PnP landmarks matter; the rest are filler."""

    def __init__(self, points):
        self.points = points

    def __getitem__(self, i):
        return self.points[i]


def landmarks_for(rot, distance_mm=500.0):
    rvec, _ = cv2.Rodrigues(rot)
    projected, _ = cv2.projectPoints(
        g._PNP_MODEL, rvec, np.array([0.0, 0.0, distance_mm]), K, None)
    projected = projected.reshape(-1, 2)
    points = [Point(0.5, 0.5)] * 500
    for k, idx in enumerate(g._PNP_IDS):
        points[idx] = Point(projected[k][0] / 640.0, projected[k][1] / 480.0)
    return Landmarks(points)


failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


def pose(rot, distance_mm=500.0):
    return g.GazeHUDNode._head_rotation(
        None, landmarks_for(rot, distance_mm), 640, 480)


def main():
    cases = [
        # name,               rotation,             yaw,  pitch
        ('neutral',           np.eye(3),              0.0,   0.0),
        ('head UP 15',        rot_x(15),              0.0,  15.0),
        ('head DOWN 15',      rot_x(-15),             0.0, -15.0),
        ('turn RIGHT 20',     rot_y(-20),            20.0,   0.0),
        ('turn LEFT 20',      rot_y(20),            -20.0,   0.0),
    ]
    print('%-18s %8s %8s' % ('pose', 'yaw', 'pitch'))
    for name, rot, want_yaw, want_pitch in cases:
        got = pose(rot)
        if got is None:
            check(name + ' returns a pose', False)
            continue
        yaw, pitch = math.degrees(got[0]), math.degrees(got[1])
        print('%-18s %8.1f %8.1f' % (name, yaw, pitch))
        check(name + ' yaw', abs(yaw - want_yaw) < 2.0, '%.1f want %.1f' % (yaw, want_yaw))
        check(name + ' pitch', abs(pitch - want_pitch) < 2.0,
              '%.1f want %.1f' % (pitch, want_pitch))

    up = pose(rot_x(15))
    down = pose(rot_x(-15))
    check('looking up gives a LARGER pitch than looking down',
          up[1] > down[1],
          '%.1f vs %.1f deg' % (math.degrees(up[1]), math.degrees(down[1])))

    # The v the node actually uses must go DOWN when the head goes up.
    span = g.HEAD_PITCH_SPAN
    v_up = 0.5 - 0.5 * max(-1.0, min(1.0, up[1] / span))
    v_down = 0.5 - 0.5 * max(-1.0, min(1.0, down[1] / span))
    check('v decreases when the head looks up', v_up < v_down,
          'v %.2f up vs %.2f down' % (v_up, v_down))

    near = pose(np.eye(3), 300.0)
    far = pose(np.eye(3), 900.0)
    check('distance does not fake a head tilt',
          abs(math.degrees(near[1])) < 2.0 and abs(math.degrees(far[1])) < 2.0,
          'near %.1f far %.1f deg' % (math.degrees(near[1]), math.degrees(far[1])))

    degenerate = Landmarks([Point(0.5, 0.5)] * 500)
    check('degenerate landmarks are refused, not crashed on',
          g.GazeHUDNode._head_rotation(None, degenerate, 640, 480) is None)

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
