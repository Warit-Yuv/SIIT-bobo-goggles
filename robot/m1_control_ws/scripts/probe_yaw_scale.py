"""Calibrate wheel odometry rotation against the gyro.

WHY THIS EXISTS
---------------
"Rotate a bit and the map is unaligned" is the signature of a systematic
rotation error. If the base controller under- or over-reports how far the robot
turned, every corner in the map is rotated by that error, and the errors add up
as you drive. Scan matching can absorb a little of it; past that it shears.

Wheel odometry computes rotation from wheel displacement over a known track
width (`wheelbase` on /base_node_M1, currently 0.25 m) and multiplies by
`angular_scale` (currently 1.0, i.e. uncalibrated). If either is wrong, rotation
is wrong by a fixed percentage - which is exactly what "unaligned after
rotating" looks like.

The gyro is the independent reference here. It has its own scale error, but it
is a completely different measurement principle, so agreement between the two is
strong evidence that both are right.

HOW TO RUN IT
-------------
    scripts/probe_yaw_scale.py 60

Then DRIVE the robot on the floor, turning both ways, for the whole window.
Include some long sweeps rather than only tiny corrections - a big rotation
makes the ratio much more precise.

The wheels must be turning. With the robot lifted, /odom_raw correctly reports
zero and there is nothing to compare.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

args = [a for a in sys.argv[1:] if not a.startswith('--')]
DURATION = float(args[0]) if args else 40.0
MIN_SPAN_DEG = 30.0

rclpy.init()
n = rclpy.create_node('yaw_scale')

yaw = {'imu': [], 'raw': [], 'odom': []}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(seq):
    if not seq:
        return []
    out = [seq[0]]
    for v in seq[1:]:
        d = v - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def mk(key):
    def cb(msg):
        q = (msg.orientation if hasattr(msg, 'orientation')
             else msg.pose.pose.orientation)
        yaw[key].append(yaw_of(q))
    return cb


n.create_subscription(Imu, '/imu/data', mk('imu'), 20)
n.create_subscription(Odometry, '/odom_raw', mk('raw'), 20)
n.create_subscription(Odometry, '/odom', mk('odom'), 20)

print(f'Recording {DURATION:.0f}s.')
print('DRIVE the robot on the floor. Turn both ways. Make some long sweeps.')
print()

t0 = time.monotonic()
last = 0.0
while time.monotonic() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.monotonic() - t0
    if el - last >= 3.0:
        last = el

        def span_now(k):
            v = unwrap(yaw[k])
            return abs(math.degrees(v[-1] - v[0])) if len(v) > 1 else 0.0
        print(f'  t={el:5.1f}s  net travel: imu {span_now("imu"):7.1f} deg  '
              f'wheels {span_now("raw"):7.1f} deg')

n.destroy_node()
rclpy.shutdown()


def travel(key):
    """Total angular travel, summing absolute deltas.

    Net displacement is wrong here: turning left then right cancels out and the
    ratio becomes meaningless. Summing |delta| measures how much rotation
    actually happened in either direction.
    """
    v = unwrap(yaw[key])
    return sum(abs(v[i] - v[i - 1]) for i in range(1, len(v)))


t_imu = math.degrees(travel('imu'))
t_raw = math.degrees(travel('raw'))
t_odom = math.degrees(travel('odom'))

print()
print('=== angular travel (sum of absolute deltas) ===')
print(f'  gyro   /imu/data   {t_imu:8.1f} deg')
print(f'  wheels /odom_raw   {t_raw:8.1f} deg')
print(f'  EKF    /odom       {t_odom:8.1f} deg')

print()
if t_imu < MIN_SPAN_DEG:
    print(f'Only {t_imu:.0f} deg of rotation was recorded - not enough to judge.')
    print(f'Run again and turn the robot more (need {MIN_SPAN_DEG:.0f}+ deg).')
    sys.exit(0)

ratio = t_raw / t_imu
print('=== wheel odometry rotation accuracy ===')
print(f'  wheels / gyro = {ratio:.4f}')

if 0.97 <= ratio <= 1.03:
    print('  Within 3% - wheel rotation is accurate. Drift is coming from')
    print('  somewhere else: scan matching, TF timing, or actual wheel slip.')
else:
    err = (ratio - 1.0) * 100.0
    print(f'  Off by {err:+.1f}%. Wheel odometry is '
          f'{"under" if ratio < 1 else "over"}-reporting rotation.')
    print()
    print('  To correct it, set angular_scale on /base_node_M1:')
    print(f'      current 1.0  ->  suggested {1.0 / ratio:.3f}')
    print()
    print('  (Dividing, not multiplying: if the wheels report {ratio:.2f}x the')
    print('   true angle, the scale must shrink by the same factor.)')
    print()
    print('  Test it live:')
    print(f'      ros2 param set /base_node_M1 angular_scale {1.0 / ratio:.3f}')

print()
r_ekf = t_odom / t_imu
print('=== EKF rotation fidelity ===')
print(f'  odom / gyro = {r_ekf:.4f}')
if 0.90 <= r_ekf <= 1.10:
    print('  The EKF tracks the gyro faithfully - fusion is not damping motion.')
else:
    print(f'  The EKF differs from the gyro by {abs(r_ekf - 1) * 100:.0f}%.')
    print('  With wheel yaw fused this should be close to the wheel ratio.')

sys.exit(0)
