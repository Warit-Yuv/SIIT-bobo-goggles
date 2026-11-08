"""Measure yaw drift while the robot is STILL.

This is the test that should have been run before changing the EKF.

Removing wheel-derived yaw made the IMU the only authority on heading. That
fixed the hand-rotation case, but it has a cost: this IMU has no magnetometer
(imu_filter_param.yaml sets use_mag: false), so its yaw is pure gyro
integration. Any gyro bias integrates into a slow, continuous rotation that no
sensor ever corrects.

While the wheels are on the ground and the robot is driving, wheel odometry is
a genuinely good yaw source - mecanum kinematics measure rotation directly.
Dropping it removes a real correction, and the drift shows up during ordinary
mapping, which is the common case.

This prints how much /odom yaw moves while nothing is moving at all.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
DRIFT_LIMIT_DEG = 1.0      # per minute, before it is worth worrying about

rclpy.init()
n = rclpy.create_node('drift_probe')

yaw = {'imu': [], 'odom': [], 'raw': []}
gyro_z = []


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


def mk_odom(key):
    def cb(msg):
        yaw[key].append((time.monotonic(), yaw_of(msg.pose.pose.orientation)))
    return cb


def imu_cb(msg):
    yaw['imu'].append((time.monotonic(), yaw_of(msg.orientation)))
    gyro_z.append((time.monotonic(), msg.angular_velocity.z))


n.create_subscription(Imu, '/imu/data', imu_cb, 20)
n.create_subscription(Odometry, '/odom', mk_odom('odom'), 20)
n.create_subscription(Odometry, '/odom_raw', mk_odom('raw'), 20)

print(f'Recording {DURATION:.0f}s. DO NOT TOUCH THE ROBOT.')
print('(The point is to measure drift with nothing moving at all.)')
print()

t0 = time.monotonic()
while time.monotonic() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)

n.destroy_node()
rclpy.shutdown()

print('=== yaw change while stationary ===')
rows = {}
for key, label in (('imu', '/imu/data'),
                   ('odom', '/odom      (SLAM uses this)'),
                   ('raw', '/odom_raw  (wheels)')):
    vals = unwrap([v for _, v in yaw[key]])
    if len(vals) < 2:
        print(f'  {label:28} no data')
        rows[key] = 0.0
        continue
    net = math.degrees(vals[-1] - vals[0])
    span = math.degrees(max(vals) - min(vals))
    rows[key] = net
    print(f'  {label:28} net {net:+7.2f} deg   span {span:6.2f} deg   '
          f'{len(vals)} msgs')

print()
print('=== gyro bias ===')
if gyro_z:
    n_s = len(gyro_z)
    mean = sum(v for _, v in gyro_z) / n_s
    # Integrating a constant bias over a minute is what drift actually is.
    per_min = math.degrees(mean * 60.0)
    print(f'  mean angular_velocity.z : {mean:+.6f} rad/s')
    print(f'  => integrates to        : {per_min:+.1f} deg/minute')
    print(f'  over a 10 min session   : {per_min * 10:+.0f} deg')
else:
    print('  no gyro data')

print()
print('=== verdict ===')
d = abs(rows['odom'])
minutes = DURATION / 60.0
rate = d / minutes if minutes > 0 else 0
if d < 0.2:
    print(f'  /odom drifted {d:.2f} deg while still - healthy.')
elif rate <= DRIFT_LIMIT_DEG:
    print(f'  /odom drifted {d:.2f} deg ({rate:.1f} deg/min) - acceptable.')
else:
    print(f'  /odom drifted {d:.2f} deg ({rate:.1f} deg/min) while STILL.')
    print('  That is gyro-bias drift, and it accumulates for the whole')
    print('  session. Heading needs a correction source that does not')
    print('  integrate: wheel odometry while the robot is driving.')
    print('  -> restore yaw to odom0_config in config/ekf_m1.yaml')

sys.exit(0)
