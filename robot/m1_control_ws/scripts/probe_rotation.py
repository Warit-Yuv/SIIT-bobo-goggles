"""Measure whether rotating the robot BY HAND (wheels still) moves /odom yaw.

The report was: the map only updates when driving, not when the robot is
rotated by hand while floating.

That distinction is diagnostic. When the wheels are off the ground and the
robot is turned by hand, the wheel encoders see NOTHING. So:

  * if /odom yaw tracks the turn, the IMU is genuinely fused and SLAM should
    update;
  * if /odom yaw stays put, the EKF is trusting wheel odometry over the IMU,
    and slam_toolbox never sees motion - it triggers on minimum_travel_heading
    measured against the ODOMETRY pose, not against reality.

Prints a live readout plus a final summary of how far each source moved.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

rclpy.init()
n = rclpy.create_node('rotation_probe')

samples = {'imu': [], 'odom': [], 'raw': []}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(seq):
    """Accumulate yaw so a turn past +-pi does not cancel itself out."""
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


def mk_cb(key):
    def cb(msg):
        # Imu exposes .orientation directly; Odometry nests it under
        # pose.pose.orientation. Getting this wrong raises AttributeError on
        # every message and the probe records nothing.
        q = msg.orientation if hasattr(msg, 'orientation') else msg.pose.pose.orientation
        samples[key].append((time.monotonic(), yaw_of(q)))
    return cb


n.create_subscription(Imu, '/imu/data', mk_cb('imu'), 20)
n.create_subscription(Odometry, '/odom', mk_cb('odom'), 20)
n.create_subscription(Odometry, '/odom_raw', mk_cb('raw'), 20)

print(f'Recording for {DURATION:.0f}s.')
print('ROTATE THE ROBOT BY HAND now (wheels off the ground).')
print()
print(f"{'t':>5}  {'imu yaw':>9}  {'odom yaw':>9}  {'raw yaw':>9}   (degrees)")
print('-' * 56)

t0 = time.monotonic()
last_print = 0.0
while time.monotonic() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.monotonic() - t0
    if el - last_print >= 1.0:
        last_print = el

        def deg(key):
            if not samples[key]:
                return '   --'
            return f'{math.degrees(samples[key][-1][1]):8.1f}'
        print(f'{el:5.1f}  {deg("imu"):>9}  {deg("odom"):>9}  '
              f'{deg("raw"):>9}')

n.destroy_node()
rclpy.shutdown()

print()
print('=== how far did each source actually move? ===')
res = {}
for key, label in (('imu', '/imu/data  (raw sensor)'),
                   ('odom', '/odom       (EKF output -> SLAM uses this)'),
                   ('raw', '/odom_raw   (wheel encoders only)')):
    vals = unwrap([v for _, v in samples[key]])
    if len(vals) < 2:
        print(f'  {label:46} no data')
        res[key] = 0.0
        continue
    span = math.degrees(max(vals) - min(vals))
    net = math.degrees(vals[-1] - vals[0])
    res[key] = span
    print(f'  {label:46} {len(vals):4d} msgs  '
          f'net {net:+7.1f} deg  span {span:6.1f} deg')

print()
print('=== verdict ===')
imu, odom, raw = res['imu'], res['odom'], res['raw']
if imu < 10:
    print('  The IMU barely moved. Either the robot was not turned, or the')
    print('  IMU yaw is not working at all.')
elif odom < 10 <= imu:
    print('  CONFIRMED BUG: the IMU saw the turn but /odom did not.')
    print('  The EKF is ignoring IMU yaw, so slam_toolbox never sees motion')
    print('  and refuses to update the map.')
elif odom >= 10:
    print('  /odom DID follow the hand turn, so the odometry path is fine.')
    print('  The map not updating is then a slam_toolbox threshold or')
    print('  transform problem, not a sensor problem.')
if raw < 5 <= imu:
    print(f'  (wheel odometry moved {raw:.1f} deg, as expected with the')
    print('   wheels off the ground - that is the whole point of the test.)')

sys.exit(0)
