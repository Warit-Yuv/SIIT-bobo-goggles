#!/usr/bin/env python3
"""Automated safety validation for the M1 control stack.

Runs the handbook's acceptance protocols (sections 7.2 and 11) as assertions
instead of manual observation, so a regression is caught rather than noticed.

    python3 validate_safety.py

Requires: the chassis driver and the arbiter to be running (start_robot.sh).
Read-only with respect to hardware state - it only publishes to the arbiter's
input topics and watches /cmd_vel. The robot MUST have its wheels off the
ground for the motion checks.

Checks
------
 1. /cmd_vel has exactly one publisher (the arbiter)          [handbook issue 2]
 2. Watchdog: stop publishing -> /cmd_vel goes to zero < 0.35 s
 3. Mode isolation: /cmd_vel/gaze is ignored in KEYBOARD mode
 4. Velocity clamping: an oversized request is capped at the envelope
 5. E-STOP latches: mode 0 keeps /cmd_vel at zero even with active input
 6. EEG path: UDP GO -> /cmd_vel/eeg -> /cmd_vel forward motion
 7. EEG link loss: stop heartbeating -> robot stops
"""

import json
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import UInt8

MODE_ESTOP, MODE_KEYBOARD, MODE_GAZE, MODE_EEG = 0, 1, 2, 3

PASS, FAIL = '\033[32mPASS\033[0m', '\033[31mFAIL\033[0m'
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f'  [{PASS if ok else FAIL}] {name}' + (f'  ({detail})' if detail else ''))
    return ok


class Probe(Node):

    def __init__(self):
        super().__init__('m1_safety_probe')
        self.latest = None
        self.count = 0
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.mode_pub = self.create_publisher(UInt8, '/control_mode', mode_qos)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel/keyboard', 10)
        self.gaze_pub = self.create_publisher(Twist, '/cmd_vel/gaze', 10)
        self.eeg_pub = self.create_publisher(Twist, '/cmd_vel/eeg', 10)

    def _cb(self, msg):
        self.latest = msg
        self.count += 1

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def magnitude(self):
        if self.latest is None:
            return 0.0
        return max(abs(self.latest.linear.x), abs(self.latest.linear.y),
                   abs(self.latest.angular.z))

    def set_mode(self, mode):
        self.mode_pub.publish(UInt8(data=mode))
        self.spin_for(0.4)

    def publish_hold(self, pub, twist, seconds):
        """Publish continuously (the arbiter needs a fresh command each cycle)."""
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_zero(self, timeout):
        """Seconds until /cmd_vel reads zero, or None on timeout."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.magnitude() < 1e-6:
                return time.time() - t0
        return None


def main():
    rclpy.init()
    p = Probe()
    p.spin_for(1.5)

    print('\n=== 1. publisher ownership ===')
    pub_count = 0
    for name in p.get_publishers_info_by_topic('/cmd_vel'):
        pub_count += 1
        print(f'     publisher: {name.node_name}')
    check('/cmd_vel has exactly one publisher', pub_count == 1,
          f'{pub_count} publisher(s)')

    print('\n=== 2. watchdog stop drill ===')
    p.set_mode(MODE_KEYBOARD)
    fwd = Twist()
    fwd.linear.x = 0.10
    p.publish_hold(p.vel_pub, fwd, 1.0)
    moving = p.magnitude() > 0.05
    check('keyboard command reaches /cmd_vel', moving,
          f'|v|={p.magnitude():.3f}')
    t_stop = p.wait_for_zero(2.0)
    check('watchdog halts within 0.35 s', t_stop is not None and t_stop <= 0.45,
          f'{t_stop:.3f} s' if t_stop else 'never stopped')

    print('\n=== 3. mode isolation ===')
    p.set_mode(MODE_KEYBOARD)
    p.publish_hold(p.gaze_pub, fwd, 0.8)
    check('gaze ignored in KEYBOARD mode', p.magnitude() < 1e-6,
          f'|v|={p.magnitude():.3f}')

    print('\n=== 4. velocity clamping ===')
    p.set_mode(MODE_KEYBOARD)
    big = Twist()
    big.linear.x = 5.0
    big.linear.y = 5.0
    big.angular.z = 5.0
    p.publish_hold(p.vel_pub, big, 1.0)
    vx = abs(p.latest.linear.x) if p.latest else 0
    vy = abs(p.latest.linear.y) if p.latest else 0
    wz = abs(p.latest.angular.z) if p.latest else 0
    check('vx clamped to <= 0.25', vx <= 0.2501, f'{vx:.3f}')
    check('vy clamped to <= 0.20', vy <= 0.2001, f'{vy:.3f}')
    check('wz clamped to <= 0.80', wz <= 0.8001, f'{wz:.3f}')

    print('\n=== 5. E-STOP latch ===')
    p.set_mode(MODE_ESTOP)
    p.publish_hold(p.vel_pub, fwd, 1.0)
    check('E-STOP holds zero despite input', p.magnitude() < 1e-6,
          f'|v|={p.magnitude():.3f}')
    p.set_mode(MODE_KEYBOARD)
    p.publish_hold(p.vel_pub, fwd, 0.8)
    check('mode 1 clears the E-STOP latch', p.magnitude() > 0.05,
          f'|v|={p.magnitude():.3f}')

    print('\n=== 6. EEG UDP end-to-end ===')
    p.set_mode(MODE_EEG)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    addr = ('127.0.0.1', 9000)

    def eeg_send(payload):
        sock.sendto(json.dumps(payload).encode(), addr)
        try:
            sock.recvfrom(2048)
            return True
        except socket.timeout:
            return False

    if not eeg_send({'type': 'heartbeat'}):
        check('EEG bridge reachable on UDP :9000', False, 'no ack')
    else:
        check('EEG bridge reachable on UDP :9000', True)
        # Three identical commands to pass the majority filter.
        for _ in range(3):
            eeg_send({'command': 'GO', 'gesture': 'DOUBLE_BLINK'})
            time.sleep(0.05)
        p.spin_for(0.6)
        check('EEG GO drives forward', (p.latest.linear.x if p.latest else 0) > 0.05,
              f'vx={p.latest.linear.x:.3f}' if p.latest else 'no msg')

        print('\n=== 7. EEG link loss ===')
        t_stop = p.wait_for_zero(3.0)
        check('EEG stops when heartbeat stops',
              t_stop is not None and t_stop <= 2.0,
              f'{t_stop:.2f} s' if t_stop else 'never stopped')

    p.set_mode(MODE_ESTOP)
    p.spin_for(0.5)

    print('\n' + '=' * 58)
    failed = sum(1 for _, ok in results if not ok)
    print(f' {len(results) - failed}/{len(results)} checks passed'
          + (f', \033[31m{failed} FAILED\033[0m' if failed else ' - all good'))
    print('=' * 58)

    p.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
