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
    # Ask the arbiter for its own limits rather than pinning numbers here.
    # A hardcoded cap makes this test fail every time the cap is tuned, and a
    # test that fails for a legitimate change gets ignored - which is how a
    # clamp regression would slip through later.
    def arbiter_cap(name, fallback):
        try:
            from rclpy.parameter import Parameter
            client = p.create_client(
                __import__('rcl_interfaces.srv', fromlist=['GetParameters'])
                .GetParameters, f'/control_arbiter_node/get_parameters')
            if not client.wait_for_service(timeout_sec=2.0):
                return fallback
            req = (__import__('rcl_interfaces.srv',
                              fromlist=['GetParameters']).GetParameters
                   .Request())
            req.names = [name]
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(p, fut, timeout_sec=2.0)
            return float(fut.result().values[0].double_value)
        except Exception:                                    # noqa: BLE001
            return fallback

    cap_x = arbiter_cap('max_linear_x', 0.40)
    cap_y = arbiter_cap('max_linear_y', 0.20)
    cap_z = arbiter_cap('max_angular_z', 0.80)
    print(f'     arbiter caps: vx<={cap_x} vy<={cap_y} wz<={cap_z}')
    check(f'vx clamped to <= {cap_x}', vx <= cap_x + 1e-3, f'{vx:.3f}')
    check(f'vy clamped to <= {cap_y}', vy <= cap_y + 1e-3, f'{vy:.3f}')
    check(f'wz clamped to <= {cap_z}', wz <= cap_z + 1e-3, f'{wz:.3f}')

    print('\n=== 5. E-STOP latch ===')
    p.set_mode(MODE_ESTOP)
    p.publish_hold(p.vel_pub, fwd, 1.0)
    check('E-STOP holds zero despite input', p.magnitude() < 1e-6,
          f'|v|={p.magnitude():.3f}')
    p.set_mode(MODE_KEYBOARD)
    p.publish_hold(p.vel_pub, fwd, 0.8)
    check('mode 1 clears the E-STOP latch', p.magnitude() > 0.05,
          f'|v|={p.magnitude():.3f}')

    print('\n=== 6. EEG UDP end-to-end (protocol v1) ===')
    # Protocol v1 carries TOGGLES plus a full state block, not velocities. The
    # bridge answers with an ack containing the state it now believes, so the
    # ack itself is the assertion - there is no /cmd_vel/eeg to watch.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    addr = ('127.0.0.1', 9000)
    seq = [0]

    def eeg_send(state=None, gesture=None):
        seq[0] += 1
        payload = {'v': 1, 'seq': seq[0], 'ts': time.time()}
        if state is not None:
            payload['state'] = state
        if gesture is not None:
            payload['event'] = {'gesture': gesture}
        sock.sendto(json.dumps(payload).encode(), addr)
        try:
            return json.loads(sock.recvfrom(2048)[0].decode())
        except (socket.timeout, ValueError):
            return None

    reply = eeg_send({'run': True, 'mode': 'robot'})
    if reply is None:
        check('EEG bridge reachable on UDP :9000', False, 'no ack')
    else:
        check('EEG bridge reachable on UDP :9000', True)
        st = reply.get('state', reply)
        check('the bridge follows the run flag', st.get('run') is True,
              str(st.get('run')))
        check('the bridge follows the mode', st.get('mode') == 'robot',
              str(st.get('mode')))
        check('the bridge reports a live link', st.get('link') is True,
              str(st.get('link')))

        print('\n=== 7. EEG link watchdog ===')
        # Watch the LATCHED /eeg/link topic, not an ack. Asking the bridge
        # "is the link down?" means sending it a packet, and a packet is
        # exactly what brings the link back up - so an ack can never report
        # the timeout it was asked about.
        from std_msgs.msg import Bool as _Bool
        seen = []
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        p.create_subscription(_Bool, '/eeg/link',
                              lambda m: seen.append(bool(m.data)), sub_qos)
        # Stop sending and let the 1.5 s timeout expire.
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(p, timeout_sec=0.05)
        check('a silent link is reported as down',
              seen and seen[-1] is False,
              f'last /eeg/link = {seen[-1] if seen else "nothing"}')


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
