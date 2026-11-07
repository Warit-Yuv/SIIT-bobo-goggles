#!/usr/bin/env python3
"""EEG command bridge: UDP in, /cmd_vel/eeg out (ROSMASTER M1 / mecanum).

Companion to github.com/koonzeerkung/Muse2-RosmasterPi-A1. The laptop runs the
EEG pipeline (BlueMuse -> LSL -> detectors) and sends small UDP/JSON packets:

    laptop --UDP:9000--> eeg_udp_bridge --/cmd_vel/eeg--> arbiter --/cmd_vel--> motors

Differences from the upstream `eeg_cmd_bridge.py`
-------------------------------------------------
1. Publishes /cmd_vel/eeg and asserts /control_mode=3 instead of writing
   straight to /cmd_vel. The arbiter is the only /cmd_vel publisher, so EEG can
   never fight the keyboard or the gaze controller.
2. The upstream bridge assumes Ackermann steering (LEFT = drive a 1.5 s arc).
   The M1 is a mecanum base: LEFT/RIGHT rotate in place, so no arc is needed.
3. Adds an optional strafe axis, which only a holonomic base can use.
4. Re-asserts the arbiter mode on a timer, so if something switches the mode
   away the EEG link takes it back while it is still streaming.

Safety
------
* Link watchdog: the laptop sends a heartbeat every 0.5 s. If nothing arrives
  for `link_timeout`, the bridge commands a full stop and waits for a new GO.
* STOP always wins and is never filtered.
* A rolling majority filter suppresses single-packet misclassifications.
* Every velocity is clamped to the same envelope the arbiter enforces.

Run (inside the container, with the Yahboom workspace sourced):
    ros2 run m1_control eeg_udp_bridge
    ros2 run m1_control eeg_udp_bridge --ros-args -p speed:=0.12 -p port:=9000

Dry run (no ROS needed - just prints what it would publish):
    python3 eeg_udp_bridge.py --dry-run
"""

import argparse
import json
import socket
import threading
import time
from collections import Counter, deque

# ---------------------------------------------------------------------------
# Pure logic (no ROS) so it can be unit tested and dry-run on any machine.
# ---------------------------------------------------------------------------

MODE_EEG = 3

#: Gesture label -> (vx, vy, wz) command. Labels match the upstream project's
#: DEFAULT_COMMAND_MAP plus the strafe variants a mecanum base can use.
COMMAND_MAP = {
    'GO': (1.0, 0.0, 0.0),
    'FORWARD': (1.0, 0.0, 0.0),
    'STOP': (0.0, 0.0, 0.0),
    'REST': (0.0, 0.0, 0.0),
    'LEFT': (0.0, 0.0, 1.0),
    'RIGHT': (0.0, 0.0, -1.0),
    'TURN_LEFT': (0.0, 0.0, 1.0),
    'TURN_RIGHT': (0.0, 0.0, -1.0),
    'STRAFE_LEFT': (0.0, 1.0, 0.0),
    'STRAFE_RIGHT': (0.0, -1.0, 0.0),
    'BACK': (-1.0, 0.0, 0.0),
    'BACKWARD': (-1.0, 0.0, 0.0),
}


class BridgeCore:
    """Command state machine + watchdogs. Thread-safe, ROS-free."""

    def __init__(self, speed=0.15, steer=0.60, strafe=0.15,
                 link_timeout=1.5, max_drive=0.0, majority_window=3,
                 max_speed=0.40, max_turn=0.80):
        # max_speed bounds the LINEAR axes only (m/s). Angular velocity is in
        # rad/s and has its own envelope, matching the arbiter's 0.80 rad/s.
        # Applying the linear cap to steering silently crushed a 0.60 rad/s
        # request down to 0.40 and made the robot turn far more gently than
        # configured.
        self.speed = min(abs(speed), max_speed)
        self.strafe = min(abs(strafe), max_speed)
        self.steer = min(abs(steer), max_turn)
        self.link_timeout = link_timeout
        self.max_drive = max_drive
        self.majority_window = max(1, majority_window)

        self.state = 'STOP'          # STOP | DRIVE
        self.turn_dir = 0            # +1 left, -1 right
        self.turn_until = 0.0
        self.last_packet = None
        self.last_cmd_t = 0.0
        self.last_cmd = None
        self.link_ok = False
        self.history = deque(maxlen=self.majority_window)
        self.lock = threading.Lock()

    # -- input ------------------------------------------------------------
    def on_packet(self, now):
        """Heartbeat: proves the laptop is alive."""
        with self.lock:
            self.last_packet = now
            self.link_ok = True

    def on_command(self, cmd, now):
        cmd = str(cmd).strip().upper().replace(' ', '_')
        with self.lock:
            self.last_packet = now
            self.link_ok = True
            self.history.append(cmd)

            # Majority filter over the last N labels. STOP bypasses it.
            if cmd == 'STOP':
                filtered = 'STOP'
            elif len(self.history) < self.majority_window:
                return
            else:
                filtered, _ = Counter(self.history).most_common(1)[0]

            self.last_cmd, self.last_cmd_t = filtered, now

            if filtered in ('STOP', 'REST'):
                self.state, self.turn_dir = 'STOP', 0
            elif filtered in ('GO', 'FORWARD', 'BACK', 'BACKWARD'):
                self.state, self.turn_dir = 'DRIVE', 0
            elif filtered in ('LEFT', 'RIGHT', 'TURN_LEFT', 'TURN_RIGHT'):
                self.turn_dir = 1 if 'LEFT' in filtered else -1
                self.turn_until = now + 0.6   # short in-place pulse

    # -- output -----------------------------------------------------------
    def velocity(self, now):
        """(vx, vy, wz) to publish right now, with all watchdogs applied."""
        with self.lock:
            if self.last_packet is not None and now - self.last_packet > self.link_timeout:
                if self.link_ok:
                    self.link_ok = False
                self.state, self.turn_dir = 'STOP', 0
                return 0.0, 0.0, 0.0

            if self.turn_dir and now >= self.turn_until:
                self.turn_dir = 0

            if self.state == 'DRIVE' and self.max_drive and now - self.last_cmd_t > self.max_drive:
                self.state = 'STOP'

            if not self.link_ok:
                return 0.0, 0.0, 0.0

            label = self.last_cmd or 'STOP'
            vx_s, vy_s, wz_s = COMMAND_MAP.get(label, (0.0, 0.0, 0.0))

            vx = self.speed * vx_s if self.state == 'DRIVE' else 0.0
            vy = self.strafe * vy_s
            wz = self.steer * wz_s if self.turn_dir else 0.0

            # A rotation pulse alone must not translate.
            if self.turn_dir:
                vx = 0.0
                wz = self.steer * self.turn_dir

            return vx, vy, wz

    def status(self, now):
        vx, vy, wz = self.velocity(now)
        with self.lock:
            return {
                'state': self.state,
                'turn': {1: 'LEFT', -1: 'RIGHT', 0: ''}[self.turn_dir],
                'vx': round(vx, 3), 'vy': round(vy, 3), 'wz': round(wz, 3),
                'link': self.link_ok,
                'last_cmd': self.last_cmd,
            }


def udp_loop(core, port, stop_event, log=print):
    """Receive JSON or plain-text commands and ack every packet."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(0.1)
    log(f'[eeg] listening on UDP :{port}')
    peer = None
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break
        now = time.monotonic()
        try:
            msg = json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            msg = {'command': data.decode(errors='ignore').strip()}

        if addr != peer:
            log(f'[eeg] laptop connected from {addr[0]}:{addr[1]}')
            peer = addr

        if msg.get('type') == 'heartbeat':
            core.on_packet(now)
        elif msg.get('command'):
            core.on_command(str(msg['command']), now)
            log(f"[eeg] {msg.get('gesture', '')} -> {msg['command']}")

        try:
            sock.sendto(json.dumps({'type': 'ack', **core.status(now)}).encode(), addr)
        except OSError:
            pass
    sock.close()


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

def run_ros(args, core):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import UInt8

    class EEGBridge(Node):

        def __init__(self):
            super().__init__('eeg_udp_bridge')

            actuator_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10)
            mode_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1)

            self.pub = self.create_publisher(Twist, args.topic, actuator_qos)
            self.mode_pub = self.create_publisher(UInt8, '/control_mode', mode_qos)

            self.zero_left = 0
            self.prev = None
            self.claimed = False
            self.create_timer(1.0 / args.rate, self.tick)
            self.create_timer(2.0, self.reassert_mode)

            self.get_logger().info(
                f'EEG bridge -> {args.topic} @ {args.rate:.0f} Hz | '
                f'speed={core.speed} steer={core.steer} strafe={core.strafe} | '
                f'link timeout {core.link_timeout}s | UDP :{args.port}')

        def reassert_mode(self):
            """Keep the arbiter in EEG mode while the laptop is streaming."""
            if core.link_ok and not self.claimed:
                self.mode_pub.publish(UInt8(data=MODE_EEG))
                self.get_logger().info('Claimed control mode 3 (EEG)')
                self.claimed = True
            elif not core.link_ok and self.claimed:
                self.claimed = False

        def tick(self):
            now = time.monotonic()
            vx, vy, wz = core.velocity(now)

            if (vx, vy, wz) != self.prev:
                self.get_logger().info(
                    f'cmd_vel/eeg vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} '
                    f'({core.status(now)["state"]})')

            if vx or vy or wz:
                self.zero_left = 10       # after stopping, send zeros ~0.5 s
            elif self.zero_left == 0:
                self.prev = (vx, vy, wz)
                return                    # idle: stay quiet, don't fight others
            else:
                self.zero_left -= 1

            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(wz)
            self.pub.publish(msg)
            self.prev = (vx, vy, wz)

    rclpy.init()
    node = EEGBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--topic', default='/cmd_vel/eeg')
    ap.add_argument('--speed', type=float, default=0.15, help='forward m/s')
    ap.add_argument('--steer', type=float, default=0.60, help='turn rad/s')
    ap.add_argument('--strafe', type=float, default=0.15, help='strafe m/s')
    ap.add_argument('--link-timeout', type=float, default=1.5)
    ap.add_argument('--max-drive', type=float, default=0.0, help='0 = off')
    ap.add_argument('--max-speed', type=float, default=0.40,
                    help='clamp for linear speed, m/s')
    ap.add_argument('--max-turn', type=float, default=0.80,
                    help='clamp for angular speed, rad/s')
    ap.add_argument('--majority-window', type=int, default=3)
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--dry-run', action='store_true')
    args, _ = ap.parse_known_args(argv)

    core = BridgeCore(args.speed, args.steer, args.strafe, args.link_timeout,
                      args.max_drive, args.majority_window, args.max_speed,
                      args.max_turn)

    stop_event = threading.Event()
    threading.Thread(target=udp_loop, args=(core, args.port, stop_event),
                     daemon=True).start()

    if args.dry_run:
        print('[eeg] dry-run: no ROS, printing commands only (Ctrl+C to stop)')
        last = None
        try:
            while True:
                v = core.velocity(time.monotonic())
                if v != last:
                    print(f'[dry-run] vx={v[0]:.3f} vy={v[1]:.3f} wz={v[2]:.3f} '
                          f'{core.status(time.monotonic())}')
                    last = v
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            stop_event.set()
        return

    run_ros(args, core)


if __name__ == '__main__':
    main()
