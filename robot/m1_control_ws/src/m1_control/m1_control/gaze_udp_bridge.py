#!/usr/bin/env python3
"""UDP gaze receiver: Windows gaze app -> /gaze/point.

The GazeModel app (github.com/lANXEZ/GazeModel) runs natively on Windows: it
needs the webcam, a GUI, and the physical display it calibrates against. Windows
has no ROS 2 here, so it sends one small JSON datagram per gaze sample and this
bridge turns it into a normal ROS topic.

    GazeModel (Windows) --UDP:9100--> gaze_udp_bridge --/gaze/point--> gaze_control_node

Wire format (one datagram per sample, ~30-60/s)
-----------------------------------------------
    {"type": "gaze", "u": 0.42, "v": 0.61, "valid": true, "model": "deep+iris"}
    {"type": "heartbeat"}

`u`,`v` are normalized 0..1 with (0,0) at the top-left of the screen. Plain
text `"0.42 0.61"` is also accepted, so a tracker can be tested with netcat.

Safety
------
* Heartbeat watchdog: if the app stops sending (crash, sleep, Wi-Fi loss) this
  node stops publishing. `gaze_control_node` then hits its own link timeout and
  zeroes the velocity, so the robot stops rather than driving on stale data.
* Invalid samples (blink, face lost) are forwarded with valid=false so the
  control node can hold still without losing the link.

    ros2 run m1_control gaze_udp_bridge
    ros2 run m1_control gaze_udp_bridge --ros-args -p port:=9100

Dry run, no ROS (to prove the Windows side is sending):
    python3 gaze_udp_bridge.py --dry-run
"""

import argparse
import json
import socket
import threading
import time

DEFAULT_PORT = 9100


class GazeUdpCore:
    """Protocol + watchdog state. No ROS, so it is testable standalone."""

    def __init__(self, link_timeout=1.0):
        self.link_timeout = link_timeout
        self.last_packet = None
        self.last_heartbeat = None
        self.u = 0.5
        self.v = 0.5
        self.valid = False
        self.model = ''
        self.samples = 0
        self.lock = threading.Lock()

    def on_packet(self, msg, now):
        with self.lock:
            self.last_packet = now
            self.samples += 1
            kind = str(msg.get('type', 'gaze')).lower()
            if kind == 'heartbeat':
                self.last_heartbeat = now
                return
            try:
                self.u = min(1.0, max(0.0, float(msg['u'])))
                self.v = min(1.0, max(0.0, float(msg['v'])))
            except (KeyError, TypeError, ValueError):
                return
            self.valid = bool(msg.get('valid', True))
            self.model = str(msg.get('model', ''))

    def state(self, now):
        """(u, v, valid, alive) after applying the link timeout."""
        with self.lock:
            if self.last_packet is None:
                return self.u, self.v, False, False
            alive = (now - self.last_packet) <= self.link_timeout
            return self.u, self.v, self.valid and alive, alive


def udp_loop(core, port, stop_event, log=print):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(0.1)
    log(f'[gaze] listening on UDP :{port}')
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
            if not isinstance(msg, dict):
                raise ValueError
        except (ValueError, UnicodeDecodeError):
            # Plain text "u v" fallback
            parts = data.decode(errors='ignore').split()
            if len(parts) >= 2:
                msg = {'type': 'gaze', 'u': parts[0], 'v': parts[1]}
            else:
                continue
        if addr != peer:
            log(f'[gaze] source connected from {addr[0]}:{addr[1]}')
            peer = addr
        core.on_packet(msg, now)
    sock.close()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--link-timeout', type=float, default=1.0)
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--dry-run', action='store_true')
    args, _ = ap.parse_known_args(argv)

    core = GazeUdpCore(args.link_timeout)
    stop_event = threading.Event()
    threading.Thread(target=udp_loop, args=(core, args.port, stop_event),
                     daemon=True).start()

    if args.dry_run:
        print('[gaze] dry-run: printing samples (Ctrl+C to stop)')
        try:
            while True:
                u, v, valid, alive = core.state(time.monotonic())
                print(f'[gaze] u={u:.3f} v={v:.3f} valid={valid} alive={alive} '
                      f'model={core.model} n={core.samples}')
                time.sleep(0.5)
        except KeyboardInterrupt:
            stop_event.set()
        return

    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node

    class GazeUdpBridge(Node):

        def __init__(self):
            super().__init__('gaze_udp_bridge')
            self.pub = self.create_publisher(Point, '/gaze/point', 10)
            self.was_alive = False
            self.create_timer(1.0 / args.rate, self.tick)
            self.get_logger().info(
                f'Gaze UDP -> /gaze/point @ {args.rate:.0f} Hz | '
                f'port {args.port} | link timeout {args.link_timeout}s')

        def tick(self):
            u, v, valid, alive = core.state(time.monotonic())
            if alive != self.was_alive:
                if alive:
                    self.get_logger().info('Gaze source online')
                else:
                    self.get_logger().warn('Gaze source lost - no output')
                self.was_alive = alive

            if not alive:
                # Publish nothing: gaze_control_node's own link timeout then
                # zeroes the velocity. Silence is the safe signal.
                return

            msg = Point()
            if valid:
                msg.x, msg.y = float(u), float(v)
            else:
                # Blink / face briefly lost: hold at the deadzone centre so the
                # chassis stays still and the PTZ holds its angle, instead of
                # letting the link time out and log a warning every blink.
                msg.x, msg.y = 0.5, 0.5
            msg.z = 0.0
            self.pub.publish(msg)

    rclpy.init()
    node = GazeUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
