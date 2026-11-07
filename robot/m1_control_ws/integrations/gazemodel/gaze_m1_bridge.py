"""Publish GazeModel's gaze point to the ROSMASTER M1.

Drop this file into the GazeModel repo as `gaze/m1_bridge.py`, then add the
three marked lines to `gaze/app.py` (see the patch section at the bottom of this
file's docstring, or remote.md).

Why a bridge instead of publishing ROS from the app
--------------------------------------------------
GazeModel is a PySide6 desktop app whose output is a pixel coordinate on the
*physical* display. It must therefore run where the operator's eyes and monitor
are. Two transports cover both realistic setups:

  ``UdpGazeSender``   the app runs natively on Windows, no ROS installed.
                      Sends JSON to the Pi, where `gaze_udp_bridge` turns it
                      into `/gaze/point`. This is the default and the
                      recommended path.

  ``RosGazeSender``   the app runs inside the Ubuntu VM, which already has
                      ROS 2 Humble. Publishes `/gaze/point` directly over Wi-Fi.

Both produce the same message, so `gaze_control_node` on the robot does not
care which one is in use.

Coordinates
-----------
`u`, `v` are normalized to 0..1 with (0,0) at the TOP-LEFT of the screen.
GazeModel reports pixels on the primary screen, so the bridge divides by the
screen size. Normalizing here means the robot side never needs to know the
operator's resolution - important because calibration happens at one resolution
and the app may be resized later.
"""

from __future__ import annotations

import json
import socket
import time

DEFAULT_UDP_PORT = 9100
DEFAULT_ROS_TOPIC = '/gaze/point'

#: How often to send a heartbeat when no gaze sample is available, so the robot
#: does not treat a blink or a face-loss as a dropped link.
HEARTBEAT_INTERVAL_S = 0.25


class UdpGazeSender:
    """Send normalized gaze to the robot over UDP/JSON."""

    def __init__(self, host: str, port: int = DEFAULT_UDP_PORT,
                 screen_size: tuple[int, int] | None = None):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.screen_size = screen_size
        self._last_hb = 0.0
        self._sent = 0
        self._errors = 0

    # -- lifecycle -------------------------------------------------------
    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    # -- sending ---------------------------------------------------------
    def send_pixels(self, x: float, y: float, model: str = '',
                    valid: bool = True) -> None:
        """Send a gaze point given in SCREEN PIXELS."""
        w, h = self.screen_size or (0, 0)
        if w <= 0 or h <= 0:
            raise ValueError(
                'screen_size is required to convert pixels to normalized '
                'coordinates; pass it to the constructor')
        self.send_normalized(x / w, y / h, model=model, valid=valid)

    def send_normalized(self, u: float, v: float, model: str = '',
                        valid: bool = True) -> None:
        """Send a gaze point already normalized to 0..1."""
        msg = {
            'type': 'gaze',
            'u': round(max(0.0, min(1.0, float(u))), 4),
            'v': round(max(0.0, min(1.0, float(v))), 4),
            'valid': bool(valid),
            'model': model,
            't': time.time(),
        }
        self._send(msg)
        self._sent += 1

    def heartbeat(self) -> None:
        """Keep the robot's link alive while the eyes are closed or lost."""
        now = time.monotonic()
        if now - self._last_hb < HEARTBEAT_INTERVAL_S:
            return
        self._last_hb = now
        self._send({'type': 'heartbeat'})

    def _send(self, payload: dict) -> None:
        try:
            self.sock.sendto(json.dumps(payload).encode(), self.addr)
        except OSError:
            # Never let a network hiccup kill the gaze app; the robot's own
            # watchdog handles a sustained outage.
            self._errors += 1

    # -- diagnostics -----------------------------------------------------
    def stats(self) -> str:
        return f'UDP {self.addr[0]}:{self.addr[1]} sent={self._sent} err={self._errors}'


class RosGazeSender:
    """Publish normalized gaze on /gaze/point from inside the ROS VM."""

    def __init__(self, topic: str = DEFAULT_ROS_TOPIC,
                 screen_size: tuple[int, int] | None = None):
        import rclpy
        from geometry_msgs.msg import Point
        from rclpy.node import Node

        self.screen_size = screen_size
        self._Point = Point
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('gazemodel_publisher')
        self._pub = self._node.create_publisher(Point, topic, 10)
        self._sent = 0

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass

    def send_pixels(self, x: float, y: float, model: str = '',
                    valid: bool = True) -> None:
        w, h = self.screen_size or (0, 0)
        if w <= 0 or h <= 0:
            raise ValueError('screen_size is required to normalize pixels')
        self.send_normalized(x / w, y / h, model=model, valid=valid)

    def send_normalized(self, u: float, v: float, model: str = '',
                        valid: bool = True) -> None:
        msg = self._Point()
        msg.x = float(max(0.0, min(1.0, u)))
        msg.y = float(max(0.0, min(1.0, v)))
        msg.z = 0.0
        self._pub.publish(msg)
        self._sent += 1
        # Spin briefly so the sample actually leaves the node.
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def heartbeat(self) -> None:
        return

    def stats(self) -> str:
        return f'ROS /gaze/point sent={self._sent}'


def make_sender(kind: str = 'udp', host: str = '10.0.0.54',
                port: int = DEFAULT_UDP_PORT,
                screen_size: tuple[int, int] | None = None):
    """Factory used by the app patch.

    kind: 'udp' (Windows) | 'ros' (VM) | 'none' (disable)
    """
    kind = (kind or 'none').lower()
    if kind == 'udp':
        return UdpGazeSender(host, port, screen_size)
    if kind == 'ros':
        return RosGazeSender(screen_size=screen_size)
    return None


# ===========================================================================
# PATCH - add these lines to gaze/app.py
# ===========================================================================
# 1) At the top, with the other imports:
#
#        from .m1_bridge import make_sender
#
# 2) In GazeApp.__init__, after `self.screen = QGuiApplication.primaryScreen()`
#    and before `on_face` can fire:
#
#        # --- ROSMASTER M1 link ---
#        _geo = self.screen.geometry()
#        self.m1 = make_sender(
#            kind=M1_TRANSPORT,                  # 'udp' | 'ros' | 'none'
#            host=M1_HOST,                       # robot IP
#            screen_size=(_geo.width(), _geo.height()),
#        )
#
# 3) Inside on_face, immediately after the `x, y = self.filter(...)` line and
#    before `self.pointer.move_centre_to(...)`:
#
#        if self.m1 is not None:
#            self.m1.send_pixels(x, y, model=self.active)
#
#    and where the pointer is hidden because the face was lost, add:
#
#        if self.m1 is not None:
#            self.m1.heartbeat()
#
# 4) At the top of app.py, define the two settings (edit the IP):
#
#        M1_TRANSPORT = 'udp'      # 'udp' for Windows, 'ros' inside the VM
#        M1_HOST = '10.0.0.54'     # robot IP address
#
# The patch is deliberately tiny: it hooks the one place where GazeModel already
# has a final, smoothed, calibrated screen coordinate, so no gaze math is
# duplicated and a future model swap needs no change on the robot side.
