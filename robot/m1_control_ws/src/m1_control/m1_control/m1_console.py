#!/usr/bin/env python3
"""ROSMASTER M1 operator console - one window for everything.

Built for the project's actual user: someone operating the robot on behalf of a
person who cannot leave their bed. That shapes every choice here.

    ros2 run m1_control m1_console
    ~/m1_console.sh                    # sets the DDS env for you (recommended)

Why a single window
-------------------
Switching modes, driving, hearing the room and watching the map used to be
spread across four terminals and a `ros2 topic pub` command. When something
goes wrong you need one place to look and one button to press.

    [ ROBOT ]  start / stop / diagnose the stack on the Pi
    [ MODE  ]  keyboard / gaze / eeg / gaze+eeg / E-STOP
    [ SERVO ]  head pan + tilt, with the allowed range drawn on the bar
    [ VOICE ]  hear the room, speak to it, per-direction mute
    [ VIEW  ]  launch SLAM + RViz, camera feed
    [ CALIB ]  five-point gaze calibration
    [ STATE ]  battery, video rate, gaze zone, DDS health, link health

Keyboard teleop from the GUI
----------------------------
The most-asked question was "where do I type?". The answer is: here.

The window grabs WASD/QE/A/D and publishes `/cmd_vel/keyboard` directly, so it
works while the gaze or EEG node is running - the arbiter lets a recent
keyboard command win for 0.4 s regardless of mode. No terminal, no focus
juggling.

    W/S forward-back   Q/E strafe   A/D rotate
    I/K tilt           J/L pan      U centre
    0 E-STOP           1..4 mode    Space E-STOP

Holding a key repeats. Releasing stops within one publish period (50 ms),
because the teleop loop publishes zero when no key is down; the arbiter's
350 ms watchdog is a second safety net behind that.

Why the servo range is narrow
-----------------------------
Measured on the hardware with the camera as the sensor: pan (s2) moves freely
and linearly from 0 to about 112 and then stops dead - the STM32 firmware
clamps it. So a centre of 90 gives 90 units of travel one way and only 20 the
other, which is exactly the asymmetry an operator notices as "it turns further
right than left".

The window is deliberately narrowed to 80..100 (+-10). A camera that swings
90 degrees loses the view straight ahead, and driving forward blind is
dangerous. The bar in the SERVO panel draws the allowed range, so the limit is
visible rather than mysterious.

Threading model
---------------
ROS spins in a background thread; Qt owns the main thread. Nothing in a ROS
callback touches a widget - they only update a plain dict that a QTimer reads.
That keeps the UI responsive and avoids Qt's cross-thread rules.
"""

import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8

MODE_NAMES = {0: 'E-STOP', 1: 'KEYBOARD', 2: 'GAZE', 3: 'EEG', 4: 'GAZE+EEG'}
MODE_KEYS = {'0': 0, '1': 1, '2': 2, '3': 3, '4': 4}

# Expected DDS environment. A mismatch does not raise an error - it silently
# never discovers the robot, which is far more confusing than a hard failure.
EXPECTED_DOMAIN = '30'
EXPECTED_RMW = 'rmw_cyclonedds_cpp'

# TF lookups must not block the GUI. The transform is only used to draw a
# robot marker, so "the previous frame's pose" is a perfectly good answer -
# waiting even 100 ms for a fresh one would stutter the window.
#
# This is a Duration, not a float: tf2's can_transform() compares the timeout
# against Duration() and raises TypeError on a bare number.
_TF_TIMEOUT_S = 0.02

STYLE = """
QWidget {
    background: #14171c; color: #e6e6e6;
    font-family: 'DejaVu Sans'; font-size: 12px;
}
QGroupBox {
    border: 1px solid #2b3138; border-radius: 6px; margin-top: 11px;
    padding: 12px 10px 10px 10px; font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 10px; padding: 0 5px;
    color: #8ab4f8; font-size: 11px; letter-spacing: 0.6px;
}
QPushButton {
    background: #232830; border: 1px solid #333b45; border-radius: 5px;
    padding: 7px 10px; text-align: left;
}
QPushButton:hover { background: #2c333d; border-color: #3f4954; }
QPushButton:pressed { background: #1a1f26; }
QPushButton:disabled { color: #555b63; border-color: #242a31; background: #1a1e24; }
QPushButton#mode {
    text-align: center; font-weight: bold; padding: 8px 6px; font-size: 12px;
}
QPushButton#mode:checked {
    background: #1e3a5f; border: 1px solid #3b82f6; color: #dbeafe;
}
QPushButton#estop {
    background: #7f1d1d; border: 1px solid #b91c1c;
    font-size: 14px; font-weight: bold; text-align: center; padding: 12px;
}
QPushButton#estop:hover { background: #991b1b; border-color: #dc2626; }
QPushButton#estop:pressed { background: #641414; }
QPushButton#primary { background: #1d4ed8; border-color: #2563eb; text-align: center; }
QPushButton#toggle {
    text-align: center; font-weight: bold; padding: 7px 4px; font-size: 12px;
}
QPushButton#toggle[state="on"] {
    background: #14532d; border: 1px solid #16a34a; color: #bbf7d0;
}
QPushButton#toggle[state="on"]:hover { background: #166534; }
QPushButton#toggle[state="bad"] {
    background: #4c1d1d; border: 1px solid #991b1b; color: #fecaca;
}
QPushButton#toggle[state="bad"]:hover { background: #5f2626; }
QPushButton#toggle[state="grey"] {
    background: #262b33; border: 1px solid #3f4954; color: #7b838c;
}
QPushButton#toggle[state="grey"]:hover { background: #2f353e; }
QPushButton#primary:hover { background: #2563eb; }
QCheckBox { padding: 2px; }
QCheckBox::indicator {
    width: 15px; height: 15px; border-radius: 3px;
    border: 1px solid #3f4954; background: #1a1e24;
}
QCheckBox::indicator:checked { background: #2563eb; border-color: #3b82f6; }
QLabel#big { font-size: 19px; font-weight: bold; }
QLabel#h1 { font-size: 15px; font-weight: bold; }
QLabel#dim { color: #8b939c; }
QLabel#mono { font-family: 'DejaVu Sans Mono'; font-size: 11px; }
QLabel#warn { color: #fbbf24; }
QLabel#bad { color: #ef4444; }
QLabel#good { color: #34d399; }
QLabel#banner {
    background: #3f2d0a; border: 1px solid #a16207; border-radius: 5px;
    color: #fbbf24; padding: 7px 10px;
}
QPlainTextEdit {
    background: #0d1014; border: 1px solid #2b3138; border-radius: 5px;
    font-family: 'DejaVu Sans Mono'; font-size: 11px; color: #b9c2cc;
}
QSpinBox, QDoubleSpinBox {
    background: #1a1e24; border: 1px solid #333b45; border-radius: 4px;
    padding: 3px 6px;
}
QScrollArea { border: none; }
QSplitter::handle { background: #1d2229; width: 4px; }
"""


# ---------------------------------------------------------------------------
# ROS interface
# ---------------------------------------------------------------------------

class ConsoleNode(Node):
    """Publishes commands, collects state into `state` for the GUI to read."""

    def __init__(self):
        super().__init__('m1_console')
        self.declare_parameter('robot_ip', '10.0.0.54')
        self.declare_parameter('publish_rate_hz', 20.0)
        # Servo geometry. These MUST agree with the arbiter, otherwise the GUI
        # lets the operator ask for travel the robot will silently refuse.
        # Measured on the hardware: pan (s2) moves freely 0..112 then stops
        # dead (firmware clamp). We use 80..100 (+-10) because a camera that
        # swings ~90 degrees loses the view straight ahead.
        self.declare_parameter('servo_s1_center', 40.0)
        self.declare_parameter('servo_s2_center', 90.0)
        self.declare_parameter('servo_s1_min', 5.0)
        self.declare_parameter('servo_s1_max', 75.0)
        self.declare_parameter('servo_s2_min', 80.0)
        self.declare_parameter('servo_s2_max', 100.0)
        self.declare_parameter('servo_step', 4.0)

        gp = self.get_parameter
        self.robot_ip = gp('robot_ip').value
        self.s1_center = float(gp('servo_s1_center').value)
        self.s2_center = float(gp('servo_s2_center').value)
        self.s1_min = float(gp('servo_s1_min').value)
        self.s1_max = float(gp('servo_s1_max').value)
        self.s2_min = float(gp('servo_s2_min').value)
        self.s2_max = float(gp('servo_s2_max').value)
        self.servo_step = float(gp('servo_step').value)

        self.state = {
            'mode': None, 'voltage': float('nan'), 'zone': '',
            'image_hz': 0.0, 'audio': '', 'connected': False,
            'gaze_alive': False, 'calib': [0.5, 0.5, 1.0, 1.0],
        }
        self._image_times = []
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._raw = None          # (u, v, valid, monotonic_time)

        mode_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              depth=1)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.mode_pub = self.create_publisher(UInt8, '/control_mode', mode_qos)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel/keyboard', 10)
        self.servo_pub = self.create_publisher(Vector3, '/servo/keyboard', 10)
        self.audio_en_pub = self.create_publisher(Bool, '/audio/enable', 10)
        # Named for the ROBOT's hardware. The old /audio/mic and
        # /audio/speaker were ambiguous: both bridges subscribed to both, and
        # "mic" meant "the mic at my end" to each of them, so one checkbox
        # muted two different things - see audio_bridge.py.
        self.audio_rmic_pub = self.create_publisher(
            Bool, '/audio/robot_mic', 10)
        self.audio_rspk_pub = self.create_publisher(
            Bool, '/audio/robot_speaker', 10)
        self.calib_pub = self.create_publisher(
            Float32MultiArray, '/gaze/calib', 10)

        self.create_subscription(UInt8, '/control_mode_state',
                                 lambda m: self._set('mode', int(m.data)),
                                 mode_qos)
        self.create_subscription(Float32, '/voltage',
                                 lambda m: self._set('voltage', float(m.data)),
                                 sensor_qos)
        self.create_subscription(String, '/gaze/zone',
                                 lambda m: self._set('zone', m.data), 10)
        self.create_subscription(String, '/audio/status',
                                 lambda m: self._set('audio', m.data), 10)
        self.create_subscription(Float32MultiArray, '/gaze/calib_state',
                                 self._calib_state_cb, 10)
        self.create_subscription(Point, '/gaze/raw', self._raw_cb, 10)
        self.create_subscription(CompressedImage, '/image_throttled/compressed',
                                 self._image_cb, sensor_qos)

        # ---- SLAM map ------------------------------------------------------
        # slam_toolbox publishes /map with TRANSIENT_LOCAL durability, which is
        # what lets a late-joining subscriber get the map that already exists.
        # Subscribing with the default VOLATILE would connect and then wait
        # forever for an update that may never come - the same class of bug as
        # a Reliable subscriber on a Best Effort topic.
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)

        self._map = None          # (width, height, res, ox, oy, bytes)
        self._scan = None         # (angles, ranges, range_max)
        self._map_stamp = 0.0
        self._scan_stamp = 0.0

        # TF, for the robot's pose in the map frame. Without it the map would
        # be drawn with no indication of where the robot actually is, which is
        # the one thing an operator needs from a map.
        try:
            import tf2_ros
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self, spin_thread=False)
            self._tf_ok = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'tf2 unavailable, map has no robot pose: {exc}')
            self.tf_buffer = None
            self._tf_ok = False

        # Teleop state: written by the GUI thread, read by the publish timer.
        self.teleop = Twist()
        self.servo = {'s1': self.s1_center, 's2': self.s2_center}
        self._servo_dirty = False

        rate = float(gp('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish_loop)
        self.get_logger().info(f'Console up (robot {self.robot_ip})')

    # ------------------------------------------------------------- callbacks
    def _set(self, key, value):
        with self._lock:
            self.state[key] = value
            self.state['connected'] = True

    def _calib_state_cb(self, msg):
        if len(msg.data) >= 4:
            self._set('calib', [float(v) for v in msg.data[:4]])

    def _raw_cb(self, msg):
        self._raw = (float(msg.x), float(msg.y), float(msg.z) > 0.5,
                     time.monotonic())
        self._set('gaze_alive', self._raw[2])

    def _image_cb(self, msg):
        now = time.monotonic()
        self._image_times.append(now)
        self._image_times = [t for t in self._image_times if now - t < 2.0]
        self._latest_jpeg = bytes(msg.data)
        self._set('connected', True)

    def _map_cb(self, msg):
        """Store the map as flat bytes so the GUI never touches ROS objects."""
        try:
            info = msg.info
            w, h = int(info.width), int(info.height)
            if w <= 0 or h <= 0 or len(msg.data) < w * h:
                return
            # -1 unknown, 0 free, 100 occupied. Pack into one byte each:
            # 0 = unknown, 1 = free, 2 = occupied.
            raw = bytes(msg.data)
            packed = bytearray(w * h)
            for i in range(w * h):
                v = raw[i]
                if v < 0:
                    packed[i] = 0
                elif v >= 50:
                    packed[i] = 2
                else:
                    packed[i] = 1
            self._map = (w, h, float(info.resolution),
                         float(info.origin.position.x),
                         float(info.origin.position.y),
                         bytes(packed))
            self._map_stamp = time.monotonic()
        except Exception:  # noqa: BLE001 - a malformed map must not kill the GUI
            pass

    def _scan_cb(self, msg):
        try:
            self._scan = (float(msg.angle_min), float(msg.angle_increment),
                          float(msg.range_max),
                          tuple(float(r) for r in msg.ranges))
            self._scan_stamp = time.monotonic()
        except Exception:  # noqa: BLE001
            pass

    def map_snapshot(self):
        """(map_tuple, scan_tuple, pose_or_None) for the GUI thread."""
        pose = None
        if self.tf_buffer is not None:
            try:
                from rclpy.duration import Duration
                from rclpy.time import Time
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', Time(),
                    timeout=Duration(seconds=_TF_TIMEOUT_S))
                t = tf.transform.translation
                q = tf.transform.rotation
                # yaw only; the robot is planar
                yaw = 2.0 * math.atan2(q.z, q.w)
                pose = (t.x, t.y, yaw)
            except Exception:  # noqa: BLE001 - no map->base_link yet
                pose = None
        return self._map, self._scan, pose

    # ------------------------------------------------------------- accessors
    def snapshot(self):
        with self._lock:
            s = dict(self.state)
            s['calib'] = list(self.state['calib'])
        n = len(self._image_times)
        s['image_hz'] = n / 2.0
        return s

    def latest_jpeg(self):
        return self._latest_jpeg

    def latest_raw(self):
        """(u, v, valid, age_seconds) or None if the bridge never spoke."""
        r = self._raw
        if r is None:
            return None
        return r[0], r[1], r[2], time.monotonic() - r[3]

    # -------------------------------------------------------------- commands
    def set_mode(self, mode):
        self.mode_pub.publish(UInt8(data=int(mode)))

    def estop(self):
        self.set_mode(0)
        self.teleop = Twist()
        self.vel_pub.publish(Twist())

    def audio(self, enable=None, robot_mic=None, robot_speaker=None):
        """Control the robot's audio hardware.

        robot_mic      the robot's microphone  -> audio travels robot -> me
        robot_speaker  the robot's speaker     -> audio travels me -> robot

        Named for the far end deliberately. "my mic" is a different thing on
        each machine, and that ambiguity is what made the two checkboxes
        appear swapped.
        """
        if enable is not None:
            self.audio_en_pub.publish(Bool(data=bool(enable)))
        if robot_mic is not None:
            self.audio_rmic_pub.publish(Bool(data=bool(robot_mic)))
        if robot_speaker is not None:
            self.audio_rspk_pub.publish(Bool(data=bool(robot_speaker)))

    def set_calibration(self, ox, oy, gx, gy):
        self.calib_pub.publish(
            Float32MultiArray(data=[float(ox), float(oy),
                                    float(gx), float(gy)]))

    def nudge_servo(self, d_tilt=0.0, d_pan=0.0):
        self.servo['s1'] = min(self.s1_max,
                               max(self.s1_min, self.servo['s1'] + d_tilt))
        self.servo['s2'] = min(self.s2_max,
                               max(self.s2_min, self.servo['s2'] + d_pan))
        self._servo_dirty = True

    def centre_servo(self):
        self.servo['s1'], self.servo['s2'] = self.s1_center, self.s2_center
        self._servo_dirty = True

    def _publish_loop(self):
        self.vel_pub.publish(self.teleop)
        if self._servo_dirty:
            msg = Vector3()
            msg.x = self.servo['s1']
            msg.y = self.servo['s2']
            self.servo_pub.publish(msg)
            self._servo_dirty = False


# ---------------------------------------------------------------------------
# Remote control of the robot stack
# ---------------------------------------------------------------------------

class RobotShell:
    """Run commands on the Pi over SSH without blocking the UI."""

    def __init__(self, ip, user='pi'):
        self.ip = ip
        self.user = user

    def _ssh(self, cmd, timeout=30):
        return subprocess.run(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
             f'{self.user}@{self.ip}', cmd],
            capture_output=True, text=True, timeout=timeout)

    def test(self):
        try:
            r = self._ssh('echo OK', timeout=10)
            return r.returncode == 0 and 'OK' in r.stdout
        except Exception:  # noqa: BLE001 - any failure means "no link"
            return False

    def start(self):
        return self._ssh('cd /home/pi && ./m1.sh bg', timeout=120)

    def stop(self):
        return self._ssh('cd /home/pi && ./m1.sh stop --hard', timeout=120)

    def diag(self):
        return self._ssh('cd /home/pi && ./m1.sh diag', timeout=180)


# ---------------------------------------------------------------------------
# Small custom widgets
# ---------------------------------------------------------------------------

def make_meter(QtCore, QtGui, QtWidgets, colour='#3b82f6'):
    """A thin horizontal bar for 0..1 values."""

    class Meter(QtWidgets.QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self._v = 0.0
            self.setMinimumHeight(8)
            self.setMaximumHeight(8)
            self._colour = colour

        def set_value(self, v):
            v = max(0.0, min(1.0, float(v)))
            if abs(v - self._v) > 0.005:
                self._v = v
                self.update()

        def paintEvent(self, _e):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            r = self.rect().adjusted(0, 1, 0, -1)
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#1a1e24'))
            p.drawRoundedRect(r, 3, 3)
            if self._v > 0.01:
                w = int(r.width() * self._v)
                p.setBrush(QtGui.QColor(self._colour))
                p.drawRoundedRect(QtCore.QRect(r.left(), r.top(), w, r.height()),
                                  3, 3)

    return Meter


def make_servo_bar(QtCore, QtGui, QtWidgets):
    """Horizontal bar showing position inside the allowed servo window."""

    class ServoBar(QtWidgets.QWidget):
        def __init__(self, lo, hi, centre, parent=None):
            super().__init__(parent)
            self.lo, self.hi, self.centre = float(lo), float(hi), float(centre)
            self.value = float(centre)
            self.setMinimumHeight(22)

        def set_value(self, v):
            if abs(float(v) - self.value) > 1e-9:
                self.value = float(v)
                self.update()

        def paintEvent(self, _e):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            r = self.rect().adjusted(0, 7, 0, -7)
            span = max(1e-6, self.hi - self.lo)

            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#1a1e24'))
            p.drawRoundedRect(r, 4, 4)

            # Centre tick, so "am I looking straight ahead" is obvious.
            cx = r.left() + r.width() * (self.centre - self.lo) / span
            p.setPen(QtGui.QPen(QtGui.QColor('#4b5563'), 1, QtCore.Qt.DotLine))
            p.drawLine(int(cx), r.top() - 3, int(cx), r.bottom() + 3)

            # Filled portion from centre to the current position.
            vx = r.left() + r.width() * (min(self.hi, max(self.lo, self.value))
                                         - self.lo) / span
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#2563eb'))
            x0, x1 = sorted((cx, vx))
            if x1 - x0 > 1:
                p.drawRoundedRect(
                    QtCore.QRectF(x0, r.top(), x1 - x0, r.height()), 3, 3)

            # Knob.
            p.setBrush(QtGui.QColor('#93c5fd'))
            p.drawEllipse(QtCore.QPointF(vx, r.center().y()), 5, 5)

        def sizeHint(self):
            return QtCore.QSize(160, 22)

    return ServoBar


def make_gaze_map(QtCore, QtGui, QtWidgets):
    """Square plot of the gaze point: raw (hollow) and corrected (solid)."""

    class GazeMap(QtWidgets.QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.raw = None
            self.corrected = None
            self.valid = False
            self.stale = True
            self.setMinimumSize(170, 170)
            # Let it grow: on a maximised window there is space, and a 170px
            # gaze plot in a 400px panel looks unfinished.
            self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                               QtWidgets.QSizePolicy.Expanding)

        def set_points(self, raw, corrected, valid, stale):
            self.raw, self.corrected = raw, corrected
            self.valid, self.stale = valid, stale
            self.update()

        def paintEvent(self, _e):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            side = min(self.width(), self.height())
            x0 = (self.width() - side) / 2
            y0 = (self.height() - side) / 2
            box = QtCore.QRectF(x0, y0, side, side)

            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#0d1014'))
            p.drawRoundedRect(box, 6, 6)

            # Thirds grid + deadzone circle hint.
            p.setPen(QtGui.QPen(QtGui.QColor('#232a33'), 1))
            for i in (1, 2):
                p.drawLine(int(box.left() + side * i / 3), int(box.top()),
                           int(box.left() + side * i / 3), int(box.bottom()))
                p.drawLine(int(box.left()), int(box.top() + side * i / 3),
                           int(box.right()), int(box.top() + side * i / 3))
            p.setPen(QtGui.QPen(QtGui.QColor('#1f2937'), 1, QtCore.Qt.DashLine))
            p.drawEllipse(box.center(), side * 0.18, side * 0.18)

            def to_px(u, v):
                return QtCore.QPointF(box.left() + side * u,
                                      box.top() + side * v)

            # Centre crosshair.
            p.setPen(QtGui.QPen(QtGui.QColor('#374151'), 1))
            p.drawLine(QtCore.QPointF(box.center().x() - 7, box.center().y()),
                       QtCore.QPointF(box.center().x() + 7, box.center().y()))
            p.drawLine(QtCore.QPointF(box.center().x(), box.center().y() - 7),
                       QtCore.QPointF(box.center().x(), box.center().y() + 7))

            if self.raw is not None and not self.stale:
                p.setPen(QtGui.QPen(QtGui.QColor('#f59e0b'), 2))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawEllipse(to_px(*self.raw), 6, 6)

            if self.corrected is not None and not self.stale:
                col = '#34d399' if self.valid else '#6b7280'
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(col))
                p.drawEllipse(to_px(*self.corrected), 4, 4)

            if self.stale:
                p.setPen(QtGui.QColor('#6b7280'))
                p.drawText(box, QtCore.Qt.AlignCenter, 'no gaze source')

    return GazeMap


def make_drive_pad(QtCore, QtGui, QtWidgets):
    """Visual WASD/QE keypad that lights up while keys are held."""

    class DrivePad(QtWidgets.QWidget):
        ROWS = [['Q', 'W', 'E'], ['A', 'S', 'D']]

        def __init__(self, parent=None):
            super().__init__(parent)
            self.held = set()
            self.setMinimumHeight(74)
            self.setMinimumWidth(150)

        def set_held(self, held):
            self.held = {k.upper() for k in held}
            self.update()

        def paintEvent(self, _e):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            key = min((self.width() - 8) // 3, (self.height() - 8) // 2)
            key = max(18, min(key, 34))
            gap = 4
            total_w = key * 3 + gap * 2
            x0 = (self.width() - total_w) / 2
            y0 = (self.height() - (key * 2 + gap)) / 2

            for r, row in enumerate(self.ROWS):
                for c, label in enumerate(row):
                    rect = QtCore.QRectF(x0 + c * (key + gap),
                                         y0 + r * (key + gap), key, key)
                    on = label in self.held
                    p.setPen(QtGui.QPen(
                        QtGui.QColor('#3b82f6' if on else '#333b45'), 1))
                    p.setBrush(QtGui.QColor('#1e3a5f' if on else '#1a1e24'))
                    p.drawRoundedRect(rect, 4, 4)
                    p.setPen(QtGui.QColor('#dbeafe' if on else '#8b939c'))
                    f = p.font()
                    f.setBold(on)
                    f.setPointSizeF(9)
                    p.setFont(f)
                    p.drawText(rect, QtCore.Qt.AlignCenter, label)

    return DrivePad


def make_map_view(QtCore, QtGui, QtWidgets):
    """A compact SLAM map: occupancy grid, live laser returns, robot pose.

    Deliberately not an embedded RViz. librviz has no usable Python widget
    bindings in Humble, and pulling RViz into the console would mean either a
    second ROS process or a C++ shim. Drawing the grid and the scan directly
    costs about 150 lines and gives exactly what an operator needs from a small
    map: where am I, where is the map, and is the scan still arriving.

    The view auto-fits to whatever has been mapped so far, so it stays useful
    from the first few scans to a full room without any manual zooming.
    """

    class MapView(QtWidgets.QWidget):

        def __init__(self, parent=None):
            super().__init__(parent)
            self.map = None
            self.scan = None
            self.pose = None
            self.scan_age = 99.0
            self.map_age = 99.0
            self._img = None          # cached occupancy QImage
            self._img_src = None      # identity of the map tuple it came from
            self.setMinimumHeight(168)
            self.setToolTip('SLAM map — occupancy grid, live laser, robot pose')

        def update_view(self, map_tuple, scan_tuple, pose, scan_age, map_age):
            self.map = map_tuple
            self.scan = scan_tuple
            self.pose = pose
            self.scan_age = scan_age
            self.map_age = map_age
            self.update()

        def _build_image(self):
            """Render the occupancy grid into a QImage, once per map update.

            Painting tens of thousands of individual cells on every repaint is
            fine for a 200x200 map and hopeless for a 2000x2000 one. Building
            the image once and letting Qt scale it keeps the widget cheap no
            matter how large the map grows.
            """
            w, h, res, ox, oy, cells = self.map
            img = QtGui.QImage(w, h, QtGui.QImage.Format_ARGB32)
            img.fill(QtCore.Qt.transparent)
            free = QtGui.QColor('#1f3a2e').rgb()
            occ = QtGui.QColor('#e5e7eb').rgb()
            for j in range(h):
                row = j * w
                # QImage row 0 is the top, ROS y increases upwards, so the
                # grid is written upside down to draw the right way up.
                # setPixel takes (x, y) - the row index is the y coordinate,
                # NOT the flattened offset j*w.
                dst_y = h - 1 - j
                for i in range(w):
                    v = cells[row + i]
                    if v == 2:
                        img.setPixel(i, dst_y, occ)
                    elif v == 1:
                        img.setPixel(i, dst_y, free)
            self._img = img
            self._img_src = self.map

        # -------------------------------------------------------------- paint
        def paintEvent(self, _e):
            p = QtGui.QPainter(self)
            r = self.rect().adjusted(0, 0, -1, -1)

            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#0d1014'))
            p.drawRoundedRect(QtCore.QRectF(r), 5, 5)
            p.setPen(QtGui.QPen(QtGui.QColor('#2b3138'), 1))
            p.drawRoundedRect(QtCore.QRectF(r), 5, 5)

            if self.map is None:
                p.setPen(QtGui.QColor('#6b7280'))
                f = p.font()
                f.setPointSizeF(8.5)
                p.setFont(f)
                p.drawText(r, QtCore.Qt.AlignCenter,
                           'no map yet\nlaunch SLAM + RViz')
                return

            w, h, res, ox, oy, cells = self.map

            # World-space bounds of the mapped area, plus the robot, so the
            # view never puts the robot off-screen.
            x0, y0 = ox, oy
            x1, y1 = ox + w * res, oy + h * res
            if self.pose is not None:
                px, py = self.pose[0], self.pose[1]
                x0, x1 = min(x0, px - 0.5), max(x1, px + 0.5)
                y0, y1 = min(y0, py - 0.5), max(y1, py + 0.5)

            span_x = max(0.5, x1 - x0)
            span_y = max(0.5, y1 - y0)
            inner = r.adjusted(4, 4, -4, -4)
            scale = min(inner.width() / span_x, inner.height() / span_y)

            cx_world = (x0 + x1) / 2.0
            cy_world = (y0 + y1) / 2.0
            cx_px = inner.center().x()
            cy_px = inner.center().y()

            def to_px(wx, wy):
                # ROS +y is up, screen +y is down, hence the sign flip.
                return QtCore.QPointF(cx_px + (wx - cx_world) * scale,
                                      cy_px - (wy - cy_world) * scale)

            # ---- occupancy grid -------------------------------------------
            if self._img_src is not self.map:
                self._build_image()
            if self._img is not None:
                top_left = to_px(ox, oy + h * res)
                p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
                p.drawImage(QtCore.QRectF(
                    top_left.x(), top_left.y(), w * res * scale, h * res * scale),
                    self._img)
                p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)

            # ---- laser returns --------------------------------------------
            # Drawn in the robot's frame, rotated into the map by the pose.
            if self.scan is not None and self.pose is not None \
                    and self.scan_age < 1.5:
                a0, da, rmax, ranges = self.scan
                px, py, yaw = self.pose
                p.setPen(QtGui.QPen(QtGui.QColor('#f59e0b'), 1))
                step = max(1, len(ranges) // 400)
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                for k in range(0, len(ranges), step):
                    rr = ranges[k]
                    if not (0.05 < rr < rmax):
                        continue
                    a = a0 + k * da
                    # Point in the robot frame...
                    bx, by = rr * math.cos(a), rr * math.sin(a)
                    # ...rotated and translated into the map frame.
                    wx = px + bx * cos_y - by * sin_y
                    wy = py + bx * sin_y + by * cos_y
                    p.drawPoint(to_px(wx, wy))

            # ---- robot pose -----------------------------------------------
            if self.pose is not None:
                px, py, yaw = self.pose
                c = to_px(px, py)
                size = 9.0
                tri = QtGui.QPolygonF([
                    QtCore.QPointF(size, 0.0),
                    QtCore.QPointF(-size * 0.7, size * 0.62),
                    QtCore.QPointF(-size * 0.7, -size * 0.62),
                ])
                p.save()
                p.translate(c)
                # Screen y is flipped, so a positive yaw becomes -yaw here.
                p.rotate(-math.degrees(yaw))
                p.setPen(QtGui.QPen(QtGui.QColor('#93c5fd'), 1))
                p.setBrush(QtGui.QColor('#2563eb'))
                p.drawPolygon(tri)
                p.restore()

            # ---- footer ---------------------------------------------------
            p.setPen(QtGui.QColor('#6b7280'))
            f = p.font()
            f.setPointSizeF(7.5)
            p.setFont(f)
            if self.scan_age > 1.5:
                note = 'scan stale'
            elif self.pose is None:
                note = 'no map→base_link'
            else:
                note = f'{w}×{h} @ {res * 100:.0f} cm'
            p.drawText(r.adjusted(6, 0, -6, -3),
                       QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom, note)

    return MapView


# ---------------------------------------------------------------------------
# Five-point gaze calibration
# ---------------------------------------------------------------------------

class CalibDialog:
    """Guided 5-point calibration.

    Each eye tracker has a resting bias: the operator looks straight ahead and
    the model reports u=0.62 instead of 0.5. Uncorrected, "neutral" steering
    pulls the robot to one side forever and the operator fights the offset all
    session. Five points also recover the gain, so a small comfortable eye
    movement still covers the full steering range.

    The maths, with corrected = (raw - offset) * gain + 0.5:
        centre_raw  -> 0.5   =>  offset = centre_raw - 0.5
        right, left -> 0.85, 0.15
                            =>  gain = 0.7 / (right_raw - left_raw)
    """

    TARGETS = [
        ('centre', 0.50, 0.50, 'Look at the centre of the screen'),
        ('left', 0.15, 0.50, 'Look at the LEFT dot, keep your head still'),
        ('right', 0.85, 0.50, 'Look at the RIGHT dot, keep your head still'),
        ('up', 0.50, 0.15, 'Look at the TOP dot, keep your head still'),
        ('down', 0.50, 0.85, 'Look at the BOTTOM dot, keep your head still'),
    ]

    SETTLE_S = 1.4
    SAMPLE_S = 1.6

    @classmethod
    def build(cls, QtCore, QtGui, QtWidgets):
        class _Dialog(QtWidgets.QDialog):

            def __init__(self, node, parent=None):
                super().__init__(parent)
                self.node = node
                self.result_calib = None
                self.idx = -1
                self.phase = 'idle'
                self.samples = []
                self.captured = {}
                self.timer = QtCore.QTimer(self)
                self.timer.timeout.connect(self._tick)
                self.phase_end = 0.0

                self.setWindowTitle('Gaze calibration')
                self.setModal(True)
                self.resize(460, 400)

                v = QtWidgets.QVBoxLayout(self)
                self.lbl = QtWidgets.QLabel('Ready')
                self.lbl.setObjectName('h1')
                self.lbl.setAlignment(QtCore.Qt.AlignCenter)
                v.addWidget(self.lbl)

                self.hint = QtWidgets.QLabel(
                    'You will be shown five dots. Look at each one and hold '
                    'your gaze steady.\nSit as you would normally drive.')
                self.hint.setObjectName('dim')
                self.hint.setAlignment(QtCore.Qt.AlignCenter)
                self.hint.setWordWrap(True)
                v.addWidget(self.hint)

                self.canvas = _Target(self)
                self.canvas.setMinimumHeight(170)
                v.addWidget(self.canvas, 1)

                self.status = QtWidgets.QLabel('')
                self.status.setObjectName('mono')
                self.status.setAlignment(QtCore.Qt.AlignCenter)
                v.addWidget(self.status)

                row = QtWidgets.QHBoxLayout()
                self.btn_go = QtWidgets.QPushButton('Start')
                self.btn_go.setObjectName('primary')
                self.btn_go.clicked.connect(self.start)
                self.btn_cancel = QtWidgets.QPushButton('Cancel')
                self.btn_cancel.clicked.connect(self.reject)
                row.addWidget(self.btn_go)
                row.addWidget(self.btn_cancel)
                v.addLayout(row)

            # -------------------------------------------------------- control
            def start(self):
                raw = self.node.latest_raw()
                if raw is None or raw[3] > 2.0:
                    self.lbl.setText('No gaze source')
                    self.hint.setText(
                        'The gaze bridge is not publishing /gaze/raw.\n'
                        'Start it on the robot first, then retry.')
                    return
                self.btn_go.setEnabled(False)
                self.idx = 0
                self._begin_point()

            def _begin_point(self):
                name, u, v, text = self.TARGETS[self.idx]
                self.canvas.set_target(u, v)
                self.lbl.setText(f'Step {self.idx + 1} of {len(self.TARGETS)}')
                self.hint.setText(text)
                self.samples = []
                self.phase = 'settle'
                self.phase_end = time.monotonic() + self.SETTLE_S
                self.timer.start(50)

            def _tick(self):
                now = time.monotonic()
                left = max(0.0, self.phase_end - now)
                raw = self.node.latest_raw()

                if self.phase == 'settle':
                    self.status.setText(f'hold still... {left:.1f}s')
                    if left <= 0:
                        self.phase = 'sample'
                        self.phase_end = now + self.SAMPLE_S
                    return

                if self.phase == 'sample':
                    if raw is not None and raw[2] and raw[3] < 0.5:
                        self.samples.append((raw[0], raw[1]))
                    self.status.setText(
                        f'capturing {len(self.samples)} samples  '
                        f'{left:.1f}s')
                    if left <= 0:
                        self._finish_point()
                    return

                if self.phase == 'done':
                    self.timer.stop()

            def _finish_point(self):
                name = self.TARGETS[self.idx][0]
                if len(self.samples) < 5:
                    self.lbl.setText('Not enough samples')
                    self.hint.setText(
                        'The tracker lost your eyes. Check the camera, then '
                        'press Retry.')
                    self.btn_go.setText('Retry')
                    self.btn_go.setEnabled(True)
                    self.phase = 'idle'
                    self.timer.stop()
                    return
                n = len(self.samples)
                mu = sum(s[0] for s in self.samples) / n
                mv = sum(s[1] for s in self.samples) / n
                self.captured[name] = (mu, mv)
                self.status.setText(f'{name}: u={mu:.3f} v={mv:.3f}')

                self.idx += 1
                if self.idx < len(self.TARGETS):
                    self._begin_point()
                else:
                    self._compute()

            def _compute(self):
                cu, cv = self.captured['centre']
                lu, _ = self.captured['left']
                ru, _ = self.captured['right']
                _, uu = self.captured['up']
                _, dv = self.captured['down']

                # The robot applies  corrected = (raw - offset) * gain + 0.5.
                # For the resting gaze to land on 0.5 the offset must be the
                # raw value itself, NOT (raw - 0.5). Subtracting 0.5 there
                # yields corrected = 1.0 and pins the robot to one side - it
                # looks like calibration "did nothing", which is worse than
                # not calibrating at all.
                ox = cu
                oy = cv

                # 0.7 of the output range should come from the eye travel
                # between the left and right targets:
                #   (ru - ox)*gx + 0.5 = 0.85
                #   (lu - ox)*gx + 0.5 = 0.15
                # Subtracting gives gx = 0.7 / (ru - lu).
                dx = ru - lu
                dy = dv - uu
                gx = 0.7 / dx if abs(dx) > 1e-3 else 1.0
                gy = 0.7 / dy if abs(dy) > 1e-3 else 1.0

                # A tracker that barely moved between targets produces a huge
                # gain, which would make the robot twitch on noise.
                gx = min(5.0, max(0.25, gx))
                gy = min(5.0, max(0.25, gy))

                self.result_calib = (ox, oy, gx, gy)
                self.phase = 'done'
                self.timer.stop()
                self.canvas.set_target(None, None)
                self.lbl.setText('Calibration complete')
                self.hint.setText(
                    f'offset=({ox:+.3f}, {oy:+.3f})  '
                    f'gain=({gx:.2f}, {gy:.2f})')
                self.status.setText('Press Apply to send this to the robot.')
                self.btn_go.setText('Apply')
                self.btn_go.setEnabled(True)
                try:
                    self.btn_go.clicked.disconnect()
                except TypeError:
                    pass
                self.btn_go.clicked.connect(self.accept)

            def reject(self):
                self.timer.stop()
                super().reject()

        class _Target(QtWidgets.QWidget):
            def __init__(self, parent=None):
                super().__init__(parent)
                self.u = None
                self.v = None

            def set_target(self, u, v):
                self.u, self.v = u, v
                self.update()

            def paintEvent(self, _e):
                p = QtGui.QPainter(self)
                p.setRenderHint(QtGui.QPainter.Antialiasing)
                r = self.rect().adjusted(2, 2, -2, -2)
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor('#0d1014'))
                p.drawRoundedRect(QtCore.QRectF(r), 6, 6)
                p.setPen(QtGui.QPen(QtGui.QColor('#232a33'), 1))
                p.drawRect(r)
                if self.u is None:
                    return
                c = QtCore.QPointF(r.left() + r.width() * self.u,
                                   r.top() + r.height() * self.v)
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor('#2563eb'))
                p.drawEllipse(c, 13, 13)
                p.setBrush(QtGui.QColor('#dbeafe'))
                p.drawEllipse(c, 5, 5)

        return _Dialog


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ConsoleNode()

    stop_event = threading.Event()

    def spin():
        # ExternalShutdownException is raised normally when rclpy shuts down
        # while spin_once is blocked. Without catching it, closing the window
        # prints a traceback that looks like a crash.
        from rclpy.executors import ExternalShutdownException
        while not stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except ExternalShutdownException:
                break
            except Exception as exc:  # noqa: BLE001
                print(f'[console] spin error: {exc}', file=sys.stderr)
                break

    ros_thread = threading.Thread(target=spin, daemon=True)
    ros_thread.start()

    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    except ImportError:
        print('PyQt5 is required.\n    sudo apt install python3-pyqt5',
              file=sys.stderr)
        stop_event.set()
        return 1

    Meter = make_meter(QtCore, QtGui, QtWidgets, '#3b82f6')
    MeterGreen = make_meter(QtCore, QtGui, QtWidgets, '#34d399')
    ServoBar = make_servo_bar(QtCore, QtGui, QtWidgets)
    GazeMap = make_gaze_map(QtCore, QtGui, QtWidgets)
    MapView = make_map_view(QtCore, QtGui, QtWidgets)
    DrivePad = make_drive_pad(QtCore, QtGui, QtWidgets)
    # Bind to a NEW name: assigning back to CalibDialog would make it a
    # local of main() and the right-hand reference would raise
    # UnboundLocalError.
    CalibDialogCls = CalibDialog.build(QtCore, QtGui, QtWidgets)
    shell = RobotShell(node.robot_ip)

    class Console(QtWidgets.QMainWindow):

        # Cross-thread channels. Worker threads (SSH commands, link probes)
        # must never touch a widget; they emit these and Qt delivers them on
        # the GUI thread. Touching a widget from a worker segfaults the whole
        # process, not just the thread.
        log_signal = QtCore.pyqtSignal(str)
        link_signal = QtCore.pyqtSignal(bool)
        recheck_signal = QtCore.pyqtSignal()
        view_signal = QtCore.pyqtSignal(bool)

        def __init__(self):
            super().__init__()
            self.setWindowTitle('ROSMASTER M1 — Operator Console')
            self.resize(1340, 900)
            self.setMinimumSize(1040, 680)
            self.setStyleSheet(STYLE)
            self.held = set()
            self.audio_on = False
            self._last_jpeg = None
            self._map_sig = None
            self._link_busy = False

            # Connections must exist before anything can emit.
            self.log_signal.connect(self._append_log)
            self.link_signal.connect(self._apply_link)
            self.recheck_signal.connect(self._check_link)
            self.view_signal.connect(self._apply_view_state)

            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            outer = QtWidgets.QVBoxLayout(central)
            outer.setContentsMargins(12, 12, 12, 12)
            outer.setSpacing(10)

            self.banner = QtWidgets.QLabel()
            self.banner.setObjectName('banner')
            self.banner.setWordWrap(True)
            self.banner.hide()
            outer.addWidget(self.banner)

            outer.addWidget(self._build_header())

            split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            split.setChildrenCollapsible(False)
            outer.addWidget(split, 1)

            split.addWidget(self._build_left())
            split.addWidget(self._build_centre())
            split.addWidget(self._build_right())
            split.setStretchFactor(0, 0)
            split.setStretchFactor(1, 1)
            split.setStretchFactor(2, 0)
            split.setSizes([290, 640, 320])

            outer.addWidget(self._build_log())

            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(100)

            self.link_timer = QtCore.QTimer(self)
            self.link_timer.timeout.connect(self._check_link)
            self.link_timer.start(5000)
            self._check_link()
            self._check_dds()

            # Keep the View state honest even if the stack was started or
            # stopped outside the GUI.
            self.view_timer = QtCore.QTimer(self)
            self.view_timer.timeout.connect(self._poll_view)
            self.view_timer.start(3000)
            self._poll_view()

        # ------------------------------------------------------------- layout
        def _build_header(self):
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(2, 0, 2, 0)
            h.setSpacing(18)

            self.lbl_title = QtWidgets.QLabel('ROSMASTER M1')
            self.lbl_title.setObjectName('h1')
            h.addWidget(self.lbl_title)

            self.lbl_link = QtWidgets.QLabel('checking link…')
            self.lbl_link.setObjectName('dim')
            h.addWidget(self.lbl_link)

            h.addStretch(1)

            self.lbl_mode_hdr = QtWidgets.QLabel('—')
            self.lbl_mode_hdr.setObjectName('big')
            h.addWidget(self.lbl_mode_hdr)

            h.addStretch(1)

            self.lbl_bat = QtWidgets.QLabel('—')
            self.lbl_video = QtWidgets.QLabel('—')
            self.lbl_dds = QtWidgets.QLabel('—')
            for lbl, cap in ((self.lbl_bat, 'Battery'), (self.lbl_video, 'Video'),
                             (self.lbl_dds, 'DDS')):
                box = QtWidgets.QVBoxLayout()
                box.setSpacing(0)
                c = QtWidgets.QLabel(cap)
                c.setObjectName('dim')
                box.addWidget(c)
                box.addWidget(lbl)
                holder = QtWidgets.QWidget()
                holder.setLayout(box)
                h.addWidget(holder)
            return w

        def _build_left(self):
            panel = QtWidgets.QWidget()
            v = QtWidgets.QVBoxLayout(panel)
            v.setContentsMargins(0, 0, 6, 0)
            # Tighter than the other columns: four panels share this one.
            v.setSpacing(7)

            # -- robot ----------------------------------------------------
            g = QtWidgets.QGroupBox('1 · Robot')
            gv = QtWidgets.QVBoxLayout(g)
            row = QtWidgets.QHBoxLayout()
            self.btn_start = QtWidgets.QPushButton('Start')
            self.btn_stop = QtWidgets.QPushButton('Stop')
            self.btn_diag = QtWidgets.QPushButton('Diagnose')
            for b in (self.btn_start, self.btn_stop, self.btn_diag):
                b.setEnabled(False)
                row.addWidget(b)
            gv.addLayout(row)
            self.chk_autoview = QtWidgets.QCheckBox('Also launch SLAM + RViz')
            self.chk_autoview.setChecked(True)
            gv.addWidget(self.chk_autoview)
            self.btn_start.clicked.connect(lambda: self._remote('start'))
            self.btn_stop.clicked.connect(lambda: self._remote('stop'))
            self.btn_diag.clicked.connect(lambda: self._remote('diag'))
            v.addWidget(g)

            # -- mode -----------------------------------------------------
            g = QtWidgets.QGroupBox('2 · Control mode')
            gm = QtWidgets.QGridLayout(g)
            gm.setSpacing(6)
            self.mode_buttons = {}
            for i, (mode, label) in enumerate([
                    (1, '1  Keyboard'), (2, '2  Gaze'),
                    (3, '3  EEG'), (4, '4  Gaze + EEG')]):
                b = QtWidgets.QPushButton(label)
                b.setObjectName('mode')
                b.setCheckable(True)
                b.clicked.connect(lambda _c, m=mode: node.set_mode(m))
                gm.addWidget(b, i // 2, i % 2)
                self.mode_buttons[mode] = b
            self.btn_estop = QtWidgets.QPushButton('■  EMERGENCY STOP')
            self.btn_estop.setObjectName('estop')
            self.btn_estop.clicked.connect(self._estop)
            gm.addWidget(self.btn_estop, 2, 0, 1, 2)
            v.addWidget(g)

            # -- servo ----------------------------------------------------
            g = QtWidgets.QGroupBox('3 · Camera head')
            gs = QtWidgets.QVBoxLayout(g)
            gs.setSpacing(4)
            # Both sliders plus the centre button need this much; without a
            # floor the TILT bar is the first thing to be clipped when the
            # column gets crowded.
            g.setMinimumHeight(192)

            self.bar_pan = ServoBar(node.s2_min, node.s2_max, node.s2_center)
            self.bar_tilt = ServoBar(node.s1_min, node.s1_max, node.s1_center)

            # No per-slider direction hints. They cost a text row each, and the
            # keys are already listed in the Drive panel - with the Voice panel
            # sharing this column there is no room to say everything twice.
            for label, bar in (('PAN  (J / L)', self.bar_pan),
                               ('TILT (I / K)', self.bar_tilt)):
                cap = QtWidgets.QHBoxLayout()
                t = QtWidgets.QLabel(label)
                t.setObjectName('dim')
                cap.addWidget(t)
                cap.addStretch(1)
                val = QtWidgets.QLabel('—')
                val.setObjectName('mono')
                cap.addWidget(val)
                gs.addLayout(cap)
                gs.addWidget(bar)
                if bar is self.bar_pan:
                    self.lbl_pan = val
                else:
                    self.lbl_tilt = val

            # Range readout and the centre button share one row. They were two
            # rows until the Voice panel moved into this column and there was no
            # longer room for both - the Centre button was being squeezed out
            # entirely, which is a bad thing to lose from a camera control.
            row = QtWidgets.QHBoxLayout()
            rng = QtWidgets.QLabel(
                f'pan {node.s2_min:.0f}–{node.s2_max:.0f}  '
                f'tilt {node.s1_min:.0f}–{node.s1_max:.0f}')
            rng.setObjectName('dim')
            row.addWidget(rng)
            row.addStretch(1)
            self.btn_centre = QtWidgets.QPushButton('Centre (U)')
            self.btn_centre.clicked.connect(node.centre_servo)
            row.addWidget(self.btn_centre)
            gs.addLayout(row)
            v.addWidget(g)

            # -- voice ----------------------------------------------------
            # Sits under the camera head rather than in the right column: the
            # left column was shorter than the others and left a visible gap
            # above the log, and voice is a "robot hardware" control like the
            # head, so it belongs next to it.
            g = QtWidgets.QGroupBox('4 · Voice')
            gv = QtWidgets.QVBoxLayout(g)
            gv.setSpacing(4)
            # Explicit floor: the panel was landing exactly on its minimum and
            # the status line then sat against the border.
            g.setMinimumHeight(186)
            self.btn_audio = QtWidgets.QPushButton('Enable audio')
            self.btn_audio.setObjectName('primary')
            self.btn_audio.clicked.connect(self._toggle_audio)
            gv.addWidget(self.btn_audio)

            # Two toggle buttons, Discord-style, because a checkbox gives no
            # hint about which state is "good". A button that reads "Mic on" in
            # green and flips to "Muted" in red is unambiguous at a glance,
            # which is what matters when someone else's mobility depends on it.
            #
            #   Mute    my voice -> the robot.  Drives /audio/robot_speaker.
            #   Deafen  the robot -> my ears.   Drives /audio/robot_mic.
            #
            # The topic names are the ROBOT's hardware on purpose: "my mic" is
            # a different device on each machine, and that ambiguity is what
            # made the old checkboxes look swapped.
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            self.btn_mute = QtWidgets.QPushButton()
            self.btn_mute.setObjectName('toggle')
            self.btn_mute.setCheckable(True)
            self.btn_mute.setToolTip(
                'Mute: stop sending my voice to the robot speaker.\n'
                'Unmuted (green) = the robot can hear me.')
            self.btn_mute.toggled.connect(self._on_mute)
            row.addWidget(self.btn_mute, 1)

            self.btn_deaf = QtWidgets.QPushButton()
            self.btn_deaf.setObjectName('toggle')
            self.btn_deaf.setCheckable(True)
            self.btn_deaf.setToolTip(
                'Deafen: stop hearing the robot microphone.\n'
                'Hearing (green) = I can hear the room.')
            self.btn_deaf.toggled.connect(self._on_deafen)
            row.addWidget(self.btn_deaf, 1)
            gv.addLayout(row)

            self._on_mute(False)
            self._on_deafen(False)

            # Both meters on one row. Stacked, they cost two text rows in a
            # column that has four panels in it - and a 6px-tall bar does not
            # need its own line to be readable.
            self.meter_sent = Meter()
            self.meter_recv = MeterGreen()
            mr = QtWidgets.QHBoxLayout()
            mr.setSpacing(4)
            for label, m in (('out', self.meter_sent), ('in', self.meter_recv)):
                t = QtWidgets.QLabel(label)
                t.setObjectName('dim')
                t.setFixedWidth(20)
                mr.addWidget(t)
                mr.addWidget(m, 1)
            gv.addLayout(mr)

            self.lbl_audio = QtWidgets.QLabel('off')
            self.lbl_audio.setObjectName('mono')
            gv.addWidget(self.lbl_audio)
            v.addWidget(g)

            v.addStretch(1)
            return panel

        def _build_centre(self):
            panel = QtWidgets.QWidget()
            v = QtWidgets.QVBoxLayout(panel)
            v.setContentsMargins(0, 0, 0, 0)
            v.setSpacing(10)

            g = QtWidgets.QGroupBox('Camera')
            gv = QtWidgets.QVBoxLayout(g)
            self.lbl_cam = QtWidgets.QLabel('waiting for /image_throttled/compressed…')
            self.lbl_cam.setAlignment(QtCore.Qt.AlignCenter)
            self.lbl_cam.setMinimumHeight(300)
            self.lbl_cam.setStyleSheet(
                'background:#0d1014; border:1px solid #2b3138;'
                'border-radius:5px; color:#6b7280;')
            gv.addWidget(self.lbl_cam)
            v.addWidget(g, 1)

            # -- drive ----------------------------------------------------
            g = QtWidgets.QGroupBox('Drive  (this window has keyboard focus)')
            gd = QtWidgets.QHBoxLayout(g)
            self.pad = DrivePad()
            gd.addWidget(self.pad, 0)

            right = QtWidgets.QVBoxLayout()
            right.setSpacing(3)
            self.lbl_drive = QtWidgets.QLabel('idle')
            self.lbl_drive.setObjectName('dim')
            right.addWidget(self.lbl_drive)
            hints = QtWidgets.QLabel(
                'W/S forward · Q/E strafe · A/D rotate\n'
                'I/K tilt · J/L pan · U centre\n'
                '0–4 mode · Space stop · Ctrl+Q quit')
            hints.setObjectName('dim')
            right.addWidget(hints)
            right.addStretch(1)
            gd.addLayout(right, 1)
            v.addWidget(g)

            # -- view -----------------------------------------------------
            g = QtWidgets.QGroupBox('View')
            gv = QtWidgets.QVBoxLayout(g)
            gv.setSpacing(6)
            self.lbl_view = QtWidgets.QLabel('● view stopped')
            self.lbl_view.setObjectName('dim')
            gv.addWidget(self.lbl_view)
            row = QtWidgets.QHBoxLayout()
            self.btn_slam = QtWidgets.QPushButton('Launch SLAM + RViz')
            self.btn_slam.clicked.connect(self._launch_view)
            row.addWidget(self.btn_slam)
            self.btn_killview = QtWidgets.QPushButton('Stop view')
            self.btn_killview.setEnabled(False)
            self.btn_killview.clicked.connect(self._kill_view)
            row.addWidget(self.btn_killview)
            gv.addLayout(row)
            self.chk_stopview = QtWidgets.QCheckBox(
                'Stop the view when I close this window')
            self.chk_stopview.setChecked(True)
            self.chk_stopview.setToolTip(
                'Otherwise SLAM and RViz are left running, and the next launch '
                'adds a second mapper - two maps then fight over /map.')
            gv.addWidget(self.chk_stopview)
            v.addWidget(g)
            return panel

        def _build_right(self):
            panel = QtWidgets.QWidget()
            v = QtWidgets.QVBoxLayout(panel)
            v.setContentsMargins(6, 0, 0, 0)
            v.setSpacing(10)

            # -- gaze calibration -----------------------------------------
            g = QtWidgets.QGroupBox('5 · Gaze')
            gv = QtWidgets.QVBoxLayout(g)
            gv.setSpacing(6)
            self.gaze_map = GazeMap()
            gv.addWidget(self.gaze_map, 0, QtCore.Qt.AlignHCenter)

            self.lbl_gaze = QtWidgets.QLabel('no source')
            self.lbl_gaze.setObjectName('mono')
            self.lbl_gaze.setAlignment(QtCore.Qt.AlignCenter)
            gv.addWidget(self.lbl_gaze)

            self.lbl_zone = QtWidgets.QLabel('—')
            self.lbl_zone.setObjectName('mono')
            self.lbl_zone.setAlignment(QtCore.Qt.AlignCenter)
            gv.addWidget(self.lbl_zone)

            row = QtWidgets.QHBoxLayout()
            self.btn_calib = QtWidgets.QPushButton('Calibrate…')
            self.btn_calib.setObjectName('primary')
            self.btn_calib.clicked.connect(self._calibrate)
            row.addWidget(self.btn_calib)
            self.btn_calib_reset = QtWidgets.QPushButton('Reset')
            self.btn_calib_reset.clicked.connect(self._reset_calibration)
            row.addWidget(self.btn_calib_reset)
            gv.addLayout(row)

            self.lbl_calib = QtWidgets.QLabel('offset —  gain —')
            self.lbl_calib.setObjectName('mono')
            self.lbl_calib.setAlignment(QtCore.Qt.AlignCenter)
            gv.addWidget(self.lbl_calib)
            v.addWidget(g)

            # -- map ------------------------------------------------------
            g = QtWidgets.QGroupBox('6 · Map')
            gm = QtWidgets.QVBoxLayout(g)
            gm.setContentsMargins(6, 6, 6, 6)
            gm.setSpacing(6)
            self.map_view = MapView()
            gm.addWidget(self.map_view, 1)

            self.lbl_map = QtWidgets.QLabel('no map')
            self.lbl_map.setObjectName('mono')
            self.lbl_map.setAlignment(QtCore.Qt.AlignCenter)
            gm.addWidget(self.lbl_map)

            # Localisation controls, right where the map is. These were
            # command-line only, which meant a working map sat on screen with
            # no obvious way to fix a wrong pose.
            mrow = QtWidgets.QHBoxLayout()
            mrow.setSpacing(4)
            self.btn_newmap = QtWidgets.QPushButton('New map')
            self.btn_newmap.setObjectName('primary')
            self.btn_newmap.setToolTip(
                'Start mapping from scratch. Starts slam_toolbox if it is not '
                'running, or clears its pose graph if it is.')
            self.btn_newmap.clicked.connect(self._new_map)
            mrow.addWidget(self.btn_newmap)
            self.btn_savemap = QtWidgets.QPushButton('Save')
            self.btn_savemap.setToolTip('Save the current map to ~/maps/')
            self.btn_savemap.clicked.connect(self._save_map_dialog)
            mrow.addWidget(self.btn_savemap)
            gm.addLayout(mrow)

            mrow2 = QtWidgets.QHBoxLayout()
            mrow2.setSpacing(4)
            self.btn_findme = QtWidgets.QPushButton('Find me')
            self.btn_findme.setToolTip(
                'Match the laser scan against the map and publish the best '
                'pose. Use this after the robot has been moved.')
            self.btn_findme.clicked.connect(lambda: self._map_cmd('autolocalize'))
            mrow2.addWidget(self.btn_findme)
            self.btn_localize = QtWidgets.QPushButton('Localise…')
            self.btn_localize.setToolTip(
                'Stop mapping and localise against a saved map (AMCL)')
            self.btn_localize.clicked.connect(self._localize_dialog)
            mrow2.addWidget(self.btn_localize)
            gm.addLayout(mrow2)

            mrow3 = QtWidgets.QHBoxLayout()
            mrow3.setSpacing(4)
            self.btn_mapstop = QtWidgets.QPushButton('Stop map')
            self.btn_mapstop.setToolTip('Stop mapping or localisation')
            self.btn_mapstop.clicked.connect(lambda: self._map_cmd('stop'))
            mrow3.addWidget(self.btn_mapstop)
            gm.addLayout(mrow3)

            v.addWidget(g, 1)
            return panel

        def _build_log(self):
            g = QtWidgets.QGroupBox('Log')
            gv = QtWidgets.QVBoxLayout(g)
            gv.setContentsMargins(8, 8, 8, 8)
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumHeight(84)
            gv.addWidget(self.log)
            return g

        # ------------------------------------------------------------ helpers
        def logline(self, text):
            """Append a line to the log. Safe to call from ANY thread.

            Qt widgets may only be touched from the GUI thread. Calling
            appendPlainText() directly from an SSH worker thread makes Qt try
            to queue QTextBlock/QTextCursor across threads, which it cannot do
            without qRegisterMetaType - and the result is a segfault, not an
            exception. Emitting a signal instead lets Qt deliver the string to
            the GUI thread properly.
            """
            self.log_signal.emit(f'{time.strftime("%H:%M:%S")}  {text}')

        def _append_log(self, line):
            self.log.appendPlainText(line)

        def _style(self, widget, name):
            widget.setObjectName(name)
            widget.style().unpolish(widget)
            widget.style().polish(widget)

        def _check_dds(self):
            dom = os.environ.get('ROS_DOMAIN_ID', '')
            rmw = os.environ.get('RMW_IMPLEMENTATION', '')
            ok = (dom == EXPECTED_DOMAIN and rmw == EXPECTED_RMW)
            if ok:
                self.lbl_dds.setText('ok')
                self._style(self.lbl_dds, 'good')
                self.banner.hide()
            else:
                self.lbl_dds.setText('mismatch')
                self._style(self.lbl_dds, 'bad')
                self.banner.setText(
                    f'DDS environment is wrong — the GUI will never see the '
                    f'robot. ROS_DOMAIN_ID={dom or "<unset>"} (want '
                    f'{EXPECTED_DOMAIN}), RMW_IMPLEMENTATION='
                    f'{rmw or "<unset>"} (want {EXPECTED_RMW}). '
                    f'Quit and run ~/m1_console.sh instead.')
                self.banner.show()

        def _check_link(self):
            """Probe the robot off the GUI thread.

            shell.test() is an SSH round trip that can take up to 10 s to
            fail. Run on the GUI thread it froze the window for that long,
            every 5 s - which looks exactly like the program hanging.
            """
            if self._link_busy:
                return
            self._link_busy = True

            def work():
                try:
                    ok = shell.test()
                except Exception:  # noqa: BLE001 - any failure means no link
                    ok = False
                self.link_signal.emit(ok)

            threading.Thread(target=work, daemon=True).start()

        def _apply_link(self, ok):
            self._link_busy = False
            if ok:
                self.lbl_link.setText(f'● {node.robot_ip}')
                self._style(self.lbl_link, 'good')
            else:
                self.lbl_link.setText(f'● unreachable ({node.robot_ip})')
                self._style(self.lbl_link, 'bad')
            for b in (self.btn_start, self.btn_stop, self.btn_diag):
                b.setEnabled(ok)

        def _remote(self, action):
            self.logline(f'robot {action}…')
            for b in (self.btn_start, self.btn_stop, self.btn_diag):
                b.setEnabled(False)

            def work():
                try:
                    r = getattr(shell, action)()
                    out = (r.stdout or '') + (r.stderr or '')
                    tail = '\n'.join(out.strip().splitlines()[-5:])
                    self.logline(f'{action} rc={r.returncode}')
                    if tail:
                        self.logline(tail)
                except Exception as exc:  # noqa: BLE001
                    self.logline(f'{action} failed: {exc}')
                finally:
                    # Ask the GUI thread to re-probe the link; do not touch
                    # widgets from here.
                    self.recheck_signal.emit()

            threading.Thread(target=work, daemon=True).start()
            if action == 'start' and self.chk_autoview.isChecked():
                QtCore.QTimer.singleShot(22000, self._launch_view)

        def _estop(self):
            node.estop()
            self.logline('E-STOP')
            self.lbl_drive.setText('STOPPED')

        def _on_mute(self, muted):
            """My voice -> robot speaker. Muted is the checked state."""
            node.audio(robot_speaker=not muted)
            self._style_toggle(self.btn_mute, on=not muted,
                               on_text='Mic on', off_text='Muted')
            if self.audio_on:
                self.logline('microphone muted' if muted
                             else 'microphone live')

        def _on_deafen(self, deafened):
            """Robot microphone -> my ears. Deafened is the checked state."""
            node.audio(robot_mic=not deafened)
            self._style_toggle(self.btn_deaf, on=not deafened,
                               on_text='Hearing', off_text='Deafened',
                               off_kind='grey')
            if self.audio_on:
                self.logline('deafened - cannot hear the robot' if deafened
                             else 'hearing the robot')

        def _style_toggle(self, btn, on, on_text, off_text, off_kind='bad'):
            """Colour a toggle by what it currently does, not by what it toggles.

            A checked button that turns green would be backwards for Mute -
            "checked" there means silent. The colour always describes the
            resulting state: green = audio flowing, red or grey = blocked.
            """
            btn.setText(on_text if on else off_text)
            btn.setProperty('state', 'on' if on else off_kind)
            btn.style().unpolish(btn)
            btn.style().polish(btn)

        def _toggle_audio(self):
            """Enable audio and start/stop the operator-side relay.

            Toggling /audio/enable alone is not enough. That topic tells the
            ROBOT whether to transmit; the operator still needs a bridge of its
            own to capture the microphone and play what arrives. Without one the
            Voice panel looks live while nothing is actually connected - the
            robot sends into the void and the operator hears nothing.
            """
            self.audio_on = not self.audio_on
            node.audio(enable=self.audio_on,
                       robot_mic=not self.btn_deaf.isChecked(),
                       robot_speaker=not self.btn_mute.isChecked())
            self.btn_audio.setText('Disable audio' if self.audio_on
                                   else 'Enable audio')

            if self.audio_on:
                self._start_operator_audio()
            else:
                self._stop_operator_audio()
            self.logline(f"audio {'on' if self.audio_on else 'off'}")

        def _audio_bridge_pids(self):
            try:
                r = subprocess.run(
                    ['pgrep', '-f', 'audio_bridge --no-ros'],
                    capture_output=True, text=True, timeout=3)
                return [int(p) for p in r.stdout.split()]
            except Exception:  # noqa: BLE001
                return []

        def _start_operator_audio(self):
            if self._audio_bridge_pids():
                return
            cmd = ['ros2', 'run', 'm1_control', 'audio_bridge', '--no-ros',
                   '--role', 'operator',
                   '--peer-ip', node.robot_ip,
                   # 'default' routes through PulseAudio. A VMware guest
                   # forwards the host microphone to PulseAudio, not to the raw
                   # ALSA card - reading plughw:0,0 there yields a valid stream
                   # of pure silence, which looks exactly like a dead mic.
                   '--device-in', 'default',
                   '--device-out', 'default']
            try:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL,
                                 start_new_session=True)
                self.logline('audio relay started (operator side)')
            except Exception as exc:  # noqa: BLE001
                self.logline(f'audio relay failed: {exc}')

        def _stop_operator_audio(self):
            pids = self._audio_bridge_pids()
            if not pids:
                return
            for pid in pids:
                try:
                    os.kill(pid, 15)
                except Exception:  # noqa: BLE001
                    pass
            self.logline('audio relay stopped')

        def _calibrate(self):
            dlg = CalibDialogCls(node, self)
            if dlg.exec_() == QtWidgets.QDialog.Accepted and dlg.result_calib:
                ox, oy, gx, gy = dlg.result_calib
                node.set_calibration(ox, oy, gx, gy)
                self.logline(
                    f'calibration applied  offset=({ox:+.3f},{oy:+.3f}) '
                    f'gain=({gx:.2f},{gy:.2f})')

        def _reset_calibration(self):
            # Neutral is offset 0.5 and gain 1.0: (raw - 0.5) + 0.5 == raw.
            # Sending offset 0.0 would add 0.5 to every sample and slam the
            # output to 1.0 - "reset" would look like a hard right turn.
            node.set_calibration(0.5, 0.5, 1.0, 1.0)
            self.logline('calibration reset to neutral')

        def _launch_view(self):
            """Start SLAM + RViz, but only ever one copy.

            Clicking this twice used to start a second slam_toolbox, and two
            mappers publishing /map with different maps makes the map appear to
            jump around - which reads as "the map keeps resetting" rather than
            "two mappers are running". The guard is a flock in the wrapper.

            The wrapper matters for a second reason. Launching with
            `bash -lc 'ros2 launch ...'` inherits whatever AMENT_PREFIX_PATH
            this process happened to start with, and Ubuntu's .bashrc returns
            early for non-interactive shells so it never sources the workspace
            itself. Build a package after the console starts and the view
            inherits a stale path: RViz then shows the RobotModel in ERROR with
            "Errors loading geometries..." because it cannot resolve
            package://yahboomcar_description. m1_view.sh sources both setups
            explicitly and refuses to start if the meshes are unresolvable.
            """
            self.logline('launching SLAM + RViz…')
            wrapper = os.path.expanduser('~/m1_view.sh')
            if not os.path.exists(wrapper):
                self.logline(f'missing {wrapper} - cannot launch the view')
                self.logline('  create it as documented in docs/START_STOP.md')
                return
            try:
                subprocess.Popen(
                    ['bash', wrapper],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, start_new_session=True)
                self.logline('view launched (single instance enforced)')
                # The launch takes a few seconds to come up, so do not claim it
                # is running yet - the poll timer will confirm.
                QtCore.QTimer.singleShot(6000, self._poll_view)
            except Exception as exc:  # noqa: BLE001
                self.logline(f'launch failed: {exc}')

        def _kill_view(self):
            """Stop SLAM + RViz and report what was actually stopped.

            Runs on a worker: kill_vm.sh waits ~4 s for a clean shutdown, and
            doing that on the GUI thread would freeze the window for the whole
            time.
            """
            self.logline('stopping view…')
            script = os.path.expanduser('~/m1_control_ws/scripts/kill_vm.sh')
            if not os.path.exists(script):
                self.logline(f'missing {script}')
                return

            def work():
                try:
                    r = subprocess.run(['bash', script], capture_output=True,
                                       text=True, timeout=60)
                    out = (r.stdout or '') + (r.stderr or '')
                    # Report only the summary lines; the full listing is noisy.
                    for line in out.splitlines():
                        s = line.strip()
                        if s.startswith('[') or 'Publisher count' in s \
                                or s.startswith('(nothing'):
                            self.logline(s)
                    self.logline(f'view stopped (rc={r.returncode})')
                except Exception as exc:  # noqa: BLE001
                    self.logline(f'stop failed: {exc}')
                finally:
                    self.view_signal.emit(False)

            threading.Thread(target=work, daemon=True).start()

        def _view_running(self):
            """True if an operator-side launch is already alive."""
            try:
                r = subprocess.run(['pgrep', '-f', 'm1_operator.launch.py'],
                                   capture_output=True, text=True, timeout=3)
                return r.returncode == 0 and bool(r.stdout.strip())
            except Exception:  # noqa: BLE001
                return False

        # ------------------------------------------------------------ mapping
        def _map_cmd(self, *args):
            """Run scripts/m1_map.sh on a worker thread, logging its output.

            The script talks to the ROS graph, so it needs the same environment
            as everything else here - m1_map.sh sources it itself, which is why
            this does not use `bash -lc`.
            """
            script = os.path.expanduser('~/m1_control_ws/scripts/m1_map.sh')
            if not os.path.exists(script):
                self.logline(f'missing {script}')
                return
            label = ' '.join(args) if args else 'status'
            self.logline(f'map: {label}…')

            def work():
                try:
                    r = subprocess.run(['bash', script, *args],
                                       capture_output=True, text=True,
                                       timeout=180)
                    out = (r.stdout or '') + (r.stderr or '')
                    for line in out.strip().splitlines():
                        s = line.strip()
                        if not s or s.startswith('[INFO]'):
                            continue
                        self.logline(s[:110])
                    if r.returncode != 0:
                        self.logline(f'map {label} failed (rc={r.returncode})')
                except Exception as exc:  # noqa: BLE001
                    self.logline(f'map {label} error: {exc}')

            threading.Thread(target=work, daemon=True).start()

        def _new_map(self):
            """Start mapping, or clear the pose graph if already mapping.

            Both cases are one button because they are the same intention from
            the operator's point of view - "I want a fresh map". Which one is
            correct depends on what is already running, and asking the user to
            know that difference is a design failure.
            """
            if self._map_mode() == 'mapping':
                self.logline('map: restarting the mapper with an empty graph')
                self._map_cmd('reset')
            else:
                self.logline('map: starting a new map')
                self._map_cmd('start')

        def _map_mode(self):
            """'mapping', 'localising', or 'off' - decided locally.

            pgrep is a few milliseconds, unlike the SSH probe, so this is safe
            on the GUI thread.
            """
            try:
                if subprocess.run(['pgrep', '-f', 'async_slam_toolbox'],
                                  capture_output=True).returncode == 0:
                    return 'mapping'
                if subprocess.run(['pgrep', '-f', 'nav2_amcl/lib/nav2_amcl/amcl'],
                                  capture_output=True).returncode == 0:
                    return 'localising'
            except Exception:  # noqa: BLE001
                pass
            return 'off'

        def _save_map_dialog(self):
            name, ok = QtWidgets.QInputDialog.getText(
                self, 'Save map', 'Map name:',
                QtWidgets.QLineEdit.Normal,
                time.strftime('map_%Y%m%d_%H%M'))
            if ok and name.strip():
                self._map_cmd('save', name.strip())

        def _localize_dialog(self):
            """Pick a saved map and start localising against it."""
            maps_dir = os.path.expanduser('~/maps')
            names = []
            if os.path.isdir(maps_dir):
                names = sorted(f[:-5] for f in os.listdir(maps_dir)
                               if f.endswith('.yaml'))
            if not names:
                self.logline('no saved maps in ~/maps - save one first')
                return
            name, ok = QtWidgets.QInputDialog.getItem(
                self, 'Localise', 'Saved map:', names, 0, False)
            if ok and name:
                self._map_cmd('localize', name)

        def _poll_view(self):
            # pgrep is local and returns in a few milliseconds, so unlike the
            # SSH link probe this is safe on the GUI thread.
            self.view_signal.emit(self._view_running())

        def _apply_view_state(self, running):
            self.lbl_view.setText('● SLAM + RViz running' if running
                                  else '● view stopped')
            self._style(self.lbl_view, 'good' if running else 'dim')
            self.btn_slam.setEnabled(not running)
            self.btn_killview.setEnabled(running)

        # --------------------------------------------------------------- keys
        def keyPressEvent(self, event):
            if event.isAutoRepeat():
                return
            key = event.key()
            text = event.text().lower()

            if text in MODE_KEYS:
                node.set_mode(MODE_KEYS[text])
                return
            if key == QtCore.Qt.Key_Space:
                self._estop()
                return
            if (key == QtCore.Qt.Key_Q
                    and event.modifiers() & QtCore.Qt.ControlModifier):
                # Route through close() so closeEvent runs and the view is
                # cleaned up. Calling app.quit() directly would skip it and
                # leave SLAM running.
                self.close()
                return
            if key == QtCore.Qt.Key_F11:
                # Fullscreen hides the title bar and window decorations.
                # Escape or F11 returns to the maximised window.
                if self.isFullScreen():
                    self.showMaximized()
                else:
                    self.showFullScreen()
                return
            if key == QtCore.Qt.Key_Escape and self.isFullScreen():
                self.showMaximized()
                return

            mapping = {
                'w': ('linear', 'x', 0.20), 's': ('linear', 'x', -0.20),
                'q': ('linear', 'y', 0.20), 'e': ('linear', 'y', -0.20),
                'a': ('angular', 'z', 0.60), 'd': ('angular', 'z', -0.60),
            }
            if text in mapping:
                group, axis, value = mapping[text]
                setattr(getattr(node.teleop, group), axis, value)
                self.held.add(text)
                self.pad.set_held(self.held)
                self.lbl_drive.setText(f'driving: {text.upper()}')
                return

            step = node.servo_step
            servo = {'i': (step, 0.0), 'k': (-step, 0.0),
                     'j': (0.0, step), 'l': (0.0, -step)}
            if text in servo:
                node.nudge_servo(*servo[text])
                return
            if text == 'u':
                node.centre_servo()
                return
            super().keyPressEvent(event)

        def keyReleaseEvent(self, event):
            if event.isAutoRepeat():
                return
            text = event.text().lower()
            if text in {'w', 's', 'q', 'e', 'a', 'd'}:
                node.teleop = Twist()
                self.held.discard(text)
                self.pad.set_held(self.held)
                self.lbl_drive.setText('idle' if not self.held else 'driving')
                return
            super().keyReleaseEvent(event)

        # ------------------------------------------------------------ refresh
        def refresh(self):
            s = node.snapshot()

            # mode
            mode = s['mode']
            if mode is None:
                self.lbl_mode_hdr.setText('no data')
                self._style(self.lbl_mode_hdr, 'warn')
            else:
                self.lbl_mode_hdr.setText(MODE_NAMES.get(mode, str(mode)))
                self._style(self.lbl_mode_hdr,
                            'bad' if mode == 0 else 'good')
            for m, b in self.mode_buttons.items():
                b.setChecked(mode == m)

            # battery
            v = s['voltage']
            if v != v:
                self.lbl_bat.setText('—')
                self._style(self.lbl_bat, 'dim')
            else:
                self.lbl_bat.setText(f'{v:.1f} V')
                self._style(self.lbl_bat, 'bad' if v < 10.5 else 'good')

            # video
            hz = s['image_hz']
            self.lbl_video.setText(f'{hz:.0f} fps')
            self._style(self.lbl_video, 'good' if hz > 5 else 'dim')

            # camera image
            jpeg = node.latest_jpeg()
            if jpeg is not None and jpeg is not self._last_jpeg:
                self._last_jpeg = jpeg
                pix = QtGui.QPixmap()
                if pix.loadFromData(jpeg, 'JPEG'):
                    self.lbl_cam.setPixmap(pix.scaled(
                        self.lbl_cam.size(), QtCore.Qt.KeepAspectRatio,
                        QtCore.Qt.SmoothTransformation))
            elif jpeg is None:
                self.lbl_cam.setText('waiting for /image_throttled/compressed…')

            # servo bars
            self.bar_pan.set_value(node.servo['s2'])
            self.bar_tilt.set_value(node.servo['s1'])
            self.lbl_pan.setText(f"{node.servo['s2']:.0f}")
            self.lbl_tilt.setText(f"{node.servo['s1']:.0f}")

            # gaze
            raw = node.latest_raw()
            ox, oy, gx, gy = s['calib']
            if raw is None:
                self.gaze_map.set_points(None, None, False, True)
                self.lbl_gaze.setText('no source')
                self._style(self.lbl_gaze, 'dim')
            else:
                u, vv, valid, age = raw
                stale = age > 1.0
                cu = min(1.0, max(0.0, (u - ox) * gx + 0.5))
                cv = min(1.0, max(0.0, (vv - oy) * gy + 0.5))
                self.gaze_map.set_points((u, vv), (cu, cv), valid, stale)
                if stale:
                    self.lbl_gaze.setText(f'stale {age:.1f}s')
                    self._style(self.lbl_gaze, 'warn')
                elif not valid:
                    self.lbl_gaze.setText('face lost')
                    self._style(self.lbl_gaze, 'warn')
                else:
                    self.lbl_gaze.setText(f'u={u:.2f} v={vv:.2f}')
                    self._style(self.lbl_gaze, 'good')
            self.lbl_zone.setText(s['zone'] or '—')
            self.lbl_calib.setText(
                f'offset {ox:+.2f},{oy:+.2f}   gain {gx:.2f},{gy:.2f}')

            # Map. Repainting the occupancy grid is the expensive part of a
            # refresh, so only touch it when the data actually changed: the map
            # updates on scan matching, not on a timer.
            #
            # The signature starts as None rather than as the current values.
            # Comparing "None is not None" on the first refresh is False, so an
            # equality-only guard would never run the first update and the map
            # would stay blank forever.
            m, sc, pose = node.map_snapshot()
            sig = (id(m), id(sc), pose)
            if sig != self._map_sig:
                self._map_sig = sig
                self.map_view.update_view(
                    m, sc, pose,
                    time.monotonic() - node._scan_stamp if sc else 99.0,
                    time.monotonic() - node._map_stamp if m else 99.0)

                # A one-line verdict next to the map. "Laser points not on the
                # walls" means the pose is wrong, and the fix is Find me - say
                # so here rather than leaving the operator to work it out from
                # the picture.
                if m is None:
                    self.lbl_map.setText('no map')
                    self._style(self.lbl_map, 'dim')
                elif pose is None:
                    self.lbl_map.setText('map ok · pose unknown')
                    self._style(self.lbl_map, 'warn')
                else:
                    self.lbl_map.setText(
                        f'x {pose[0]:+.2f}  y {pose[1]:+.2f}  '
                        f'{math.degrees(pose[2]):+.0f}°')
                    self._style(self.lbl_map, 'good')

            # audio
            txt = s['audio']
            if txt:
                d = dict(re.findall(r'(\w+)=(\S+)', txt))
                sent = int(d.get('sent', 0) or 0)
                recv = int(d.get('recv', 0) or 0)
                live = d.get('live') == 'True'
                enabled = d.get('enabled') == 'True'
                self.meter_sent.set_value(min(1.0, sent / 500.0))
                self.meter_recv.set_value(min(1.0, recv / 500.0))
                # "live" on the robot means inbound audio has arrived recently,
                # i.e. the operator is being heard. Spell the direction out:
                # the bare word "live" was easy to misread as "audio works".
                cap_ok = d.get('cap_ok') == 'True'
                play_ok = d.get('play_ok') == 'True'
                if not enabled:
                    self.lbl_audio.setText('muted')
                    self._style(self.lbl_audio, 'dim')
                elif not (cap_ok and play_ok):
                    # The device died. Say which one: "no audio" and "this end
                    # cannot capture at all" look identical from the counters,
                    # and only one of them is the operator's fault.
                    bad = []
                    if not cap_ok:
                        bad.append('mic')
                    if not play_ok:
                        bad.append('speaker')
                    self.lbl_audio.setText(
                        f'DEVICE DOWN: {" + ".join(bad)}')
                    self._style(self.lbl_audio, 'bad')
                elif live:
                    self.lbl_audio.setText(
                        f'2-way · out {sent} · in {recv}')
                    self._style(self.lbl_audio, 'good')
                else:
                    self.lbl_audio.setText(
                        f'out {sent} · robot hears nothing from me')
                    self._style(self.lbl_audio, 'warn')
            elif self.audio_on:
                self.lbl_audio.setText('no bridge on robot')
                self._style(self.lbl_audio, 'warn')

        def closeEvent(self, event):
            """Stop the robot, then clean up anything this window started.

            Closing the console used to leave SLAM and RViz running. The next
            launch then added a *second* slam_toolbox, and two mappers
            publishing /map with different maps makes the map appear to jump
            around. That is the whole reason "2 maps are fighting" happened, so
            the cleanup belongs here rather than in the user's memory.

            The kill script is started detached and not waited for: it needs a
            few seconds to shut nodes down cleanly, and blocking the close for
            that long would look like the window had hung.
            """
            node.estop()
            self._stop_operator_audio()

            if self.chk_stopview.isChecked() and self._view_running():
                script = os.path.expanduser(
                    '~/m1_control_ws/scripts/kill_vm.sh')
                if os.path.exists(script):
                    try:
                        subprocess.Popen(
                            ['bash', script],          # view-only, no --all,
                            start_new_session=True,    # so this process is
                            stdout=subprocess.DEVNULL,  # not in the kill list
                            stderr=subprocess.DEVNULL)
                    except Exception:  # noqa: BLE001 - never block closing
                        pass

            stop_event.set()
            event.accept()

    app = QtWidgets.QApplication(sys.argv)
    win = Console()
    # Maximised by default. The console is the only thing on screen while the
    # robot is being driven, and starting at 1340x900 on a 1080p desktop wastes
    # a third of the display. F11 toggles true fullscreen for when even the
    # title bar is in the way.
    win.showMaximized()
    try:
        app.exec_()
    finally:
        stop_event.set()
        ros_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
