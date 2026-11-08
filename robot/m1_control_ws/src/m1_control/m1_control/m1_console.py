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

try:
    from m1_control.gaze_calib import apply as apply_calib
    from m1_control.gaze_calib import fit as fit_calib
    from m1_control.gaze_calib import GAIN_MAX, GAIN_MIN
except ImportError:  # run as a plain script rather than through ros2 run
    from gaze_calib import apply as apply_calib  # noqa: F401
    from gaze_calib import fit as fit_calib
    from gaze_calib import GAIN_MAX, GAIN_MIN

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
        # Gaze overlay geometry, as a FRACTION OF THE CONTROL SPACE (a radius,
        # not a diameter). These MUST match the HUD's, or the picture the
        # operator sees would disagree with the zones the robot actually uses -
        # the worst kind of wrong, because it looks right.
        self.declare_parameter('deadzone_radius_ratio', 0.30)
        self.declare_parameter('deadzone_radius_ratio_y', 0.16)
        # Must match the HUD's head_weight, same reason as the deadzone.
        self.declare_parameter('head_weight', 0.75)

        gp = self.get_parameter
        self.robot_ip = gp('robot_ip').value
        self.s1_center = float(gp('servo_s1_center').value)
        self.s2_center = float(gp('servo_s2_center').value)
        self.s1_min = float(gp('servo_s1_min').value)
        self.s1_max = float(gp('servo_s1_max').value)
        self.s2_min = float(gp('servo_s2_min').value)
        self.s2_max = float(gp('servo_s2_max').value)
        self.servo_step = float(gp('servo_step').value)
        self.deadzone_ratio = float(gp('deadzone_radius_ratio').value)
        self.deadzone_ratio_y = float(gp('deadzone_radius_ratio_y').value)
        self.head_weight = float(gp('head_weight').value)

        self.state = {
            'mode': None, 'voltage': float('nan'), 'zone': '',
            'image_hz': 0.0, 'audio': '', 'connected': False,
            'gaze_alive': False, 'calib': [0.5, 0.5, 1.0, 1.0],
        }
        self._image_times = []
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._raw = None          # (u, v, valid, monotonic_time)
        self._latest_face = None
        self._face_stamp = 0.0

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
        self.tuning_pub = self.create_publisher(
            Float32MultiArray, '/gaze/tuning', 10)

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
        self.create_subscription(Float32MultiArray, '/gaze/tuning_state',
                                 self._tuning_state_cb, 10)
        self.create_subscription(Point, '/gaze/raw', self._raw_cb, 10)
        self.create_subscription(CompressedImage, '/image_throttled/compressed',
                                 self._image_cb, sensor_qos)
        # The operator's face, so the Gaze panel can show that the webcam can
        # actually see them. Without it a frozen dot and a camera pointed at
        # the ceiling look identical.
        self.create_subscription(CompressedImage, '/gaze/face',
                                 self._face_cb, sensor_qos)

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

    def _tuning_state_cb(self, msg):
        """Adopt whatever the HUD is actually using.

        The sliders publish a request; this is the confirmation. Adopting it
        keeps the overlay honest even if the HUD clamps a value or the request
        never arrived.
        """
        data = list(msg.data)
        if len(data) >= 2:
            self.deadzone_ratio = float(data[0])
            self.deadzone_ratio_y = float(data[1])
        if len(data) >= 3:
            self.head_weight = float(data[2])

    def _raw_cb(self, msg):
        self._raw = (float(msg.x), float(msg.y), float(msg.z) > 0.5,
                     time.monotonic())
        self._set('gaze_alive', self._raw[2])

    def _face_cb(self, msg):
        self._latest_face = bytes(msg.data)
        self._face_stamp = time.monotonic()

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

    def latest_face(self):
        """(jpeg_bytes, age_seconds) or None. Age lets the GUI drop a stale
        face instead of showing the last one forever after tracking stops."""
        if self._latest_face is None:
            return None
        return self._latest_face, time.monotonic() - self._face_stamp

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

    def set_deadzone(self, rx, ry):
        """Ask the HUD for a new deadzone and adopt it locally straight away.

        Adopting immediately rather than waiting for the echo keeps the
        overlay responsive while the slider is being dragged; the echo on
        /gaze/tuning_state then corrects it if the HUD clamped the value.
        """
        rx = min(0.48, max(0.02, float(rx)))
        ry = min(0.48, max(0.02, float(ry)))
        self.deadzone_ratio, self.deadzone_ratio_y = rx, ry
        self.tuning_pub.publish(
            Float32MultiArray(data=[rx, ry, float(self.head_weight)]))

    def set_head_weight(self, k):
        """How much of the gaze signal comes from the head, 0..1.

        Not a preference: at 0 the vertical axis stops working, because the
        iris barely moves up and down inside the eye. See gaze_hud_node.
        """
        k = min(1.0, max(0.0, float(k)))
        self.head_weight = k
        self.tuning_pub.publish(
            Float32MultiArray(data=[float(self.deadzone_ratio),
                                    float(self.deadzone_ratio_y), k]))

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


def list_cameras():
    """Every camera this machine can actually deliver a frame from.

    Returns a list of (index, label). The index is what OpenCV wants.

    Three filters, in increasing cost, because the naive version is wrong on
    both platforms this runs on:

    1. NAME. A Raspberry Pi exposes a dozen /dev/video* nodes that are not
       cameras at all - the ISP (pispbe), the HEVC decoder, the codec blocks.
       They appear in the list and cannot produce a webcam frame.

    2. FRAME. A camera also exposes a METADATA node alongside its capture
       node. It opens perfectly and never returns an image, so isOpened() is
       not enough - a frame has to be read.

    3. INDEX. /dev/video0 is not stable. Unplugging the webcam, or the host
       re-enumerating USB, moves it. That is what made the tracker "stop
       seeing the face" with no explanation.

    Filtering by name first matters for speed as much as correctness: probing
    twenty nodes with OpenCV takes seconds and would freeze the window.

    Never raises - a machine with no camera returns an empty list.
    """
    import glob
    import re

    # Substrings that identify nodes which are not capture devices. Kept
    # deliberately specific: a generic "skip anything with isp in the name"
    # would drop a real camera called "ISP Camera".
    NOT_A_CAMERA = ('pispbe', 'rpi-hevc', 'bcm2835-codec', 'bcm2835-isp',
                    'unicam-image', 'video-encoder', 'video-decoder')

    # Human-readable names, if v4l2-ctl is available. Not required.
    names = {}
    try:
        out = subprocess.run(['v4l2-ctl', '--list-devices'],
                             capture_output=True, text=True, timeout=4).stdout
        current = None
        for line in out.splitlines():
            if line and not line.startswith(('\t', ' ')):
                current = line.split('(')[0].strip().rstrip(':')
            elif line.strip().startswith('/dev/video') and current:
                m = re.search(r'(\d+)$', line.strip())
                if m:
                    names[int(m.group(1))] = current
    except Exception:  # noqa: BLE001 - names are a nicety, not required
        pass

    candidates = []
    for node in sorted(glob.glob('/dev/video*')):
        m = re.search(r'(\d+)$', node)
        if not m:
            continue
        idx = int(m.group(1))
        label = names.get(idx, '')
        if any(bad in label.lower() for bad in NOT_A_CAMERA):
            continue
        candidates.append((idx, label))

    # Verify each candidate really delivers a frame. Bounded so a broken driver
    # cannot hang the window.
    verified = []
    try:
        import cv2
    except ImportError:
        # No OpenCV in this interpreter: fall back to the name filter alone.
        return [(i, ('%s  (/dev/video%d)' % (n, i)) if n else '/dev/video%d' % i)
                for i, n in candidates]

    for idx, label in candidates:
        base = ('%s  (/dev/video%d)' % (label, idx)) if label else '/dev/video%d' % idx
        cap = None
        try:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                ok, _frame = cap.read()
                if ok:
                    verified.append((idx, base, label, True))
                else:
                    # Opened but no frame: the metadata node beside a real
                    # capture node. Recorded so it can be dropped below.
                    verified.append((idx, base, label, False))
            else:
                # COULD NOT OPEN. This is very often "another process is
                # holding it" rather than "not a camera", and hiding those is
                # a trap: the operator sees only the other camera, picks it,
                # and calibrates against the wrong lens without being told.
                verified.append((idx, base + '   [in use]', label, None))
        except Exception:  # noqa: BLE001 - one bad node must not hide the rest
            verified.append((idx, base + '   [in use]', label, None))
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:  # noqa: BLE001
                    pass

    # One physical camera exposes several /dev/video* nodes: a capture node and
    # a metadata node. If any node of a named device delivers a frame, drop the
    # rest of that device's nodes - they are the same lens and listing them
    # again just invites picking the one that never works.
    working = {label for _i, _l, label, ok in verified if ok and label}
    out = []
    for idx, label, name, ok in verified:
        if ok is False and name in working:
            continue
        if ok is None and name in working:
            continue
        out.append((idx, label))
    return out


def make_gaze_map(QtCore, QtGui, QtWidgets):
    """Square plot of the gaze point: raw (hollow) and corrected (solid)."""

    class GazeMap(QtWidgets.QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.raw = None
            self.corrected = None
            self.valid = False
            self.stale = True
            # Deliberately small. This plot answers "is the tracker alive and
            # roughly where", which needs a couple of centimetres of screen,
            # not a third of the panel. The space is better spent on the face
            # thumbnail, which answers a question this cannot: whether the
            # camera can see the operator at all.
            self.setMinimumSize(96, 96)
            self.setMaximumSize(150, 150)
            self.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                               QtWidgets.QSizePolicy.Preferred)

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
                # The raw value is not clamped to 0..1 - a biased tracker
                # legitimately reads outside it - so clamp it for DRAWING or
                # the marker leaves the box and looks like a bug.
                ru = min(1.0, max(0.0, self.raw[0]))
                rv = min(1.0, max(0.0, self.raw[1]))
                p.drawEllipse(to_px(ru, rv), 6, 6)

            if self.corrected is not None and not self.stale:
                col = '#34d399' if self.valid else '#6b7280'
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(col))
                p.drawEllipse(to_px(*self.corrected), 4, 4)

            if self.stale:
                p.setPen(QtGui.QColor('#6b7280'))
                p.drawText(box, QtCore.Qt.AlignCenter, 'no gaze source')

            # Legend. Without it the two markers are unexplained: the amber
            # ring is the RAW tracker output and the filled dot is where the
            # calibration puts it. When they sit far apart, the calibration is
            # pulling hard - which is exactly what the operator needs to see,
            # but only if they know what they are looking at.
            f = p.font()
            f.setPointSizeF(7.0)
            p.setFont(f)
            y = self.height() - 3
            p.setPen(QtGui.QPen(QtGui.QColor('#f59e0b'), 1.4))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawEllipse(QtCore.QPointF(8.0, y - 4.0), 3.5, 3.5)
            p.setPen(QtGui.QColor('#8b939c'))
            p.drawText(QtCore.QRectF(15.0, y - 11.0, 34.0, 14.0),
                       QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, 'raw')
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#34d399'))
            p.drawEllipse(QtCore.QPointF(52.0, y - 4.0), 3.5, 3.5)
            p.setPen(QtGui.QColor('#8b939c'))
            p.drawText(QtCore.QRectF(59.0, y - 11.0, 34.0, 14.0),
                       QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, 'aim')

    return GazeMap


def make_gaze_overlay(QtCore, QtGui):
    """Draws the gaze control space onto the robot camera image.

    Returns draw(pix, zone, raw, calib, dz_x, dz_y, show_debug) -> pix.

    The picture is a faithful map of the CONTROL SPACE, not a claim about where
    on the robot's view the operator is looking. The camera image is simply the
    largest convenient rectangle to draw it in: the deadzone oval, the four
    sectors, and the point the tracker thinks the operator is aiming at, all in
    the same normalized coordinates the zones are actually computed from. So if
    the dot is inside the oval, the chassis really is still; if it is up in the
    top sector, the robot really is about to drive forward.

    The deadzone radii are passed in rather than read from anywhere, because
    the HUD and this drawing MUST use the same numbers. A picture that
    disagrees with the zones the robot actually uses is the worst kind of
    wrong, because it looks right.

    A factory rather than a widget so it can be exercised without a display.
    """

    def draw(pix, zone, raw, calib, dz_x, dz_y, show_debug=True):
        p = QtGui.QPainter(pix)
        try:
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            w, h = pix.width(), pix.height()
            cx, cy = w / 2.0, h / 2.0
            # The ratios ARE radii, as a fraction of the space. No extra
            # halving: doing that is what silently made the deadzone half the
            # size it was supposed to be.
            rx = w * dz_x
            ry = h * dz_y

            # Quadrant lines: these are the actual zone boundaries, where the
            # normalised offsets are compared, so a look straight up lands
            # squarely in FORWARD rather than on a diagonal edge.
            p.setBrush(QtCore.Qt.NoBrush)
            p.setPen(QtGui.QPen(QtGui.QColor(150, 150, 150, 90), 1,
                                QtCore.Qt.DashLine))
            p.drawLine(QtCore.QPointF(cx, 0), QtCore.QPointF(cx, h))
            p.drawLine(QtCore.QPointF(0, cy), QtCore.QPointF(w, cy))

            p.setPen(QtGui.QPen(QtGui.QColor('#22c55e'), 2))
            p.drawEllipse(QtCore.QRectF(cx - rx, cy - ry, 2 * rx, 2 * ry))

            # Zone labels at the middle of each sector, so the operator can see
            # where to look rather than having to infer it from the oval.
            f = p.font()
            f.setPointSizeF(max(7.0, min(11.0, w / 64.0)))
            f.setBold(True)
            p.setFont(f)
            p.setPen(QtGui.QColor(160, 220, 180, 190))
            mid = 0.5
            for text, fx, fy in (('FWD', 0.0, -1.0), ('BACK', 0.0, 1.0),
                                 ('LEFT', -1.0, 0.0), ('RIGHT', 1.0, 0.0)):
                lx = cx + fx * rx * (1.0 + mid)
                ly = cy + fy * ry * (1.0 + mid)
                lx = min(max(lx, 16.0), w - 16.0)
                ly = min(max(ly, 12.0), h - 6.0)
                p.drawText(QtCore.QPointF(lx, ly), text)

            if raw is None:
                p.setPen(QtGui.QColor('#f59e0b'))
                p.drawText(QtCore.QPointF(10, 22), 'GAZE: no source')
                return pix

            u, v, valid, age = raw
            if age > 1.0 or not valid:
                p.setPen(QtGui.QColor('#f59e0b'))
                p.drawText(QtCore.QPointF(10, 22),
                           'GAZE: stale' if age > 1.0 else 'GAZE: face lost')
                return pix

            ox, oy, gx, gy = calib
            cu = min(1.0, max(0.0, (u - ox) * gx + 0.5))
            cv = min(1.0, max(0.0, (v - oy) * gy + 0.5))

            # Raw, hollow: makes a calibration pulling hard visible at a glance,
            # and shows whether the raw signal even reaches the edges.
            p.setPen(QtGui.QPen(QtGui.QColor(245, 158, 11, 190), 1))
            p.setBrush(QtCore.Qt.NoBrush)
            ru = min(1.0, max(0.0, u))
            rv = min(1.0, max(0.0, v))
            p.drawEllipse(QtCore.QPointF(ru * w, rv * h), 8.0, 8.0)

            driving = (zone or 'CENTER') != 'CENTER'
            col = QtGui.QColor('#ef4444' if driving else '#22c55e')
            p.setPen(QtGui.QPen(col, 2))
            p.setBrush(col)
            p.drawEllipse(QtCore.QPointF(cu * w, cv * h), 5.0, 5.0)
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawEllipse(QtCore.QPointF(cu * w, cv * h), 13.0, 13.0)

            if show_debug:
                p.setPen(col)
                p.drawText(QtCore.QPointF(10, 22), 'ZONE: ' + (zone or '-'))
                p.setPen(QtGui.QColor(150, 160, 170))
                p.drawText(QtCore.QPointF(10, 38),
                           'raw %.2f,%.2f  aim %.2f,%.2f' % (u, v, cu, cv))
            return pix
        finally:
            # A QPainter left active on a pixmap keeps painting on it and
            # eventually warns "QPainter::begin: Paint device returned
            # engine == 0". end() must run even if drawing raised.
            p.end()

    return draw


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
            self.setMinimumHeight(150)
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

def _ring(u0, v0, radius, shape, start_angle=0.0):
    """Points on a circle around (u0, v0), in the order given.

    `shape` is 'diamond' for the four cardinal directions, 'corners' for the
    four diagonals, or 'edge' for eight points between them. Everything is
    derived from a radius and an angle so the rings stay concentric and evenly
    spaced instead of drifting into each other as they are edited.
    """
    if shape == 'diamond':
        angles = (270.0, 0.0, 90.0, 180.0)          # up, right, down, left
    elif shape == 'corners':
        angles = (225.0, 315.0, 45.0, 135.0)        # TL, TR, BR, BL
    else:
        angles = tuple(22.5 + 45.0 * i for i in range(8))
    out = []
    for a in angles:
        rad = math.radians(a + start_angle)
        u = round(min(0.98, max(0.02, u0 + radius * math.sin(rad))), 3)
        v = round(min(0.98, max(0.02, v0 - radius * math.cos(rad))), 3)
        out.append((u, v))
    return out


def _ultra_dots():
    """47 dots, built out from the centre, and ending where it started.

    The ORDER is the part that matters, and it is deliberate:

        1. centre                 - establishes the neutral point before
                                    anything moves, so the operator is settled
                                    when the first measurement is taken
        2. outer diamond          - the four cardinal directions, which are the
                                    zones the robot actually uses
        3. outer corners          - the extremes of the range
        4. outer edge midpoints   - fills between the cardinals and corners
        5. inner diamond          - half the radius, where people look most
        6. inner corners
        7. vertical column        - extra samples straight up and down
        8. horizontal row         - extra samples straight left and right
        9. re-check the cardinals - back to the four directions that decide
                                    whether this works, at the END, when the
                                    operator is tired and has drifted

    Step 9 is the "come back and check" step, and it is what makes the measured
    accuracy figure mean something: the same directions are sampled late rather
    than trusting the first few seconds.

    Vertical gets the extra column because the iris barely moves up and down
    inside the eye - most of that signal is head pitch - and the centre column
    is where vertical is measured with the least horizontal interference.
    """
    R_OUT = 0.42
    R_IN = 0.21
    dots = [('CENTRE', 0.50, 0.50, 'Look at the CENTRE dot')]

    def add(prefix, uv_list):
        for i, (u, v) in enumerate(uv_list):
            dots.append(('%s%d' % (prefix, i + 1), u, v,
                         'Look at the dot, and move the way you naturally '
                         'would'))

    add('OD', _ring(0.50, 0.50, R_OUT, 'diamond'))
    add('OC', _ring(0.50, 0.50, R_OUT, 'corners'))
    add('OE', _ring(0.50, 0.50, R_OUT, 'edge'))
    add('ID', _ring(0.50, 0.50, R_IN, 'diamond'))
    add('IC', _ring(0.50, 0.50, R_IN, 'corners'))

    # Vertical column: heights the rings do not already cover.
    for i, v in enumerate((0.15, 0.25, 0.37, 0.63, 0.75, 0.85)):
        dots.append(('VC%d' % (i + 1), 0.50, v,
                     'Look at the CENTRE dot, head and eyes together'))
    # Horizontal row, for the other axis.
    for i, u in enumerate((0.15, 0.25, 0.75, 0.85)):
        dots.append(('HR%d' % (i + 1), u, 0.50, 'Look at the dot'))

    # Drop any point that lands on one already visited. The rings are generated
    # from angles and then rounded to 3 decimals, so a couple of them can
    # collide; keeping both would mean sampling the same place twice and
    # counting it as two independent measurements, which it is not.
    #
    # The rechecks below are exempt: repeating those positions is the entire
    # point of them.
    seen = set()
    core = []
    for d in dots:
        key = (round(d[1], 3), round(d[2], 3))
        if key in seen:
            continue
        seen.add(key)
        core.append(d)

    # Back to the four directions that decide whether this works, at the END,
    # when the operator is tired and has drifted. If accuracy is going to fall
    # away, this is where it shows - and repeating a position on purpose is
    # what makes it a check rather than just more training data.
    for i, (u, v) in enumerate(((0.50, 0.08), (0.92, 0.50),
                                (0.50, 0.92), (0.08, 0.50))):
        core.append(('RECHECK%d' % (i + 1), u, v,
                     'Back to the edge - look at this dot'))
    core.append(('RECHECK5', 0.50, 0.50, 'Back to the CENTRE dot'))
    return core


#: Built once, so the label below can state the real count. A hard-coded "47"
#: in a label is a number that will eventually be wrong, and a profile whose
#: label lies about how long it takes is worse than one with no label.
_ULTRA_DOTS = _ultra_dots()

#: Re-visited AFTER the fit to measure how accurate it actually is. These are
#: the five directions the operator complains about, so the report speaks in
#: the same terms they think in.
VALIDATION_DOTS = [
    ('CENTRE', 0.50, 0.50),
    ('TOP', 0.50, 0.08),
    ('BOTTOM', 0.50, 0.92),
    ('LEFT', 0.08, 0.50),
    ('RIGHT', 0.92, 0.50),
]


class CalibDialog:
    """Gaze calibration spread across the WHOLE screen.

    Why the whole screen and not the video panel
    --------------------------------------------
    An eye tracker's raw signal is tiny. Iris position changes by a few
    thousandths of a frame between looking at the middle of a screen and
    looking at its edge. The fit recovers `gain` by dividing that small change
    into a large target change, so the smaller the calibration area, the more
    the gain has to multiply - and the more it multiplies the tracker's noise
    along with the signal.

    Calibrating inside the video panel meant asking for eye movement across a
    few square centimetres. Every sample landed within a hair of every other,
    the gain was either clamped or meaningless, and the zones were unreachable.
    That is what "it does not work, it needs the whole screen" was describing.

    Dots now sit at 10% and 90% of the window in each direction, so the eyes
    travel as far as they comfortably can and the fit has real signal.

    Profiles
    --------
      fast  5 dots - centre, top, right, bottom, left. About 10 s.
            A re-alignment. It recovers offset and gain on both axes, which is
            exactly the four numbers this robot uses. Use it after shifting in
            your seat.

      long  9 dots - a 3x3 grid. About 30 s.
            The same four numbers, sampled across the middle as well, so the
            fit is not extrapolated from the edges alone and one bad dot can be
            detected and dropped.

      super 20 dots - weighted towards the middle and the vertical axis.
            About a minute. The fit has two unknowns per axis, so more dots do
            not add parameters - they add CONFIDENCE, and they let the outlier
            rejection actually work. Vertical is where this tracker is weakest
            (the iris barely moves up and down inside the eye, so most of the
            signal is head pitch), so the grid deliberately samples the centre
            column and the centre band far more densely than the corners.

    The model, per axis:   corrected = (raw - offset) * gain + 0.5
    Fitting a line  target = gain * raw + b  across the dots, then
        gain = slope        offset = (0.5 - b) / gain
    recovers both numbers by least squares for any number of dots, which is
    why one fitter serves all three profiles.
    """

    PROFILES = {
        # Centre first in every profile: it establishes the neutral point
        # before anything moves, so the operator is settled when the first
        # measurement is taken. Starting at a corner means the very first
        # samples are taken while they are still finding a comfortable
        # position, which biases the offset.
        'fast': {
            'label': 'Fast \u00b7 5 dots \u00b7 ~10 s',
            'settle': 0.9, 'sample': 1.0,
            'dots': [
                ('CENTRE', 0.50, 0.50, 'Look at the CENTRE dot'),
                ('TOP', 0.50, 0.10, 'Look at the TOP dot'),
                ('RIGHT', 0.90, 0.50, 'Look at the RIGHT dot'),
                ('BOTTOM', 0.50, 0.90, 'Look at the BOTTOM dot'),
                ('LEFT', 0.10, 0.50, 'Look at the LEFT dot'),
            ],
        },
        # Same five, then the four corners. The diamond first means the four
        # zone directions are measured early, while attention is best; the
        # corners only refine the gain.
        'long': {
            'label': 'Long \u00b7 9 dots \u00b7 ~30 s',
            'settle': 1.0, 'sample': 1.2,
            'dots': [
                ('CENTRE', 0.50, 0.50, 'Look at the CENTRE dot'),
                ('TOP', 0.50, 0.08, 'Look at the TOP dot'),
                ('RIGHT', 0.92, 0.50, 'Look at the RIGHT dot'),
                ('BOTTOM', 0.50, 0.92, 'Look at the BOTTOM dot'),
                ('LEFT', 0.08, 0.50, 'Look at the LEFT dot'),
                ('TOP-LEFT', 0.08, 0.08, 'Look at the TOP-LEFT corner'),
                ('TOP-RIGHT', 0.92, 0.08, 'Look at the TOP-RIGHT corner'),
                ('BOTTOM-RIGHT', 0.92, 0.92, 'Look at the BOTTOM-RIGHT corner'),
                ('BOTTOM-LEFT', 0.08, 0.92, 'Look at the BOTTOM-LEFT corner'),
            ],
        },
        # Centre, diamond, corners, then the inner ring, then the vertical
        # column. The last three revisit the centre and the two vertical
        # extremes, because vertical is the axis that keeps failing and the
        # end of a run is when the operator has drifted.
        'super': {
            'label': 'Super \u00b7 20 dots \u00b7 ~1 min',
            'settle': 1.2, 'sample': 1.2,
            'dots': [
                ('CENTRE', 0.50, 0.50, 'Look at the CENTRE dot'),
                ('TOP', 0.50, 0.08, 'Look at the TOP dot'),
                ('RIGHT', 0.92, 0.50, 'Look at the RIGHT dot'),
                ('BOTTOM', 0.50, 0.92, 'Look at the BOTTOM dot'),
                ('LEFT', 0.08, 0.50, 'Look at the LEFT dot'),
                ('TOP-LEFT', 0.08, 0.08, 'Look at the TOP-LEFT corner'),
                ('TOP-RIGHT', 0.92, 0.08, 'Look at the TOP-RIGHT corner'),
                ('BOTTOM-RIGHT', 0.92, 0.92, 'Look at the BOTTOM-RIGHT corner'),
                ('BOTTOM-LEFT', 0.08, 0.92, 'Look at the BOTTOM-LEFT corner'),
                # Inner ring, half the radius: fills the middle, which is where
                # the operator actually looks most of the time.
                ('IN-TOP', 0.50, 0.28, 'Look just above the CENTRE dot'),
                ('IN-RIGHT', 0.72, 0.50, 'Look just right of centre'),
                ('IN-BOTTOM', 0.50, 0.72, 'Look just below the CENTRE dot'),
                ('IN-LEFT', 0.28, 0.50, 'Look just left of centre'),
                ('IN-TOP-LEFT', 0.28, 0.28, 'Look at the inner corner'),
                ('IN-TOP-RIGHT', 0.72, 0.28, 'Look at the inner corner'),
                ('IN-BOTTOM-RIGHT', 0.72, 0.72, 'Look at the inner corner'),
                ('IN-BOTTOM-LEFT', 0.28, 0.72, 'Look at the inner corner'),
                # Back to the axis that matters, at the end.
                ('RECHECK TOP', 0.50, 0.08, 'Back to the TOP dot'),
                ('RECHECK CENTRE', 0.50, 0.50, 'Back to the CENTRE dot'),
                ('RECHECK BOTTOM', 0.50, 0.92, 'Back to the BOTTOM dot'),
            ],
        },
        'ultra': {
            'label': 'Ultra \u00b7 %d dots \u00b7 ~2 min' % len(_ULTRA_DOTS),
            'settle': 1.6, 'sample': 1.4,
            'dots': _ULTRA_DOTS,
        },
    }

    #: Per-dot timings ramp down across a run: the first dots are slow so the
    #: operator can settle into a posture, and the last are quick because by
    #: then they know the rhythm. 0.55 means the final dot takes 55% as long as
    #: the first.
    #:
    #: A flat timing is what made the 47-dot run feel endless. The tail of a
    #: long run is also where attention fades, so speeding it up there costs
    #: less than it would at the start.
    RAMP = 0.55

    #: Never let the ramp squeeze a sample window below this, whatever the
    #: profile says. The HUD publishes at 30 Hz and this dialog ticks at 50 ms,
    #: so 0.5 s is about 10 usable samples - comfortably above MIN_SAMPLES.
    #:
    #: Without this floor the fastest dots of a ramped run collected three
    #: samples and were discarded as "tracker lost your eyes", which is a lie:
    #: the window was simply too short to collect any.
    MIN_SAMPLE_SEC = 0.5

    # A dot with fewer usable samples than this is treated as lost, and the
    # whole run is refused rather than fitted from noise.
    MIN_SAMPLES = 5

    # The dot shrinks from PULSE_MAX to PULSE_MIN over the settle phase, so it
    # is smallest exactly when sampling begins. See _pulse_radius().
    PULSE_MAX = 34.0
    PULSE_MIN = 9.0

    @classmethod
    def build(cls, QtCore, QtGui, QtWidgets):
        class _Dialog(QtWidgets.QDialog):

            def __init__(self, node, area_rect, parent=None):
                super().__init__(parent)
                self.node = node
                # GLOBAL screen rect this dialog should cover - the whole
                # console window, which is maximised, so in practice the whole
                # screen.
                #
                # It used to be the camera picture, which is only a few hundred
                # pixels wide inside the window. Calibrating in that rectangle
                # asked the operator to move their eyes across a few square
                # centimetres, so every raw sample landed within a hair of every
                # other and the fit had almost no signal to work with. The
                # resulting gain was near-useless and the whole thing felt like
                # it "did not work".
                #
                # Spread across the full screen, the same eye movement produces
                # a far larger signal, which is what makes the gain - and so the
                # reach of the zones - meaningful.
                #
                # The dots themselves are placed by _dot_area(), which trims
                # this rect to exclude the control bar.
                self.screen_area = QtCore.QRect(area_rect)
                self.profile = 'fast'
                # Deadzone proportions, so the preview oval matches the robot.
                self.dz_x = float(node.deadzone_ratio)
                self.dz_y = float(node.deadzone_ratio_y)
                self.result_calib = None
                self.idx = -1
                self.phase = 'idle'
                self.samples = []
                self.captured = []
                self.u = None
                self.v = None
                self.pulse_t0 = None
                # Validation state. After the fit the five cardinal directions
                # are re-visited to MEASURE how accurate the result is, rather
                # than trusting the fit's own error estimate.
                self.val_samples = []
                self.val_idx = 0
                self.val_errors = []
                # Always present, so the caller can read them without a
                # getattr guard even when the run is cancelled early.
                self.quality = None
                self.accuracy = None
                self.accuracy_worst = None
                self.timer = QtCore.QTimer(self)
                self.timer.timeout.connect(self._tick)
                self.phase_end = 0.0
                # Separate from the state-machine timer: this one exists only
                # to repaint, so the shrink looks continuous rather than
                # stepping at the sample rate.
                self.anim = QtCore.QTimer(self)
                self.anim.timeout.connect(self.update)

                # Frameless and translucent. A QDialog with a parent is still
                # a top-level window, so its geometry is in global screen
                # coordinates - that is what lets the dots land exactly on the
                # camera image instead of on some other rectangle.
                self.setWindowFlags(QtCore.Qt.FramelessWindowHint |
                                    QtCore.Qt.Dialog)
                self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
                self.setModal(True)
                self.setWindowTitle('Gaze calibration')
                # Cover exactly the area we were told to cover. Deriving it
                # from the parent instead put the dialog at the parent's
                # top-left with the parent's size, which happens to be the same
                # thing for a maximised console but silently disagrees with
                # `screen_area` - and the dots are placed from screen_area, so
                # the two must not be allowed to drift apart.
                self.setGeometry(QtCore.QRect(self.screen_area))

                v = QtWidgets.QVBoxLayout(self)
                v.setContentsMargins(0, 0, 0, 22)
                v.addStretch(1)

                bar = QtWidgets.QFrame()
                bar.setObjectName('calibbar')
                bar.setStyleSheet(
                    '#calibbar { background: rgba(14,17,22,0.97);'
                    ' border:1px solid #2b3138; border-radius:8px; }')
                # A MINIMUM as well as a maximum. With only a maximum the bar
                # shrank to its sizeHint, which put a 300 px box in the middle
                # of a 1920 px screen with the hint text wrapped into it - the
                # cramped, clipped layout that was reported.
                bar.setMinimumWidth(620)
                bar.setMaximumWidth(820)
                bv = QtWidgets.QVBoxLayout(bar)
                bv.setContentsMargins(18, 14, 18, 14)
                bv.setSpacing(10)

                self.lbl = QtWidgets.QLabel('Ready')
                self.lbl.setObjectName('h1')
                self.lbl.setAlignment(QtCore.Qt.AlignCenter)
                bv.addWidget(self.lbl)

                self.hint = QtWidgets.QLabel(
                    'The dots spread across the WHOLE screen. Look at each one '
                    'the way you naturally would — eyes and head together.\n'
                    'Do not strain your eyes to keep your head still: the '
                    'tracker uses your head movement, and calibrating how you '
                    'actually move is what makes it work.')
                self.hint.setObjectName('dim')
                self.hint.setAlignment(QtCore.Qt.AlignCenter)
                self.hint.setWordWrap(True)
                self.hint.setMinimumHeight(38)
                bv.addWidget(self.hint)

                # The result gets its OWN label, in its own colour, with a
                # fixed height so it cannot be squeezed to nothing.
                #
                # This is why the previous quality report was invisible: it was
                # appended to `hint`, which is a dim wrapped paragraph already
                # holding several lines of instructions. The extra text pushed
                # the label past what the bar could show and it was simply
                # clipped away - the calibration appeared to end with no
                # verdict at all.
                self.result = QtWidgets.QLabel('')
                self.result.setAlignment(QtCore.Qt.AlignCenter)
                self.result.setWordWrap(True)
                self.result.setMinimumHeight(46)
                self.result.setStyleSheet(
                    'color:#bbf7d0; background:#14532d;'
                    'border:1px solid #16a34a; border-radius:6px;'
                    'padding:6px; font-weight:bold;')
                self.result.hide()
                bv.addWidget(self.result)

                prow = QtWidgets.QHBoxLayout()
                prow.addWidget(QtWidgets.QLabel('Profile'))
                self.cmb_profile = QtWidgets.QComboBox()
                # cls, not self: PROFILES lives on the outer CalibDialog class
                # and _Dialog is a nested class, so self.PROFILES would only
                # work if it were re-declared here - which is exactly the bug
                # this line had.
                for key in ('fast', 'long', 'super', 'ultra'):
                    self.cmb_profile.addItem(cls.PROFILES[key]['label'], key)
                self.cmb_profile.currentIndexChanged.connect(self._on_profile)
                prow.addWidget(self.cmb_profile, 1)
                bv.addLayout(prow)

                self.status = QtWidgets.QLabel('')
                self.status.setObjectName('mono')
                self.status.setAlignment(QtCore.Qt.AlignCenter)
                bv.addWidget(self.status)

                row = QtWidgets.QHBoxLayout()
                self.btn_go = QtWidgets.QPushButton('Start')
                self.btn_go.setObjectName('primary')
                self.btn_go.clicked.connect(self.start)
                self.btn_cancel = QtWidgets.QPushButton('Cancel  (Esc)')
                self.btn_cancel.clicked.connect(self.reject)
                row.addWidget(self.btn_go)
                row.addWidget(self.btn_cancel)
                bv.addLayout(row)

                v.addWidget(bar, 0, QtCore.Qt.AlignHCenter)
                self.bar = bar

            # ------------------------------------------------------- painting
            def _on_profile(self, _i):
                self.profile = self.cmb_profile.currentData() or 'fast'

            def keyPressEvent(self, e):
                if e.key() == QtCore.Qt.Key_Escape:
                    self.reject()
                    return
                super().keyPressEvent(e)

            def _dot_area(self):
                """Where the dots may be drawn, in this dialog's coordinates.

                The whole window MINUS the control bar at the bottom.

                The bottom dots used to be drawn behind that bar and simply
                disappeared, which is fatal for a calibration: the operator
                has no idea whether to keep holding their gaze there, and the
                samples taken while the dot is invisible are taken looking at
                whatever they happened to be looking at.

                Shrinking the area slightly costs nothing. What matters for
                the fit is how far apart the dots are in EYE movement, and the
                area is still most of a maximised window.
                """
                full = QtCore.QRect(0, 0, self.width(), self.height())
                bar = getattr(self, 'bar', None)
                if bar is None:
                    return full
                top = bar.mapTo(self, QtCore.QPoint(0, 0)).y()
                if top <= 0:
                    return full
                return QtCore.QRect(0, 0, full.width(), top - 8)

            def paintEvent(self, _e):
                p = QtGui.QPainter(self)
                p.setRenderHint(QtGui.QPainter.Antialiasing)

                full = QtCore.QRect(0, 0, self.width(), self.height())
                area = self._dot_area()

                # Dim the whole window, then lift the calibration area back to
                # full brightness. The operator keeps seeing the view they
                # drive with while the dots are on top of it.
                p.fillRect(full, QtGui.QColor(6, 8, 11, 150))

                p.setPen(QtGui.QPen(QtGui.QColor('#3b82f6'), 2))
                p.drawRect(area.adjusted(1, 1, -1, -1))

                # Where the deadzone will sit, drawn as a dashed oval so the
                # operator can see before starting how far they will have to
                # look to drive. Same proportions the robot will use.
                cx = area.left() + area.width() * 0.5
                cy = area.top() + area.height() * 0.5
                rx = area.width() * self.dz_x
                ry = area.height() * self.dz_y
                p.setPen(QtGui.QPen(QtGui.QColor(34, 197, 94, 130), 2,
                                    QtCore.Qt.DashLine))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawEllipse(QtCore.QRectF(cx - rx, cy - ry, 2 * rx, 2 * ry))

                if self.u is None:
                    return

                c = QtCore.QPointF(area.left() + area.width() * self.u,
                                   area.top() + area.height() * self.v)
                self._draw_dot(p, c)

            def _pulse_radius(self):
                """How big the dot is right now.

                The dot shrinks from PULSE_MAX to PULSE_MIN across the settle
                phase, so it is smallest at the exact moment sampling starts.

                This is the instruction, not decoration. A static dot gives no
                cue about when to start looking at it, and people arrive late -
                which puts the first samples of every dot on the PREVIOUS
                dot's position. That is a small, systematic, one-directional
                error repeated twenty times, and it is exactly the kind of
                thing that makes a calibration feel like it did nothing.
                """
                if self.pulse_t0 is None:
                    return cls.PULSE_MIN
                settle = cls.PROFILES[self.profile]['settle']
                frac = (time.monotonic() - self.pulse_t0) / max(1e-3, settle)
                frac = min(1.0, max(0.0, frac))
                return cls.PULSE_MAX - (cls.PULSE_MAX - cls.PULSE_MIN) * frac

            def _draw_dot(self, p, c):
                """Red, shrinking, with a soft halo.

                Red rather than blue because the rest of this screen is blue
                and the one thing the operator must find instantly is the dot.
                The halo is drawn at a fraction of the core so it shrinks with
                it instead of staying a fixed ring around nothing.
                """
                r = self._pulse_radius()
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(239, 68, 68, 40))
                p.drawEllipse(c, r * 2.2, r * 2.2)
                p.setBrush(QtGui.QColor(239, 68, 68, 100))
                p.drawEllipse(c, r * 1.45, r * 1.45)
                p.setBrush(QtGui.QColor('#ef4444'))
                p.drawEllipse(c, r, r)
                # A small white core so the exact centre is unambiguous even
                # when the dot is at its largest.
                p.setBrush(QtGui.QColor('#fee2e2'))
                p.drawEllipse(c, max(2.0, r * 0.28), max(2.0, r * 0.28))

            # -------------------------------------------------------- control
            def _dots(self):
                return cls.PROFILES[self.profile]['dots']

            def start(self):
                raw = self.node.latest_raw()
                if raw is None or raw[3] > 2.0:
                    self.lbl.setText('No gaze source')
                    self.hint.setText(
                        'Nothing is publishing /gaze/raw yet.\n\n'
                        'Start the gaze HUD on this machine first:\n'
                        '    Gaze panel -> Start gaze\n'
                        'and wait for "gaze started" in the log.')
                    return
                self.btn_go.setEnabled(False)
                self.cmb_profile.setEnabled(False)
                self.idx = 0
                self.captured = []
                self.anim.start(33)
                self._begin_point()

            def _timing(self):
                """(settle, sample) for the dot about to be shown.

                Ramps down across the run: slow at the start so the operator can
                settle into a posture, quick at the end because by then they
                know the rhythm. A flat timing is what made the 47-dot run feel
                endless, and the tail of a long run is where attention fades
                anyway, so speeding up there costs least.
                """
                spec = cls.PROFILES[self.profile]
                total = max(1, len(self._dots()))
                frac = min(1.0, self.idx / float(total))
                k = 1.0 - (1.0 - cls.RAMP) * frac
                settle = max(cls.MIN_SAMPLE_SEC, spec['settle'] * k)
                sample = max(cls.MIN_SAMPLE_SEC, spec['sample'] * k)
                return settle, sample

            def _begin_point(self):
                name, u, v, text = self._dots()[self.idx]
                self.u, self.v = u, v
                # Restart the shrink for every dot: the dot appearing large
                # and contracting is the cue to look at it NOW.
                self.pulse_t0 = time.monotonic()
                self.lbl.setText(
                    f'{name}  ·  {self.idx + 1} of {len(self._dots())}')
                self.hint.setText(text)
                self.samples = []
                settle, _sample = self._timing()
                self.phase = 'settle'
                self.phase_end = time.monotonic() + settle
                self.timer.start(50)
                self.update()

            def _begin_validation(self):
                """Re-visit the five cardinal directions and MEASURE the error.

                The fit reports how well it reproduces the dots it was trained
                on, which is not the same as being accurate. This visits points
                the fit has already seen the direction of, and reports the
                actual distance between where the operator looked and where the
                model says they looked - per direction, in the words the
                operator uses.
                """
                self.phase = 'val_settle'
                self.val_idx = 0
                self.val_samples = []
                self.val_errors = []
                self._begin_val_point()

            def _begin_val_point(self):
                name, u, v = VALIDATION_DOTS[self.val_idx]
                self.u, self.v = u, v
                self.pulse_t0 = time.monotonic()
                self.lbl.setText(f'Checking  ·  {name}')
                self.hint.setText('Look at the dot — this measures the result')
                self.samples = []
                self.phase = 'val_settle'
                self.phase_end = time.monotonic() + 0.9
                self.timer.start(50)
                self.update()

            def _tick(self):
                now = time.monotonic()
                left = max(0.0, self.phase_end - now)
                raw = self.node.latest_raw()

                if self.phase == 'settle':
                    self.status.setText(f'get your eyes on the dot… {left:.1f}s')
                    if left <= 0:
                        self.phase = 'sample'
                        _settle, sample = self._timing()
                        self.phase_end = now + sample
                    return

                if self.phase == 'sample':
                    if raw is not None and raw[2] and raw[3] < 0.5:
                        self.samples.append((raw[0], raw[1]))
                    self.status.setText(
                        f'capturing {len(self.samples)} samples  {left:.1f}s')
                    if left <= 0:
                        self._finish_point()
                    return

                if self.phase == 'val_settle':
                    self.status.setText(f'checking… {left:.1f}s')
                    if left <= 0:
                        self.phase = 'val_sample'
                        self.phase_end = now + 0.9
                    return

                if self.phase == 'val_sample':
                    if raw is not None and raw[2] and raw[3] < 0.5:
                        self.samples.append((raw[0], raw[1]))
                    self.status.setText(
                        f'checking {len(self.samples)} samples  {left:.1f}s')
                    if left <= 0:
                        self._finish_val_point()
                    return

                if self.phase == 'done':
                    self.timer.stop()

            def _finish_val_point(self):
                name, u, v = VALIDATION_DOTS[self.val_idx]
                if len(self.samples) >= cls.MIN_SAMPLES:
                    n = len(self.samples)
                    mu = sum(s[0] for s in self.samples) / n
                    mv = sum(s[1] for s in self.samples) / n
                    cu, cv = apply_calib(mu, mv, self.result_calib)
                    # Report in pixels of a maximised window, because that is
                    # the unit the operator can judge.
                    px = math.hypot((cu - u) * 1920, (cv - v) * 1080)
                    self.val_errors.append((name, px))
                self.val_idx += 1
                if self.val_idx < len(VALIDATION_DOTS):
                    self._begin_val_point()
                else:
                    self._report_accuracy()

            def _report_accuracy(self):
                """Show the measured error per direction, and say what it means."""
                self.phase = 'done'
                self.timer.stop()
                self.anim.stop()
                self.u = self.v = None

                if not self.val_errors:
                    worst = 0.0
                    summary = 'Could not measure accuracy (no samples).'
                else:
                    worst = max(e for _n, e in self.val_errors)
                    parts = '   '.join('%s %d' % (n, round(e))
                                       for n, e in self.val_errors)
                    summary = 'Measured error (px):  ' + parts

                self.accuracy = dict(self.val_errors)
                self.accuracy_worst = worst

                # The worst direction is the actionable one, so it gets named.
                if self.val_errors and worst > 220:
                    bad = max(self.val_errors, key=lambda e: e[1])[0]
                    verdict = (f'{bad} is off by {worst:.0f} px. Raise the Head '
                               f'slider and run this again.')
                elif self.val_errors and worst > 110:
                    bad = max(self.val_errors, key=lambda e: e[1])[0]
                    verdict = f'{bad} is the weakest direction ({worst:.0f} px).'
                elif self.val_errors:
                    verdict = 'Accurate in all five directions.'
                else:
                    verdict = 'Press Apply to use this calibration.'

                self.lbl.setText('Calibration complete')
                self.result.setText(summary + '\n' + verdict)
                self.result.show()
                self.hint.setText(
                    f'offset=({self.result_calib[0]:+.3f}, '
                    f'{self.result_calib[1]:+.3f})   '
                    f'gain=({self.result_calib[2]:.2f}, '
                    f'{self.result_calib[3]:.2f})')
                self.status.setText('Press Apply to send this to the robot.')
                self.btn_go.setText('Apply')
                self.btn_go.setEnabled(True)
                try:
                    self.btn_go.clicked.disconnect()
                except TypeError:
                    pass
                self.btn_go.clicked.connect(self.accept)
                self.update()

            def _finish_point(self):
                if len(self.samples) < cls.MIN_SAMPLES:
                    self.lbl.setText('Not enough samples')
                    self.hint.setText(
                        'The tracker lost your eyes on that dot. Check the '
                        'camera and the lighting, then press Retry.')
                    self.btn_go.setText('Retry')
                    self.btn_go.setEnabled(True)
                    self.cmb_profile.setEnabled(True)
                    self.phase = 'idle'
                    self.u = self.v = None
                    self.timer.stop()
                    self.anim.stop()
                    self.update()
                    return
                n = len(self.samples)
                mu = sum(s[0] for s in self.samples) / n
                mv = sum(s[1] for s in self.samples) / n
                self.captured.append((mu, mv))
                self.status.setText(
                    f'got {n} samples  (raw u={mu:.3f} v={mv:.3f})')

                self.idx += 1
                if self.idx < len(self._dots()):
                    self._begin_point()
                else:
                    self._compute()

            def _quality(self, calib, dots):
                """How well the fitted model reproduces the dots it was given.

                This is the honest answer to "is this calibration any good?".
                Without it the only feedback was four numbers that look
                plausible whatever they are - which is exactly how a broken
                calibration can run to completion, report success, and still
                steer wrong.

                Returns (rms_u, rms_v, worst_dot_name, clamped_u, clamped_v)
                in normalized units, where 0.01 is about 19 px on a 1920-wide
                screen.
                """
                su = sv = 0.0
                worst = (0.0, '')
                for (mu, mv), d in zip(self.captured, dots):
                    cu, cv = apply_calib(mu, mv, calib)
                    eu = cu - d[1]
                    ev = cv - d[2]
                    su += eu * eu
                    sv += ev * ev
                    e = abs(eu) + abs(ev)
                    if e > worst[0]:
                        worst = (e, d[0])
                n = max(1, len(self.captured))
                ox, oy, gx, gy = calib

                # A gain pinned at a limit means the fit could not use the
                # data, and it matters in BOTH directions:
                #
                #   at GAIN_MAX  the raw signal barely moved between dots, so
                #                the fit is extrapolating from almost nothing.
                #                That is a weak signal - the vertical axis on
                #                this hardware, where the iris hardly moves up
                #                and down inside the eye.
                #   at GAIN_MIN  the raw signal moved far more than the dots
                #                did, which is what noise-dominated tracking
                #                looks like.
                #
                # Either way it cannot be fixed by calibrating again, so it
                # has to be reported rather than averaged away.
                def at_limit(g):
                    return (g >= GAIN_MAX - 1e-9 or g <= GAIN_MIN + 1e-9)

                return (math.sqrt(su / n), math.sqrt(sv / n), worst[1],
                        at_limit(gx), at_limit(gy))

            def _compute(self):
                dots = self._dots()
                n = len(self.captured)

                # The maths lives in gaze_calib.py so the HUD, which applies
                # the result, cannot disagree with the console, which produces
                # it: both import the same function.
                ox, oy, gx, gy = fit_calib(
                    self.captured, [(d[1], d[2]) for d in dots])
                calib = (ox, oy, gx, gy)

                rms_u, rms_v, worst_dot, clamped_u, clamped_v = \
                    self._quality(calib, dots)
                self.quality = {
                    'rms_u': rms_u, 'rms_v': rms_v, 'worst': worst_dot,
                    'clamped_u': clamped_u, 'clamped_v': clamped_v,
                    'dots': n,
                }

                self.result_calib = calib

                # Fit quality first: if a gain is pinned, the SIGNAL is wrong
                # and re-visiting dots will not tell us anything new, so say so
                # immediately instead of making the operator sit through a
                # validation pass that cannot help.
                verdict = self._verdict(rms_u, rms_v, gx, gy)
                if clamped_u or clamped_v:
                    self.phase = 'done'
                    self.timer.stop()
                    self.anim.stop()
                    self.u = self.v = None
                    self.lbl.setText('Calibration failed')
                    self.result.setText(verdict)
                    self.result.show()
                    self.hint.setText(
                        f'gain=({gx:.2f}, {gy:.2f})   '
                        f'fit error {rms_u * 1920:.0f} px across, '
                        f'{rms_v * 1080:.0f} px down')
                    self.status.setText('Press Apply to use it anyway, or '
                                        'Cancel and fix the signal.')
                    self._enable_apply()
                    return

                # Otherwise measure it for real. The fit's own error is how well
                # it reproduces the dots it was trained on, which is not the
                # same as being accurate - and "accurate" is the only question
                # the operator actually asks.
                self.lbl.setText('Measuring accuracy…')
                self.hint.setText('Five more dots — this checks the result')
                self._begin_validation()

            def _enable_apply(self):
                self.btn_go.setText('Apply')
                self.btn_go.setEnabled(True)
                try:
                    self.btn_go.clicked.disconnect()
                except TypeError:
                    pass
                self.btn_go.clicked.connect(self.accept)
                self.update()

            @staticmethod
            def _verdict(rms_u, rms_v, gx, gy):
                """One sentence the operator can act on.

                Ordered by what is worth acting on first: a clamped gain means
                the SIGNAL is wrong and no amount of recalibrating helps, so it
                outranks a merely loose fit.
                """
                if gy >= GAIN_MAX - 1e-9:
                    return ('Vertical signal is too weak - the tracker barely '
                            'moved up and down. Raise the Head slider.')
                if gy <= GAIN_MIN + 1e-9:
                    return ('Vertical signal is wild - tracking noise is '
                            'dominating. Check the lighting, then retry.')
                if gx >= GAIN_MAX - 1e-9:
                    return ('Horizontal signal is too weak. Check the camera '
                            'can see your whole face.')
                if gx <= GAIN_MIN + 1e-9:
                    return ('Horizontal signal is wild - noise is dominating. '
                            'Check the lighting, then retry.')
                if rms_v > 0.06:
                    return 'Vertical fit is loose. Try again, moving your head.'
                if rms_u > 0.06:
                    return 'Horizontal fit is loose. Try again, looking further.'
                if rms_u < 0.02 and rms_v < 0.02:
                    return 'Excellent fit. Press Apply.'
                return 'Good fit. Press Apply.'

            def reject(self):
                self.timer.stop()
                self.anim.stop()
                super().reject()

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
    DrawGazeOverlay = make_gaze_overlay(QtCore, QtGui)
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
        # Camera enumeration runs on a worker thread because opening a camera
        # can block for a second or more; the result comes back here so the
        # widget is only ever touched from the GUI thread.
        cams_signal = QtCore.pyqtSignal(object)

        def __init__(self):
            super().__init__()
            self.setWindowTitle('ROSMASTER M1 — Operator Console')
            self.resize(1340, 900)
            self.setMinimumSize(1040, 680)
            self.setStyleSheet(STYLE)
            self.held = set()
            self.audio_on = False
            self._last_jpeg = None
            self._last_face = None
            self._map_sig = None
            self._link_busy = False
            self._cam_index = None

            # Connections must exist before anything can emit.
            self.log_signal.connect(self._append_log)
            self.link_signal.connect(self._apply_link)
            self.recheck_signal.connect(self._check_link)
            self.view_signal.connect(self._apply_view_state)
            self.cams_signal.connect(self._apply_cameras)

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

            self.gaze_timer = QtCore.QTimer(self)
            self.gaze_timer.timeout.connect(self._poll_gaze)
            self.gaze_timer.start(4000)
            self._poll_gaze()

            # Populate the camera picker once at startup, and make the picker
            # reflect the current source. Doing it here rather than in the
            # panel builder keeps the window responsive: the scan runs on a
            # worker thread and the result arrives on cams_signal.
            self._on_gaze_src(0)
            QtCore.QTimer.singleShot(300, self._refresh_cameras)

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
                b.clicked.connect(lambda _c, m=mode: self._choose_mode(m))
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

            # Face + plot side by side. The plot alone says "the dot is here";
            # it cannot say whether the webcam can see a face at all, and those
            # two failures look identical on a bare dot.
            top = QtWidgets.QHBoxLayout()
            top.setSpacing(8)
            # The face gets most of the row. It answers a question the plot
            # cannot - can the camera see me at all - and a 72 px thumbnail is
            # too small to tell whether the face is lit, centred, or half out
            # of frame, which are the things that actually go wrong.
            self.lbl_face = QtWidgets.QLabel('no\nface')
            self.lbl_face.setAlignment(QtCore.Qt.AlignCenter)
            self.lbl_face.setMinimumSize(120, 120)
            self.lbl_face.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                        QtWidgets.QSizePolicy.Expanding)
            self.lbl_face.setStyleSheet(
                'background:#0d1014; border:1px solid #2b3138;'
                'border-radius:5px; color:#6b7280; font-size:10px;')
            top.addWidget(self.lbl_face, 1)
            self.gaze_map = GazeMap()
            top.addWidget(self.gaze_map, 0, QtCore.Qt.AlignTop)
            gv.addLayout(top)

            self.lbl_gaze = QtWidgets.QLabel('no source')
            self.lbl_gaze.setObjectName('mono')
            self.lbl_gaze.setAlignment(QtCore.Qt.AlignCenter)
            gv.addWidget(self.lbl_gaze)

            self.lbl_zone = QtWidgets.QLabel('—')
            self.lbl_zone.setObjectName('mono')
            self.lbl_zone.setAlignment(QtCore.Qt.AlignCenter)
            gv.addWidget(self.lbl_zone)

            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            self.cmb_gaze_src = QtWidgets.QComboBox()
            self.cmb_gaze_src.addItems(['Mediapipe', 'Mouse', 'External'])
            self.cmb_gaze_src.setToolTip(
                'Mediapipe  iris tracking from the local webcam (real eye tracking)\n'
                'Mouse      the HUD window follows the mouse pointer\n'
                'External   wait for /gaze/point from another program')
            self.cmb_gaze_src.setFixedWidth(96)
            self.cmb_gaze_src.currentIndexChanged.connect(self._on_gaze_src)
            row.addWidget(self.cmb_gaze_src)
            self.btn_gaze = QtWidgets.QPushButton('Start gaze')
            self.btn_gaze.setObjectName('primary')
            self.btn_gaze.setToolTip(
                'Start the gaze HUD on this machine. Opens a window showing the\n'
                'robot camera with your gaze point overlaid.')
            self.btn_gaze.clicked.connect(self._toggle_gaze)
            row.addWidget(self.btn_gaze, 1)
            self.btn_gaze_stop = QtWidgets.QPushButton('Stop')
            self.btn_gaze_stop.setEnabled(False)
            self.btn_gaze_stop.clicked.connect(self._stop_gaze)
            row.addWidget(self.btn_gaze_stop)
            gv.addLayout(row)

            # Camera picker. Needed because /dev/video0 is not stable: unplug
            # the webcam or reset the host USB and it comes back as
            # /dev/video1, which the node then cannot find. Rather than make
            # that a command-line problem, the available cameras are listed
            # here and the chosen one is passed to the HUD at start.
            crow = QtWidgets.QHBoxLayout()
            crow.setSpacing(4)
            lbl_cam = QtWidgets.QLabel('Camera')
            lbl_cam.setObjectName('dim')
            crow.addWidget(lbl_cam)
            self.cmb_cam = QtWidgets.QComboBox()
            self.cmb_cam.setToolTip(
                'Which webcam the eye tracker uses. Only cameras that can '
                'actually deliver a frame are listed.\n'
                'Press Refresh after plugging one in.')
            crow.addWidget(self.cmb_cam, 1)
            self.btn_cam_refresh = QtWidgets.QPushButton('↻')
            self.btn_cam_refresh.setFixedWidth(28)
            self.btn_cam_refresh.setToolTip('Rescan for cameras')
            self.btn_cam_refresh.clicked.connect(self._refresh_cameras)
            crow.addWidget(self.btn_cam_refresh)
            gv.addLayout(crow)

            row = QtWidgets.QHBoxLayout()
            self.btn_calib = QtWidgets.QPushButton('Calibrate…')
            self.btn_calib.setObjectName('primary')
            self.btn_calib.clicked.connect(self._calibrate)
            row.addWidget(self.btn_calib)
            self.btn_calib_reset = QtWidgets.QPushButton('Reset')
            self.btn_calib_reset.clicked.connect(self._reset_calibration)
            row.addWidget(self.btn_calib_reset)
            gv.addLayout(row)

            # Its own row, full width: this is the control that makes gaze mode
            # readable at all, so it must not be a small button squeezed in
            # beside two others. Turns itself on when a gaze mode is selected.
            self.btn_overlay = QtWidgets.QPushButton('Overlay')
            self.btn_overlay.setObjectName('toggle')
            self.btn_overlay.setCheckable(True)
            self.btn_overlay.setToolTip(
                'Draw the gaze deadzone, zones and aim point on the camera '
                'image. On by default in gaze mode: without it you cannot '
                'tell where the computer thinks you are looking.')
            self.btn_overlay.toggled.connect(self._toggle_overlay)
            gv.addWidget(self.btn_overlay)

            self.lbl_calib = QtWidgets.QLabel('offset —  gain —')
            self.lbl_calib.setObjectName('mono')
            self.lbl_calib.setAlignment(QtCore.Qt.AlignCenter)
            gv.addWidget(self.lbl_calib)

            # -- deadzone, adjustable while driving ------------------------
            # This is the one number that has to be tuned by feel, so it is a
            # slider rather than a config file. A deadzone that is too small
            # drives on tracker noise; too large and nothing is reachable.
            # Horizontal and vertical are separate because the eye travels
            # roughly 2.5x further sideways than up and down.
            self.sld_dzx = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.sld_dzx.setRange(4, 48)          # percent of width, radius
            self.sld_dzx.setToolTip(
                'Horizontal deadzone, as a percentage of the screen width '
                'either side of centre. Bigger = you must look further before '
                'it turns.')
            gv.addWidget(self._labelled('Deadzone  ↔', self.sld_dzx))

            self.sld_dzy = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.sld_dzy.setRange(4, 48)          # percent of height, radius
            self.sld_dzy.setToolTip(
                'Vertical deadzone, as a percentage of the screen height. Keep '
                'this SMALLER than the horizontal one: the eye cannot travel '
                'as far up and down, so an equal setting makes forward and '
                'backward almost unreachable.')
            gv.addWidget(self._labelled('Deadzone  ↕', self.sld_dzy))

            # The head/eye balance. This is the knob that decides whether
            # up-and-down works at all: at 0 the signal is iris-only, and the
            # iris barely moves vertically inside the eye, so the vertical axis
            # becomes noise. At 1 the eyes are ignored entirely.
            self.sld_head = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.sld_head.setRange(0, 100)
            self.sld_head.setToolTip(
                'How much of the gaze comes from your HEAD rather than your '
                'eyes.\n'
                'Raise it if you have to strain your eyes to reach a zone.\n'
                'At 0 the vertical axis stops working.')
            gv.addWidget(self._labelled('Head  ↔↕', self.sld_head))

            # Set the starting positions BEFORE connecting: setValue() emits
            # valueChanged, and the handler reads ALL of these sliders, so
            # connecting first would fire it while the later ones do not exist
            # yet. That bug made the console fail to start.
            self.sld_dzx.setValue(int(round(node.deadzone_ratio * 100)))
            self.sld_dzy.setValue(int(round(node.deadzone_ratio_y * 100)))
            self.sld_head.setValue(int(round(node.head_weight * 100)))
            self.sld_dzx.valueChanged.connect(self._deadzone_changed)
            self.sld_dzy.valueChanged.connect(self._deadzone_changed)
            self.sld_head.valueChanged.connect(self._deadzone_changed)

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

        def _labelled(self, text, widget):
            """A slider with its name to the left, in one row."""
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            lbl = QtWidgets.QLabel(text)
            lbl.setObjectName('dim')
            lbl.setFixedWidth(84)
            row.addWidget(lbl)
            row.addWidget(widget, 1)
            box = QtWidgets.QWidget()
            box.setLayout(row)
            box.setFixedHeight(24)
            return box

        def _deadzone_changed(self, _v):
            node.set_deadzone(self.sld_dzx.value() / 100.0,
                              self.sld_dzy.value() / 100.0)
            node.set_head_weight(self.sld_head.value() / 100.0)
            # Redraw immediately: the overlay is the only feedback that tells
            # the operator what the new deadzone actually looks like.
            self._last_jpeg = None

        def _choose_mode(self, mode):
            node.set_mode(mode)
            # Gaze modes: show the overlay without being asked. The whole point
            # of gaze control is knowing where the computer thinks you are
            # looking, and that is unreadable unless it is drawn.
            if mode in (2, 4) and not self.btn_overlay.isChecked():
                self.btn_overlay.setChecked(True)

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

        # --------------------------------------------------------------- gaze
        def _gaze_pids(self):
            try:
                r = subprocess.run(['pgrep', '-f', 'gaze_hud_node'],
                                   capture_output=True, text=True, timeout=3)
                return [int(p) for p in r.stdout.split()]
            except Exception:  # noqa: BLE001
                return []

        def _on_gaze_src(self, _i):
            """Only eye tracking uses a camera, so only then is it relevant."""
            is_eye = self.cmb_gaze_src.currentText() == 'Mediapipe'
            self.cmb_cam.setEnabled(is_eye)
            self.btn_cam_refresh.setEnabled(is_eye)
            self.cmb_cam.setToolTip(
                'Which webcam the eye tracker uses.' if is_eye else
                'Only used by Mediapipe eye tracking.')

        def _refresh_cameras(self):
            """Rescan and repopulate the picker.

            Runs the probe off the GUI thread: opening a camera can block for
            a second or more, and doing that inside a button handler makes the
            whole window appear to freeze.
            """
            self.btn_cam_refresh.setEnabled(False)
            self.cmb_cam.clear()
            self.cmb_cam.addItem('scanning…', None)

            def work():
                try:
                    cams = list_cameras()
                except Exception as exc:  # noqa: BLE001
                    cams = []
                    self.log_signal.emit(f'camera scan failed: {exc}')
                self.cams_signal.emit(cams)

            threading.Thread(target=work, daemon=True).start()

        def _apply_cameras(self, cams):
            self.cmb_cam.clear()
            if not cams:
                # Say it here rather than at start time. "No camera" discovered
                # at start looks like the tracker failed; discovered here it is
                # obviously a camera problem.
                self.cmb_cam.addItem('no camera found', None)
                self.btn_cam_refresh.setEnabled(True)
                self.logline('no camera found - plug one in and press ↻')
                return
            for idx, label in cams:
                self.cmb_cam.addItem(label, idx)
            self.btn_cam_refresh.setEnabled(True)
            # Keep the previous choice if it is still present, so a refresh does
            # not silently switch cameras on the operator.
            want = getattr(self, '_cam_index', None)
            if want is not None:
                for i in range(self.cmb_cam.count()):
                    if self.cmb_cam.itemData(i) == want:
                        self.cmb_cam.setCurrentIndex(i)
                        break
            self._cam_index = self.cmb_cam.currentData()
            self.logline(f'{len(cams)} camera(s): '
                         + ', '.join(l for _i, l in cams))

        def _selected_camera(self):
            """The index to hand the HUD, or None."""
            idx = self.cmb_cam.currentData()
            self._cam_index = idx
            return idx

        def _toggle_gaze(self):
            if self._gaze_pids():
                self.logline('gaze already running')
                return
            wrapper = os.path.expanduser('~/m1_gaze.sh')
            if not os.path.exists(wrapper):
                self.logline(f'missing {wrapper} - see docs/GAZE.md')
                return
            src = self.cmb_gaze_src.currentText().lower()
            args = ['bash', wrapper, src]
            if src == 'mediapipe':
                cam = self._selected_camera()
                if cam is None:
                    # Refuse rather than start something that cannot work.
                    # Starting with no camera falls back to mouse gaze, which
                    # looks like tracking and is not - that confusion cost
                    # several rounds of debugging.
                    self.logline('no camera selected - press ↻ and pick one, '
                                 'or choose Mouse to drive without eye tracking')
                    return
                # The wrapper passes everything after the source through to
                # `ros2 run ... --ros-args`, so this reaches the node.
                args += ['-p', f'webcam_id:={int(cam)}']
                self.logline(f'using camera index {int(cam)}')
            try:
                subprocess.Popen(
                    args,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True)
                self.logline(
                    f'gaze started ({self.cmb_gaze_src.currentText()}) - '
                    f'the HUD window shows the robot camera with your gaze point')
                QtCore.QTimer.singleShot(8000, self._poll_gaze)
            except Exception as exc:  # noqa: BLE001
                self.logline(f'gaze failed to start: {exc}')

        def _stop_gaze(self):
            pids = self._gaze_pids()
            if not pids:
                self.logline('gaze not running')
                return
            for pid in pids:
                try:
                    os.kill(pid, 15)
                except Exception:  # noqa: BLE001
                    pass
            self.logline('gaze stopped')
            QtCore.QTimer.singleShot(1500, self._poll_gaze)

        def _toggle_overlay(self, on):
            self._style_toggle(self.btn_overlay, on=on,
                               on_text='Overlay on', off_text='Overlay',
                               off_kind='grey')
            # Force the next frame to be redrawn: the picture is only rebuilt
            # when a new JPEG arrives, so without this the overlay would not
            # appear (or disappear) until the video happened to update.
            self._last_jpeg = None

        def _overlay_camera(self, pix, s):
            """Draw the gaze HUD onto the robot camera image.

            The drawing itself lives in make_gaze_overlay() so it can be
            exercised without a display. This only supplies the live values.
            """
            return DrawGazeOverlay(pix, s['zone'], node.latest_raw(),
                                   s['calib'], node.deadzone_ratio,
                                   node.deadzone_ratio_y)

        def _poll_gaze(self):
            running = bool(self._gaze_pids())
            self.btn_gaze.setEnabled(not running)
            self.btn_gaze_stop.setEnabled(running)
            self.btn_gaze.setText('Gaze running' if running else 'Start gaze')

        def _kill_gaze_now(self):
            """Stop the gaze HUD synchronously, without waiting for it.

            The HUD is started detached (start_new_session) so a crash here
            cannot take it down mid-drive. That is right for a crash and wrong
            for a clean exit: the process outlives the console, keeps the
            webcam open, and leaves an OpenCV window on screen with nobody
            driving it. Closing the console has to close it too.

            SIGTERM, not SIGKILL: the node's shutdown path releases the V4L2
            capture, and killing it outright can leave /dev/video0 held until
            the kernel notices.
            """
            pids = self._gaze_pids()
            for pid in pids:
                try:
                    os.kill(pid, 15)
                except ProcessLookupError:
                    pass          # already gone, which is the goal
                except Exception:  # noqa: BLE001 - never block closing
                    pass
            return pids

        # ---------------------------------------------------------- calibration
        def _calibrate(self):
            # The WHOLE window, not the camera picture. Calibrating inside the
            # few hundred pixels of the video panel asks for eye movements so
            # small that the fit has almost no signal; spread across the screen
            # the same movement is many times larger and the gain means
            # something. See CalibDialog's docstring.
            area = QtCore.QRect(self.mapToGlobal(QtCore.QPoint(0, 0)),
                                self.size())
            dlg = CalibDialogCls(node, area, self)
            if dlg.exec_() == QtWidgets.QDialog.Accepted and dlg.result_calib:
                ox, oy, gx, gy = dlg.result_calib
                node.set_calibration(ox, oy, gx, gy)
                self.logline(
                    f'calibration applied  offset=({ox:+.3f},{oy:+.3f}) '
                    f'gain=({gx:.2f},{gy:.2f})')
                # Put the fit quality in the log too, not just in the dialog:
                # the dialog is gone by the time the operator is actually
                # driving, and "is this calibration any good" is a question
                # asked later, not at Apply.
                q = dlg.quality
                if q:
                    self.logline(
                        f'  fit error {q["rms_u"] * 1920:.0f} px across, '
                        f'{q["rms_v"] * 1080:.0f} px down over {q["dots"]} dots '
                        f'(worst: {q["worst"]})')
                    if q['clamped_v'] or q['clamped_u']:
                        axis = 'vertical' if q['clamped_v'] else 'horizontal'
                        self.logline(
                            f'  WARNING: {axis} gain hit a limit - the tracker '
                            f'and the dots do not agree on that axis')
                # And the MEASURED accuracy, which is the number that matters.
                acc = dlg.accuracy
                if acc:
                    parts = '  '.join(f'{k} {v:.0f}' for k, v in acc.items())
                    self.logline(f'  measured error (px):  {parts}')
                    if dlg.accuracy_worst and dlg.accuracy_worst > 110:
                        self.logline(
                            f'  weakest direction is off by '
                            f'{dlg.accuracy_worst:.0f} px')

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
                # Through _choose_mode, not node.set_mode: selecting a gaze
                # mode must also turn the overlay on, or the operator gets a
                # mode they cannot see. Doing it in two places is how the two
                # paths drifted apart before.
                self._choose_mode(MODE_KEYS[text])
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
                    scaled = pix.scaled(
                        self.lbl_cam.size(), QtCore.Qt.KeepAspectRatio,
                        QtCore.Qt.SmoothTransformation)
                    if self.btn_overlay.isChecked():
                        scaled = self._overlay_camera(scaled, s)
                    self.lbl_cam.setPixmap(scaled)
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

            # face thumbnail
            face = node.latest_face()
            if face is None:
                self.lbl_face.setText('no\nface')
            else:
                fdata, fage = face
                if fage > 2.0:
                    # Drop a stale face rather than showing the last one
                    # forever: a frozen picture of a face reads as "tracking
                    # is fine" when in fact nothing is arriving.
                    self.lbl_face.setText('no\nface')
                elif fdata is not self._last_face:
                    self._last_face = fdata
                    fpix = QtGui.QPixmap()
                    if fpix.loadFromData(fdata, 'JPEG'):
                        self.lbl_face.setPixmap(fpix.scaled(
                            self.lbl_face.size(), QtCore.Qt.KeepAspectRatio,
                            QtCore.Qt.SmoothTransformation))

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

            # The gaze HUD was started detached and does not die with this
            # process, so it has to be stopped explicitly. Unconditional: it
            # is a window this console opened, and it holds the webcam.
            killed = self._kill_gaze_now()
            if killed:
                self.logline(f'stopped gaze ({len(killed)} process(es))')

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
