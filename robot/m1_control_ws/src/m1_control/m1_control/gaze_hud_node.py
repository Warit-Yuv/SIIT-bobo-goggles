#!/usr/bin/env python3
"""Dual-action gaze controller with a live HUD.

Runs on the OPERATOR PC (it needs the operator-facing webcam and a display).
It shows the robot's video feed with a HUD overlay and splits the screen into
two behaviours:

  * Inside the deadzone circle (centre of the frame, radius = 25% of the short
    side): the chassis stays still and gaze offsets drive small proportional
    PTZ pan/tilt steps. "Look around without driving."
  * Outside the deadzone: the frame is divided into quadrants. Gaze must stay
    in one quadrant for `dwell_time_sec` before a chassis command is issued,
    which rejects the natural saccades that would otherwise cause false drives.

Outputs (never /cmd_vel directly - the arbiter owns that):

    /cmd_vel/gaze   geometry_msgs/Twist    vx, vy, wz
    /servo/gaze     geometry_msgs/Vector3  x = pan s1, y = tilt s2
    /control_mode   std_msgs/UInt8         asserted to 2 while running

Gaze sources (`gaze_source` parameter)
--------------------------------------
  mediapipe  iris tracking from the local webcam (needs `mediapipe`)
  mouse      click on the HUD window to place the gaze point (no hardware)
  external   subscribe to /gaze/point (geometry_msgs/Point, values 0..1)
             so an external model can publish its own estimate
  keyboard   the HUD is display-only; gaze is driven by /gaze/point only

Run on the PC:
    ros2 run m1_control gaze_hud_node
    ros2 run m1_control gaze_hud_node --ros-args -p gaze_source:=mouse
"""

import json
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist, Vector3
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8

try:
    from m1_control.gaze_calib import apply as apply_calib
except ImportError:  # run as a plain script rather than through ros2 run
    from gaze_calib import apply as apply_calib

MODE_GAZE = 2

#: Canonical 3D face model, in millimetres, for cv2.solvePnP.
#: Standard FaceMesh correspondences. Only the ROTATION is used, so the scale
#: of these numbers does not matter - which is fortunate, because it is a
#: generic face rather than this operator's.
_PNP_MODEL = np.array([
    [0.0, 0.0, 0.0],          # 1    nose tip
    [0.0, -63.6, -12.5],      # 199  chin
    [-43.3, 32.7, -26.0],     # 33   left eye, outer corner
    [43.3, 32.7, -26.0],      # 263  right eye, outer corner
    [-28.9, -28.9, -24.1],    # 61   left mouth corner
    [28.9, -28.9, -24.1],     # 291  right mouth corner
], dtype=np.float64)
_PNP_IDS = (1, 199, 33, 263, 61, 291)

#: Radians of head rotation that map to full deflection. Beyond roughly 20 deg
#: people stop turning their head and move their eyes instead, so this is about
#: where the head term should stop contributing.
HEAD_YAW_SPAN = math.radians(22.0)
HEAD_PITCH_SPAN = math.radians(16.0)

#: One place for the window title, because it is also the key used to ask
#: OpenCV whether the window still exists.
WINDOW_TITLE = 'M1 Gaze Teleoperation HUD'

# Quadrant labels
ZONE_CENTER = 'CENTER'
ZONE_FORWARD = 'FORWARD'
ZONE_BACKWARD = 'BACKWARD'
ZONE_TURN_LEFT = 'TURN_LEFT'
ZONE_TURN_RIGHT = 'TURN_RIGHT'


class GazeHUDNode(Node):

    def __init__(self):
        super().__init__('gaze_hud_node')

        # ---- parameters ----
        self.declare_parameter('camera_topic', '/image_throttled/compressed')
        self.declare_parameter('gaze_source', 'mediapipe')
        self.declare_parameter('webcam_id', 0)
        self.declare_parameter('dwell_time_sec', 0.40)
        # Deadzone radii, as a FRACTION OF THE CONTROL SPACE, not of a camera
        # frame. 0.30 means "you may look 30% of the width either side of
        # centre before the robot moves". Both are radii, so the oval spans
        # 2x these numbers across the whole space.
        #
        # The previous code multiplied by an extra 0.5, which silently halved
        # both radii: a deadzone that should have covered 50% of the width
        # covered 25%. That is why it felt far too small to be useful.
        self.declare_parameter('deadzone_radius_ratio', 0.30)
        # Vertical is deliberately much smaller than horizontal. The eye
        # travels roughly 2.5x further sideways than up and down, so an equal
        # radius makes FORWARD and BACKWARD unreachable while turning stays
        # easy - which is exactly the complaint that drove this change.
        self.declare_parameter('deadzone_radius_ratio_y', 0.16)

        self.declare_parameter('drive_speed_x', 0.20)
        self.declare_parameter('drive_speed_y', 0.20)
        self.declare_parameter('turn_speed_z', 0.60)

        # PTZ aiming is POSITIONAL, not integrating - see _hud_tick(). The span
        # is the travel used across the whole deadzone: looking at its edge aims
        # half a span from centre and no further, and looking back returns the
        # head to centre.
        self.declare_parameter('servo_pan_span', 10.0)
        self.declare_parameter('servo_tilt_span', 20.0)
        # Low-pass per tick. 1.0 = instant (twitchy), 0.2 = very smooth.
        self.declare_parameter('servo_smoothing', 0.25)
        self.declare_parameter('servo_rate_limit_hz', 12.0)

        # How far the iris signal is stretched before calibration. Calibration
        # learns the real scale, so this only matters for the uncalibrated case
        # and for keeping the raw value inside 0..1.
        self.declare_parameter('iris_gain_h', 3.0)
        self.declare_parameter('iris_gain_v', 3.0)
        # How much of the gaze signal comes from the HEAD rather than the eye.
        #
        # This is not a refinement, it is what makes vertical work at all.
        # The iris barely moves vertically inside the eye - a few percent of
        # the eye's width - so an iris-only vertical signal is mostly noise.
        # Measured on the hardware, the fitted vertical gain came back clamped
        # at its 0.25 minimum, which is the fit saying "there is no usable
        # relationship here".
        #
        # Head pitch, by contrast, is large and clean: to look at the top of a
        # screen people tilt their head up by 10-20 degrees. Adding it means
        # the vertical signal is dominated by something that actually moves.
        # The same is true of yaw horizontally, which is why looking at the far
        # edges of the screen no longer requires straining the eyes.
        self.declare_parameter('head_weight', 0.75)
        # Low-pass on the raw gaze, 0..1. Tracker noise is what makes a zone
        # flicker on and off, and a flickering zone reads as "it does not
        # work" even when the logic is correct. Higher = steadier but laggier.
        self.declare_parameter('gaze_smoothing', 0.55)
        # Publish the operator's face so the console can show it. Small and
        # slow on purpose: it travels over the same Wi-Fi as the video feed.
        self.declare_parameter('publish_face', True)
        self.declare_parameter('face_publish_hz', 5.0)
        self.declare_parameter('face_size', 112)

        self.declare_parameter('servo_s1_center', 40.0)
        self.declare_parameter('servo_s2_center', 90.0)
        self.declare_parameter('servo_s1_min', 5.0)
        self.declare_parameter('servo_s1_max', 75.0)
        self.declare_parameter('servo_s2_min', 80.0)
        self.declare_parameter('servo_s2_max', 100.0)

        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('show_window', True)
        # Mode is the OPERATOR's decision, not this node's. Default off.
        # With this on, the HUD re-asserted GAZE every second, which made
        # keyboard control impossible while it was running: the arbiter was
        # flipped back to GAZE within ~16 ms of every KEYBOARD command.
        self.declare_parameter('assert_mode', False)

        # Per-axis correction, same four numbers and same formula as
        # gaze_udp_bridge:  corrected = (raw - offset) * gain + 0.5
        self.declare_parameter('gaze_offset_x', 0.5)
        self.declare_parameter('gaze_offset_y', 0.5)
        self.declare_parameter('gaze_gain_x', 1.0)
        self.declare_parameter('gaze_gain_y', 1.0)
        # Where the console writes the fitted model. Shared machine, so a file
        # beats trying to push weights across a topic.
        self.declare_parameter(
            'model_path', os.path.expanduser('~/gaze_calibration.json'))
        # The four quick profiles fit an offset and a gain rather than a
        # model, so they are saved separately.
        self.declare_parameter(
            'legacy_path', os.path.expanduser('~/gaze_legacy_calib.json'))

        gp = self.get_parameter
        self.camera_topic = gp('camera_topic').value
        self.gaze_source = str(gp('gaze_source').value).lower()
        self.dwell_threshold = float(gp('dwell_time_sec').value)
        self.deadzone_ratio = float(gp('deadzone_radius_ratio').value)
        self.deadzone_ratio_y = float(gp('deadzone_radius_ratio_y').value)
        self.drive_vx = float(gp('drive_speed_x').value)
        self.drive_vy = float(gp('drive_speed_y').value)
        self.turn_wz = float(gp('turn_speed_z').value)
        self.pan_span = float(gp('servo_pan_span').value)
        self.tilt_span = float(gp('servo_tilt_span').value)
        self.servo_smoothing = float(np.clip(gp('servo_smoothing').value,
                                             0.01, 1.0))
        self.iris_gain_h = float(gp('iris_gain_h').value)
        self.iris_gain_v = float(gp('iris_gain_v').value)
        self.head_weight = float(np.clip(gp('head_weight').value, 0.0, 1.0))
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.gaze_smoothing = float(np.clip(gp('gaze_smoothing').value,
                                            0.0, 0.95))
        self.publish_face = bool(gp('publish_face').value)
        self.face_period = 1.0 / max(0.5, float(gp('face_publish_hz').value))
        self.face_size = int(gp('face_size').value)
        self.show_window = bool(gp('show_window').value)
        self.assert_mode = bool(gp('assert_mode').value)

        # NAMED AS THE HARDWARE NAMES THEM.
        #
        #   s1 = TILT  centre 40, range 5..75    larger = look UP
        #   s2 = PAN   centre 90, range 80..100  larger = look LEFT
        #
        # The arbiter reads /servo/gaze as s1 = msg.x, s2 = msg.y. This node
        # used to hold pan in .x and clip pan to the s1 range, which clamped
        # pan to 5..75 (its real range is 80..100) and tilt to 80..100 (its
        # real range is 5..75). Both axes were therefore pinned against the
        # wrong limits, which is what made the head slam to a stop and look
        # jerky rather than following the eyes.
        self.pan_center = float(gp('servo_s2_center').value)
        self.tilt_center = float(gp('servo_s1_center').value)
        self.pan = self.pan_center
        self.tilt = self.tilt_center
        self.s1_min, self.s1_max = float(gp('servo_s1_min').value), float(gp('servo_s1_max').value)
        self.s2_min, self.s2_max = float(gp('servo_s2_min').value), float(gp('servo_s2_max').value)
        self.invert_pan = bool(gp('invert_pan').value)
        self.invert_tilt = bool(gp('invert_tilt').value)

        # Held in memory rather than re-read from the parameters: the console
        # sets it over /gaze/calib, and gaze_udp_bridge already learned that
        # set_parameters() from inside a callback silently fails to stick.
        self._calib = [float(gp('gaze_offset_x').value),
                       float(gp('gaze_offset_y').value),
                       float(gp('gaze_gain_x').value),
                       float(gp('gaze_gain_y').value)]
        # ...and then immediately replaced by whatever was saved last session.
        # The four quick profiles (fast/long/super/ultra) only ever lived in
        # this list, so restarting the HUD silently threw away a calibration
        # the operator had just spent minutes producing - and the only symptom
        # was that the gaze went back to being wrong.
        self._load_legacy_calib()

        servo_period = 1.0 / max(1.0, float(gp('servo_rate_limit_hz').value))
        self._last_servo = 0.0
        self._servo_period = servo_period

        # ---- state ----
        self.battery = float('nan')
        self.fps = 0.0
        self._fps_t = time.time()
        self._fps_n = 0
        self.frame = None

        self.gaze_u = 0.5
        self.gaze_v = 0.5
        self.gaze_valid = False
        # Smoothed copies, which is what the rest of the node uses. Raw is
        # kept for /gaze/raw so calibration is not biased by the filter.
        self.smooth_u = 0.5
        self.smooth_v = 0.5
        self._hist_u = deque(maxlen=5)
        self._hist_v = deque(maxlen=5)
        self._was_valid = False
        self.mouse_click = None
        self._window_ready = False
        self._saw_visible = False
        self._WIN = WINDOW_TITLE
        self._last_face = 0.0

        self.zone = ZONE_CENTER
        self.zone_since = time.time()
        self.confirmed_zone = ZONE_CENTER

        # ---- interfaces ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel/gaze', 10)
        self.servo_pub = self.create_publisher(Vector3, '/servo/gaze', 10)
        self.mode_pub = self.create_publisher(UInt8, '/control_mode', mode_qos)

        # Publish the uncorrected gaze point so the operator console can
        # calibrate against it.
        #
        # This was the gap that made "Calibrate" report "no gaze source" even
        # with a perfectly working webcam: /gaze/raw only ever came from
        # gaze_udp_bridge, so the local-webcam path (mediapipe or mouse) had no
        # stream to calibrate from. The console cannot care which source
        # produced a gaze point - it needs the same topic either way.
        #
        # z carries validity: 1.0 = a real fix, 0.0 = face lost / blink.
        self.raw_pub = self.create_publisher(Point, '/gaze/raw', 10)

        # The CORRECTED point, which is what should steer anything.
        #
        # This was missing, and it is why the console's Calibrate button did
        # nothing at all: the console published /gaze/calib, but on the
        # mediapipe path no node subscribed to it. Only gaze_udp_bridge did,
        # and that is the *other* gaze source. The numbers went nowhere.
        #
        # Publishing the corrected point here also lets the Pi-side
        # gaze_control_node work with the local HUD, which it previously
        # could not: the HUD drove /cmd_vel/gaze itself and never emitted
        # /gaze/point.
        self.point_pub = self.create_publisher(Point, '/gaze/point', 10)
        self.calib_state_pub = self.create_publisher(
            Float32MultiArray, '/gaze/calib_state', 10)
        # The console needs the zone to draw its overlay, and it should not
        # have to reimplement the dwell state machine to work it out.
        self.zone_pub = self.create_publisher(String, '/gaze/zone', 10)
        self.face_pub = self.create_publisher(
            CompressedImage, '/gaze/face', sensor_qos)
        # The deadzone and the head weight are the two numbers the operator has
        # to tune by feel, so they are adjustable while driving instead of
        # needing a relaunch. The console sends them here; the HUD echoes them
        # back on the _state topic so the console's sliders show what is
        # actually in force after clamping.
        #
        # Payload: [deadzone_x, deadzone_y, head_weight]
        self.tuning_state_pub = self.create_publisher(
            Float32MultiArray, '/gaze/tuning_state', 10)
        self.create_subscription(Float32MultiArray, '/gaze/tuning',
                                 self._tuning_cb, 10)
        self.create_timer(1.0, self._report_tuning)

        self.create_subscription(CompressedImage, self.camera_topic,
                                 self.image_cb, sensor_qos)
        self.create_subscription(Float32, '/voltage', self.voltage_cb, sensor_qos)
        self.create_subscription(Point, '/gaze/point', self.external_gaze_cb, 10)
        self.create_subscription(Float32MultiArray, '/gaze/calib',
                                 self._calib_cb, 10)
        self.create_timer(1.0, self._report_calib)
        # The console fits the model and writes it; pick it up without needing
        # a restart of this node.
        self.create_timer(1.0, self._reload_model)
        # Raw features, so the console can collect calibration samples. Ten
        # numbers per frame is a few hundred bytes - nothing next to video.
        self.feature_pub = self.create_publisher(
            Float32MultiArray, '/gaze/features', 10)

        self._tracker = None
        self._model = None
        self._model_ready = False
        self._model_mtime = 0.0
        self.features = None
        self._cap = None
        self._cam_failures = 0
        self._face_seen_at = 0.0
        self._no_face_since = 0.0
        self._warned_no_face = False
        self._reload_model(force=True)
        if self.gaze_source == 'mediapipe':
            self._init_mediapipe()
        elif self.gaze_source == 'mouse':
            self.get_logger().info('Gaze source = mouse (click the HUD to aim)')
        elif self.gaze_source not in ('external', 'keyboard'):
            self.get_logger().warn(
                f'Unknown gaze_source "{self.gaze_source}" - using external only')

        self.create_timer(1.0, self._assert_mode)
        self.create_timer(1.0 / 30.0, self._hud_tick)

        self.get_logger().info(
            f'Gaze HUD up | source={self.gaze_source} | dwell={self.dwell_threshold}s '
            f'| deadzone={self.deadzone_ratio:.0%} | feed={self.camera_topic}')

    # ------------------------------------------------------------------ setup
    def _find_camera(self):
        """Open the first /dev/videoN that can actually deliver a frame.

        The index is NOT stable. Unplugging the camera, or the host
        re-enumerating USB, moves it from /dev/video0 to /dev/video1 and back.
        A hard-coded index then silently fails to open, the node falls back to
        mouse gaze, and the operator sees "no face" with nothing anywhere
        saying why - which looks exactly like a broken tracker.

        A V4L2 camera also exposes several nodes: the capture node and a
        metadata node that opens fine but never produces a frame. That is why
        this reads a frame instead of trusting isOpened().
        """
        tried = []
        preferred = int(self.get_parameter('webcam_id').value)
        # Preferred index first, then the rest.
        for idx in [preferred] + [i for i in range(10) if i != preferred]:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                tried.append('%d:no-open' % idx)
                continue
            ok, _frame = cap.read()
            if not ok:
                cap.release()
                tried.append('%d:no-frame' % idx)
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info(
                'Camera %d opened (%s)'
                % (idx, 'preferred' if idx == preferred else 'auto-detected'))
            return cap

        import glob
        nodes = sorted(glob.glob('/dev/video*'))
        self.get_logger().error(
            'No usable camera. Tried indices %s. Devices present: %s. '
            'Is the webcam connected and passed through to this machine?'
            % (', '.join(tried) if tried else 'none',
               ', '.join(nodes) if nodes else 'NONE - /dev/video* does not exist'))
        return None

    def _init_mediapipe(self):
        try:
            from m1_control.gaze_tracker import GazeTracker
        except ImportError:
            from gaze_tracker import GazeTracker
        try:
            self._tracker = GazeTracker(log=lambda m: self.get_logger().info(m))
            self._cap = self._find_camera()
            if self._cap is None:
                raise RuntimeError('no camera device could deliver a frame')
            self.get_logger().info(
                'FaceLandmarker tracking ready '
                '(blendshapes + head pose, 10 features)')
        except Exception as exc:  # noqa: BLE001
            # Falling back to mouse is the right behaviour - the operator can
            # still calibrate and drive - but it must be LOUD. A silent
            # fallback looks identical to a broken tracker, and the operator
            # then spends an hour recalibrating a camera that is not there.
            self.get_logger().error(
                f'EYE TRACKING UNAVAILABLE ({exc}). '
                f'Falling back to MOUSE gaze - this is NOT eye tracking. '
                f'Fix the camera before calibrating.')
            self._tracker = None
            self.gaze_source = 'mouse'

    # --------------------------------------------------------------- callbacks
    def voltage_cb(self, msg: Float32):
        self.battery = float(msg.data)

    def _calib_cb(self, msg: Float32MultiArray):
        """A calibration arrived. Persist it, then reload the model file.

        Two things travel on this topic and they need different treatment:

          * the four quick profiles send an offset and a gain, which are just
            four numbers - they are written straight to disk here
          * the Posture profile sends NEUTRAL and relies on the model file,
            which the console already wrote; this callback just triggers the
            reload

        Either way the console and this node run on the same machine, so a
        file is the shared truth. It keeps the weights, the standardisation,
        the ignored-column list and the reported error together, and it
        survives a restart of either side.
        """
        data = list(msg.data)
        if len(data) >= 4 and data != [0.5, 0.5, 1.0, 1.0]:
            # A real offset/gain, not the "use the model" neutral marker.
            self._calib = [float(x) for x in data[:4]]
            self._save_legacy_calib()
        self._reload_model(force=True)

    # ------------------------------------------------- legacy calibration
    def _legacy_path(self):
        try:
            return os.path.expanduser(self.get_parameter('legacy_path').value)
        except Exception:  # noqa: BLE001
            return os.path.expanduser('~/gaze_legacy_calib.json')

    def _save_legacy_calib(self):
        """Write the four quick-profile numbers so they outlive this process."""
        path = self._legacy_path()
        try:
            tmp = path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump({'offset_x': self._calib[0],
                           'offset_y': self._calib[1],
                           'gain_x': self._calib[2],
                           'gain_y': self._calib[3]}, fh, indent=2)
            # Replace in one step: a half-written calibration file is worse
            # than none, because it loads without complaint and steers wrong.
            os.replace(tmp, path)
            self.get_logger().info(f'Calibration saved to {path}')
        except OSError as exc:
            self.get_logger().warn(f'Could not save calibration: {exc}')

    def _load_legacy_calib(self):
        """Adopt a saved offset/gain, if there is one."""
        path = self._legacy_path()
        try:
            with open(path, encoding='utf-8') as fh:
                d = json.load(fh)
            values = [float(d[k]) for k in
                      ('offset_x', 'offset_y', 'gain_x', 'gain_y')]
        except (OSError, KeyError, TypeError, ValueError):
            return          # no file, or a broken one: keep the parameters
        self._calib = values
        self.get_logger().info(
            'Restored calibration  offset=(%+.3f, %+.3f)  gain=(%.2f, %.2f)'
            % tuple(values))

    def _report_calib(self):
        """Echo the calibration that is actually in force.

        This used to publish a hardcoded [0.5, 0.5, 1.0, 1.0] in both branches,
        so the console's status line read "offset +0.50 +0.50 gain 1.00, 1.00"
        no matter what had been calibrated. A status line that cannot report a
        change is worse than no status line: it looks like confirmation.
        """
        self.calib_state_pub.publish(
            Float32MultiArray(data=[float(x) for x in self._calib]))

    def _reload_model(self, force=False):
        """Load the calibration file if it is new or missing."""
        try:
            path = os.path.expanduser(self.get_parameter('model_path').value)
        except Exception:  # noqa: BLE001
            return
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            if self._model_ready:
                self._model_ready = False
                self.get_logger().warn(
                    'Calibration file disappeared - gaze will be uncalibrated')
            return
        if not force and mtime == self._model_mtime:
            return
        try:
            from m1_control.gaze_model import GazeModel
        except ImportError:
            from gaze_model import GazeModel
        model = GazeModel.load(path)
        if not model.is_ready():
            return
        self._model = model
        self._model_mtime = mtime
        self._model_ready = True
        self.get_logger().info(
            f'Calibration loaded: {model.report()}')

    def _tuning_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn(
                f'/gaze/tuning needs [dz_x, dz_y, head_weight], got {len(data)}')
            return
        # Clamped: a deadzone at 0 would drive on tracker noise alone, and one
        # at 0.5 would cover the whole space and never drive at all.
        self.deadzone_ratio = float(np.clip(data[0], 0.02, 0.48))
        self.deadzone_ratio_y = float(np.clip(data[1], 0.02, 0.48))
        if len(data) >= 3:
            self.head_weight = float(np.clip(data[2], 0.0, 1.0))
        self._report_tuning()

    def _report_tuning(self):
        self.tuning_state_pub.publish(
            Float32MultiArray(data=[float(self.deadzone_ratio),
                                    float(self.deadzone_ratio_y),
                                    float(self.head_weight)]))

    def _smooth(self, target_u, target_v):
        """Median-of-5, then a fixed low-pass.

        Tracker noise is not symmetric: most frames are close to the truth and
        the occasional frame is wildly wrong - the iris ring is estimated, and
        a blink, an eyelash or a specular highlight moves it by a large amount
        in one frame and back again in the next. A plain low-pass cannot
        remove that without also making every real eye movement lag.

        A median rejects up to two bad frames out of five no matter HOW wrong
        they are, and it costs a couple of frames of latency, which is nothing
        next to the 0.4 s dwell. The low-pass then takes the remaining
        frame-to-frame wobble off.
        """
        self._hist_u.append(target_u)
        self._hist_v.append(target_v)
        mu = float(np.median(self._hist_u))
        mv = float(np.median(self._hist_v))
        a = 1.0 - self.gaze_smoothing
        self.smooth_u += a * (mu - self.smooth_u)
        self.smooth_v += a * (mv - self.smooth_v)
        return self.smooth_u, self.smooth_v

    def _reset_smooth(self, u, v):
        """Start the filters from a known point after a gap."""
        self._hist_u.clear()
        self._hist_v.clear()
        for _ in range(5):
            self._hist_u.append(u)
            self._hist_v.append(v)
        self.smooth_u, self.smooth_v = float(u), float(v)

    def _correct(self, u, v):
        """Kept for the mouse/external paths, which are already normalised."""
        try:
            from m1_control.gaze_calib import apply as apply_calib
        except ImportError:
            from gaze_calib import apply as apply_calib
        return apply_calib(u, v, self._calib)

    def external_gaze_cb(self, msg: Point):
        if self.gaze_source in ('external', 'keyboard'):
            self.gaze_u = float(np.clip(msg.x, 0.0, 1.0))
            self.gaze_v = float(np.clip(msg.y, 0.0, 1.0))
            self.gaze_valid = True

    def image_cb(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.frame = frame
        self._fps_n += 1

    def _on_mouse(self, event, x, y, flags, param):
        """Mouse gaze: the pointer position IS the gaze, not just a click."""
        if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN,
                     cv2.EVENT_LBUTTONUP, cv2.EVENT_LBUTTONDBLCLK):
            self.mouse_click = (int(x), int(y))

    def _publish_face_crop(self, cam, face):
        """Publish a small crop of the operator's face for the console."""
        now = time.time()
        if now - self._last_face < self.face_period:
            return
        self._last_face = now
        try:
            h, w = cam.shape[:2]
            pts = face.points
            x0, x1 = float(pts[:, 0].min()), float(pts[:, 0].max())
            y0, y1 = float(pts[:, 1].min()), float(pts[:, 1].max())
            pad_x = (x1 - x0) * 0.25
            pad_y = (y1 - y0) * 0.35
            cx0 = int(max(0, x0 - pad_x))
            cx1 = int(min(w, x1 + pad_x))
            cy0 = int(max(0, y0 - pad_y))
            cy1 = int(min(h, y1 + pad_y))
            if cx1 - cx0 < 16 or cy1 - cy0 < 16:
                return
            # Annotate a COPY of the whole frame, then crop. Drawing after
            # cropping would need every landmark shifted into crop space, and
            # a sign error there puts the eye outlines somewhere else on the
            # face - which looks like a tracking bug rather than a drawing one.
            shown = cam.copy()
            try:
                from m1_control.gaze_tracker import draw_face
            except ImportError:
                from gaze_tracker import draw_face
            draw_face(shown, face)
            # A box round the face, so the operator can see what the tracker
            # is actually locked onto rather than guessing from the eyes.
            cv2.rectangle(shown, (int(x0), int(y0)), (int(x1), int(y1)),
                          (60, 160, 255), 1, cv2.LINE_AA)
            crop = shown[cy0:cy1, cx0:cx1]
            side = max(crop.shape[:2])
            square = np.zeros((side, side, 3), np.uint8)
            oy = (side - crop.shape[0]) // 2
            ox = (side - crop.shape[1]) // 2
            square[oy:oy + crop.shape[0], ox:ox + crop.shape[1]] = crop
            small = cv2.resize(square, (self.face_size, self.face_size),
                               interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', small,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                return
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.face_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 - never break tracking for this
            self.get_logger().debug(f'face crop failed: {exc}')

    def _update_gaze_from_webcam(self):
        """One camera frame -> ten features -> a screen point.

        The tracker produces FEATURES; the model turns features into a point.
        Keeping those separate is what allows the model to be replaced (and
        recalibrated) without touching the tracking, and vice versa.
        """
        if self._tracker is None:
            return
        ok, cam = self._cap.read()
        if not ok:
            self.gaze_valid = False
            self._cam_failures += 1
            # A camera can be unplugged, or re-enumerated to a different
            # /dev/videoN, in the middle of a session.
            if self._cam_failures >= 30:            # ~1 s at 30 Hz
                self._cam_failures = 0
                self.get_logger().warn(
                    'Camera stopped delivering frames - searching again')
                try:
                    self._cap.release()
                except Exception:                   # noqa: BLE001
                    pass
                self._cap = self._find_camera()
                if self._cap is None:
                    self.get_logger().error(
                        'Camera is gone. Eye tracking is off until it is '
                        'reconnected.')
            return
        self._cam_failures = 0
        cam = cv2.flip(cam, 1)          # mirror, so the HUD reads like a mirror

        face = self._tracker.process(cam, int(time.time() * 1000))
        self.face = face

        if face is None:
            self.gaze_valid = False
            # Say WHY, once. "Cannot see face" has several causes and the log
            # is the only place that can tell them apart.
            if self._face_seen_at == 0.0:
                if self._no_face_since == 0.0:
                    self._no_face_since = time.time()
                elif (time.time() - self._no_face_since > 5.0
                      and not self._warned_no_face):
                    self._warned_no_face = True
                    self.get_logger().warn(
                        'Camera is delivering frames but NO FACE has been '
                        'detected. Point the camera at your face - and check '
                        'you picked the right camera in the Gaze panel. This '
                        'machine has an integrated webcam and a USB one, and '
                        'only one of them is looking at you.')
            return

        self._face_seen_at = time.time()
        self._no_face_since = 0.0
        self._warned_no_face = False

        # A blinking frame is not a gaze measurement. Using one drags the point
        # towards wherever the estimate goes when the eye is closed, which is
        # usually the middle.
        self.gaze_valid = not face.blinking
        self.features = face.features

        if self._model_ready and self._model.is_ready():
            self.gaze_u, self.gaze_v = self._model.predict(face.features)
        else:
            # Uncalibrated: show the raw eye position so the operator can at
            # least see the tracker is alive. Deliberately NOT used for driving
            # - an uncalibrated point is meaningless.
            self.gaze_u = float(np.clip(0.5 + face.features[0] * 2.0, 0.0, 1.0))
            self.gaze_v = float(np.clip(0.5 + face.features[1] * 2.0, 0.0, 1.0))

        if self.publish_face:
            self._publish_face_crop(cam, face)

        # Publish the features the calibration needs. Sent AFTER the model has
        # consumed them so the console can never sample a frame this node did
        # not also act on.
        self.feature_pub.publish(Float32MultiArray(
            data=[float(x) for x in face.features]))

    def _update_gaze_from_mouse(self):
        """Pointer position -> normalized gaze.

        Valid from the moment the HUD opens, defaulting to centre. The
        alternative - waiting for a click - is a source that silently reports
        "no gaze" until the operator happens to press a button, which makes
        every calibration abort on its first dot and looks exactly like a
        broken tracker.
        """
        w = self.frame.shape[1] if self.frame is not None else 640
        h = self.frame.shape[0] if self.frame is not None else 480
        if self.mouse_click is not None:
            mx, my = self.mouse_click
            self.gaze_u = float(np.clip(mx / max(1, w), 0.0, 1.0))
            self.gaze_v = float(np.clip(my / max(1, h), 0.0, 1.0))
        self.gaze_valid = True

    def _classify(self, dx, dy, rx, ry):
        """Map an offset from centre to a control zone, with an oval deadzone.

        `rx` and `ry` are separate on purpose. A circular deadzone sized for
        horizontal comfort makes the vertical zones unreachable: the eye
        travels much further sideways than up and down, so the operator could
        turn easily but almost never got far enough above or below centre to
        drive. Normalising each axis by its own radius fixes that without
        touching how far sideways the eyes must go.

        Returns (zone, normalized_distance) - 1.0 is the deadzone edge.
        """
        nx = dx / max(1e-6, rx)
        ny = dy / max(1e-6, ry)
        dist = float(np.hypot(nx, ny))
        if dist < 1.0:
            return ZONE_CENTER, dist
        # Compare the NORMALISED offsets, so "up" competes on equal terms with
        # "sideways" rather than losing because the eye moves less that way.
        if abs(nx) >= abs(ny):
            return (ZONE_TURN_LEFT if nx < 0 else ZONE_TURN_RIGHT), dist
        return (ZONE_FORWARD if ny < 0 else ZONE_BACKWARD), dist

    def _publish_servo(self):
        now = time.time()
        if now - self._last_servo < self._servo_period:
            return
        self._last_servo = now
        # x = s1 = TILT, y = s2 = PAN. The arbiter reads it that way and the
        # hardware is wired that way; sending them crossed is what pinned both
        # axes against each other's limits.
        self.servo_pub.publish(
            Vector3(x=float(self.tilt), y=float(self.pan), z=0.0))

    def _assert_mode(self):
        # Only ever *offer* the mode, and only while this node is genuinely
        # tracking. Silence otherwise: an idle or crashed HUD must never be
        # able to take the robot away from the operator.
        if self.assert_mode and self.gaze_valid:
            self.mode_pub.publish(UInt8(data=MODE_GAZE))

    def _hud_tick(self):
        # ---- 1. ACQUIRE -------------------------------------------------
        # Every source updates gaze_u/gaze_v/gaze_valid here, BEFORE anything
        # is published.
        #
        # The mouse update used to live further down, after /gaze/raw had
        # already been published, so the published value was always one tick
        # stale - and on the first tick it was the stale INVALID default, which
        # is what made every calibration abort on its first dot.
        if self.gaze_source == 'mediapipe':
            self._update_gaze_from_webcam()
        elif self.gaze_source == 'mouse':
            self._update_gaze_from_mouse()

        # ---- 2. PUBLISH -------------------------------------------------
        # Feed the calibration stream. Published here rather than in the
        # individual source callbacks so every source - mediapipe, mouse or
        # external - produces the same /gaze/raw, and the console never has to
        # know which one is running.
        #
        # RAW, not smoothed: the calibration fits a line through these samples,
        # and filtering first would bias the fit toward wherever the eyes had
        # been resting.
        raw = Point()
        raw.x = float(self.gaze_u)
        raw.y = float(self.gaze_v)
        raw.z = 1.0 if self.gaze_valid else 0.0
        self.raw_pub.publish(raw)

        # Everything that decides whether to drive uses the SMOOTHED point.
        if not self.gaze_valid:
            self._was_valid = False
            su, sv = self.smooth_u, self.smooth_v
        else:
            if not self._was_valid:
                # First good frame after a gap: start from here rather than
                # sliding in from wherever the eyes were when tracking stopped.
                self._reset_smooth(self.gaze_u, self.gaze_v)
                self._was_valid = True
            su, sv = self._smooth(self.gaze_u, self.gaze_v)

        # Apply the calibration for the mediapipe source only. Mouse gaze is
        # already in the frame the operator is pointing at, and the external
        # source has been corrected upstream by gaze_udp_bridge - correcting
        # either of those again would shift them for no reason.
        if self.gaze_source == 'mediapipe':
            cu, cv = self._correct(su, sv)
            pt = Point()
            pt.x, pt.y = float(cu), float(cv)
            pt.z = 0.0
            self.point_pub.publish(pt)
        else:
            cu, cv = su, sv

        frame = self.frame
        if frame is None:
            return

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        # Oval deadzone in pixels. The ratios are already radii as a fraction
        # of the control space, so there is no further halving: multiplying by
        # 0.5 here is what made the deadzone half the size it claimed to be.
        dz_rx = max(4.0, w * self.deadzone_ratio)
        dz_ry = max(4.0, h * self.deadzone_ratio_y)

        # Everything downstream works on the CORRECTED point: that is where
        # the operator meant to look. The raw point is still drawn, hollow,
        # so it is obvious when the calibration is pulling hard in one
        # direction.
        gx = int(cu * w)
        gy = int(cv * h)
        # The RAW point is no longer clamped to 0..1 (a biased tracker legitimately
        # reads outside it), so clamp it here purely for drawing - an off-screen
        # coordinate would put the marker outside the image and look like a bug.
        rx = int(min(1.0, max(0.0, self.gaze_u)) * w)
        ry = int(min(1.0, max(0.0, self.gaze_v)) * h)
        dx, dy = gx - cx, gy - cy

        zone, dist = self._classify(dx, dy, dz_rx, dz_ry)
        now = time.time()

        # ---- dwell state machine ----
        if zone != self.zone:
            self.zone = zone
            self.zone_since = now
            self.confirmed_zone = ZONE_CENTER

        twist = Twist()

        if zone == ZONE_CENTER:
            # Fine PTZ aiming only - the chassis never moves in the deadzone.
            #
            # POSITIONAL, not integrating. An earlier version added a small
            # step to the servo angle on every tick, which is rate control:
            # holding the eyes slightly off centre drove the head further and
            # further until it hit a travel limit and stayed there. At 30 Hz a
            # full deflection moved the servo 45 deg/s, which is the jerkiness
            # the operator felt, and it made fine aiming impossible.
            #
            # Here the offset maps to an absolute angle, so a small look is a
            # small angle that then STOPS, and looking back returns the head to
            # centre. The range is inherently bounded by the span.
            if self.gaze_valid:
                fx = float(np.clip(-dx / max(1.0, dz_rx), -1.0, 1.0))
                fy = float(np.clip(-dy / max(1.0, dz_ry), -1.0, 1.0))
                if self.invert_pan:
                    fx = -fx
                if self.invert_tilt:
                    fy = -fy
                # Full deflection at the deadzone edge == half the span.
                pan_target = self.pan_center + fx * (self.pan_span * 0.5)
                tilt_target = self.tilt_center + fy * (self.tilt_span * 0.5)
                pan_target = float(np.clip(pan_target, self.s2_min, self.s2_max))
                tilt_target = float(np.clip(tilt_target, self.s1_min, self.s1_max))

                # Low-pass, so tracker jitter does not vibrate the servo. This
                # is what turns "correct but twitchy" into "smooth".
                a = self.servo_smoothing
                self.pan += a * (pan_target - self.pan)
                self.tilt += a * (tilt_target - self.tilt)
                self._publish_servo()
        else:
            dwell = now - self.zone_since
            if dwell >= self.dwell_threshold:
                self.confirmed_zone = zone
                if zone == ZONE_FORWARD:
                    twist.linear.x = self.drive_vx
                elif zone == ZONE_BACKWARD:
                    twist.linear.x = -self.drive_vx
                elif zone == ZONE_TURN_LEFT:
                    twist.angular.z = self.turn_wz
                elif zone == ZONE_TURN_RIGHT:
                    twist.angular.z = -self.turn_wz

        self.vel_pub.publish(twist)
        self.zone_pub.publish(String(data=self.confirmed_zone))

        # ---- HUD ----
        overlay = frame.copy()
        cv2.ellipse(overlay, (cx, cy), (int(dz_rx), int(dz_ry)), 0, 0, 360,
                    (0, 200, 0), 2)
        cv2.line(overlay, (cx - 12, cy), (cx + 12, cy), (0, 200, 0), 1)
        cv2.line(overlay, (cx, cy - 12), (cx, cy + 12), (0, 200, 0), 1)
        cv2.line(overlay, (cx, 0), (cx, h), (70, 70, 70), 1)
        cv2.line(overlay, (0, cy), (w, cy), (70, 70, 70), 1)

        # Reticle colour encodes state: red = driving, green = looking.
        col = (0, 0, 255) if self.confirmed_zone != ZONE_CENTER else (0, 255, 0)
        cv2.circle(overlay, (rx, ry), 14, (120, 120, 120), 1)
        cv2.circle(overlay, (gx, gy), 10, col, 2)
        cv2.circle(overlay, (gx, gy), 2, col, -1)

        label = self.confirmed_zone
        if zone != ZONE_CENTER and self.confirmed_zone == ZONE_CENTER:
            remain = max(0.0, self.dwell_threshold - (now - self.zone_since))
            label = f'{zone} ({remain:.2f}s)'

        cv2.putText(overlay, f'ZONE: {label}', (16, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(overlay, f'MODE: 2-GAZE', (16, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(overlay, f'PTZ P{int(self.pan)} T{int(self.tilt)}',
                    (16, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        bat = 'BAT: --' if np.isnan(self.battery) else f'BAT: {self.battery:.1f}V'
        bcol = (0, 255, 0)
        if not np.isnan(self.battery) and self.battery < 10.5:
            bcol = (0, 0, 255)
        cv2.putText(overlay, bat, (w - 150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, bcol, 2)
        cv2.putText(overlay, f'{self.fps:.0f} FPS', (w - 150, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        if not self.gaze_valid:
            cv2.putText(overlay, 'NO GAZE', (cx - 60, int(cy - dz_ry - 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        if now - self._fps_t >= 1.0:
            self.fps = self._fps_n / (now - self._fps_t)
            self._fps_n = 0
            self._fps_t = now

        if self.show_window:
            if not self._window_ready:
                cv2.namedWindow(self._WIN, cv2.WINDOW_NORMAL)
                cv2.setMouseCallback(self._WIN, self._on_mouse)
                self._window_ready = True
            cv2.imshow(self._WIN, overlay)
            key = cv2.waitKey(1) & 0xFF

            # Closing the window with the X button does NOT stop this loop.
            # imshow() then re-creates the window on the next frame, so the
            # HUD looked like it "kept reopening" no matter how many times it
            # was closed - nothing was ever asking whether it still existed.
            #
            # getWindowProperty raises cv2.error once the window is gone on
            # some backends and returns < 1 on others, so both mean the same
            # thing here: the operator wants out.
            #
            # Guarded by _saw_visible on purpose. Not every OpenCV backend
            # implements WND_PROP_VISIBLE; those return -1 forever. Without the
            # guard the HUD would exit on its first frame on exactly the
            # machines where the check cannot work - turning "the window will
            # not close" into "the window never opens".
            try:
                vis = cv2.getWindowProperty(self._WIN, cv2.WND_PROP_VISIBLE)
                if vis >= 1:
                    self._saw_visible = True
                elif self._saw_visible:
                    raise KeyboardInterrupt
            except cv2.error:
                if self._saw_visible:
                    raise KeyboardInterrupt

            if key == ord('q'):
                raise KeyboardInterrupt
            if key == ord('u'):
                # Same axis swap as everywhere else: s1 is tilt, s2 is pan.
                self.pan = self.pan_center
                self.tilt = self.tilt_center
                self._publish_servo()

    def destroy_node(self):
        # getattr throughout: destroy_node runs on failure paths too, including
        # ones where __init__ did not get far enough to set these.
        cap = getattr(self, '_cap', None)
        if cap is not None:
            cap.release()
        if getattr(self, 'show_window', False):
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeHUDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001
        # ExternalShutdownException and RCLError both arrive here when the
        # process is killed with SIGTERM (timeout, Ctrl+C in a wrapper script,
        # systemd). Printing a traceback for that is noise: nothing is wrong,
        # the context was simply shut down underneath us. The console has the
        # same guard for the same reason.
        pass
    finally:
        try:
            node.vel_pub.publish(Twist())
        except Exception:  # noqa: BLE001 - context may already be gone
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
