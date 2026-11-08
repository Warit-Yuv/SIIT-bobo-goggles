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

import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist, Vector3
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, UInt8

try:
    from m1_control.gaze_calib import apply as apply_calib
except ImportError:  # run as a plain script rather than through ros2 run
    from gaze_calib import apply as apply_calib

MODE_GAZE = 2

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
        self.declare_parameter('deadzone_radius_ratio', 0.25)

        self.declare_parameter('drive_speed_x', 0.20)
        self.declare_parameter('drive_speed_y', 0.20)
        self.declare_parameter('turn_speed_z', 0.60)
        self.declare_parameter('servo_gain_pan', 1.5)
        self.declare_parameter('servo_gain_tilt', 1.5)
        self.declare_parameter('servo_rate_limit_hz', 12.0)

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

        gp = self.get_parameter
        self.camera_topic = gp('camera_topic').value
        self.gaze_source = str(gp('gaze_source').value).lower()
        self.dwell_threshold = float(gp('dwell_time_sec').value)
        self.deadzone_ratio = float(gp('deadzone_radius_ratio').value)
        self.drive_vx = float(gp('drive_speed_x').value)
        self.drive_vy = float(gp('drive_speed_y').value)
        self.turn_wz = float(gp('turn_speed_z').value)
        self.gain_pan = float(gp('servo_gain_pan').value)
        self.gain_tilt = float(gp('servo_gain_tilt').value)
        self.show_window = bool(gp('show_window').value)
        self.assert_mode = bool(gp('assert_mode').value)

        self.pan = float(gp('servo_s1_center').value)
        self.tilt = float(gp('servo_s2_center').value)
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
        self.mouse_click = None
        self._window_ready = False
        self._saw_visible = False
        self._WIN = WINDOW_TITLE

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

        self.create_subscription(CompressedImage, self.camera_topic,
                                 self.image_cb, sensor_qos)
        self.create_subscription(Float32, '/voltage', self.voltage_cb, sensor_qos)
        self.create_subscription(Point, '/gaze/point', self.external_gaze_cb, 10)
        self.create_subscription(Float32MultiArray, '/gaze/calib',
                                 self._calib_cb, 10)
        self.create_timer(1.0, self._report_calib)

        self._face_mesh = None
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
    def _init_mediapipe(self):
        try:
            import mediapipe as mp
            self._mp = mp
            self._face_mesh = mp.solutions.face_mesh.FaceMesh(
                max_num_faces=1,
                refine_landmarks=True,          # needed for iris landmarks
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5)
            self._cap = cv2.VideoCapture(int(self.get_parameter('webcam_id').value))
            if not self._cap.isOpened():
                raise RuntimeError('webcam did not open')
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('MediaPipe iris tracking ready')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'mediapipe/webcam unavailable ({exc}) - falling back to mouse gaze')
            self._face_mesh = None
            self.gaze_source = 'mouse'

    # --------------------------------------------------------------- callbacks
    def voltage_cb(self, msg: Float32):
        self.battery = float(msg.data)

    def _calib_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warn(
                f'/gaze/calib needs 4 floats [ox, oy, gx, gy], got {len(data)}')
            return
        self._calib = [float(v) for v in data[:4]]
        self.get_logger().info(
            'Calibration set: offset=(%.3f, %.3f) gain=(%.2f, %.2f)'
            % tuple(self._calib))
        self._report_calib()

    def _report_calib(self):
        self.calib_state_pub.publish(
            Float32MultiArray(data=[float(v) for v in self._calib]))

    def _correct(self, u, v):
        """Raw tracker output -> the point the operator actually meant.

        Same function the console uses to FIT the calibration, imported
        rather than reimplemented: a fit and an apply that disagree by a sign
        or an intercept look exactly like "calibration does nothing".
        """
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
        """Mouse gaze: any click or drag inside the HUD aims the reticle."""
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_MOUSEMOVE) and flags & cv2.EVENT_FLAG_LBUTTON \
                or event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_click = (int(x), int(y))

    # ------------------------------------------------------------- gaze input
    def _update_gaze_from_webcam(self):
        """Iris position -> normalized screen coordinates."""
        if self._face_mesh is None:
            return
        ok, cam = self._cap.read()
        if not ok:
            self.gaze_valid = False
            return
        cam = cv2.flip(cam, 1)
        rgb = cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)
        res = self._face_mesh.process(rgb)
        if not res.multi_face_landmarks:
            self.gaze_valid = False
            return

        lm = res.multi_face_landmarks[0].landmark
        # MediaPipe FaceMesh iris indices: 468-472 left, 473-477 right.
        # 33/133 = left eye corners, 362/263 = right eye corners.
        l_iris = np.mean([[lm[i].x, lm[i].y] for i in range(468, 473)], axis=0)
        r_iris = np.mean([[lm[i].x, lm[i].y] for i in range(473, 478)], axis=0)
        l_out, l_in = np.array([lm[33].x, lm[33].y]), np.array([lm[133].x, lm[133].y])
        r_in, r_out = np.array([lm[362].x, lm[362].y]), np.array([lm[263].x, lm[263].y])

        def ratio(iris, a, b):
            span = np.linalg.norm(b - a)
            return 0.0 if span < 1e-6 else float(np.dot(iris - a, b - a) / (span ** 2))

        # Left eye runs outer->inner, right eye inner->outer, so both ratios
        # increase toward the nose. Average and map to -1..+1.
        h = 0.5 * (ratio(l_iris, l_out, l_in) + ratio(r_iris, r_in, r_out))
        # Vertical: iris height relative to the eye opening (159 upper, 145 lower).
        upper = np.array([lm[159].x, lm[159].y])
        lower = np.array([lm[145].x, lm[145].y])
        v = ratio(l_iris, upper, lower)

        self.gaze_u = float(np.clip((h - 0.5) * 3.0 + 0.5, 0.0, 1.0))
        self.gaze_v = float(np.clip((v - 0.5) * 3.0 + 0.5, 0.0, 1.0))
        self.gaze_valid = True

    # ------------------------------------------------------------------ logic
    def _classify(self, dx, dy, deadzone_r):
        dist = float(np.hypot(dx, dy))
        if dist < deadzone_r:
            return ZONE_CENTER, dist
        # Horizontal component dominates -> turn, otherwise drive.
        if abs(dx) >= abs(dy):
            return (ZONE_TURN_LEFT if dx < 0 else ZONE_TURN_RIGHT), dist
        return (ZONE_FORWARD if dy < 0 else ZONE_BACKWARD), dist

    def _publish_servo(self):
        now = time.time()
        if now - self._last_servo < self._servo_period:
            return
        self._last_servo = now
        self.servo_pub.publish(
            Vector3(x=float(self.pan), y=float(self.tilt), z=0.0))

    def _assert_mode(self):
        # Only ever *offer* the mode, and only while this node is genuinely
        # tracking. Silence otherwise: an idle or crashed HUD must never be
        # able to take the robot away from the operator.
        if self.assert_mode and self.gaze_valid:
            self.mode_pub.publish(UInt8(data=MODE_GAZE))

    def _hud_tick(self):
        if self.gaze_source == 'mediapipe':
            self._update_gaze_from_webcam()

        # Feed the calibration stream. Published here rather than in the
        # individual source callbacks so every source - mediapipe, mouse or
        # external - produces the same /gaze/raw, and the console never has to
        # know which one is running.
        raw = Point()
        raw.x = float(self.gaze_u)
        raw.y = float(self.gaze_v)
        raw.z = 1.0 if self.gaze_valid else 0.0
        self.raw_pub.publish(raw)

        # Apply the calibration for the mediapipe source only. Mouse gaze is
        # already in the frame the operator is pointing at, and the external
        # source has been corrected upstream by gaze_udp_bridge - correcting
        # either of those again would shift them for no reason.
        if self.gaze_source == 'mediapipe':
            cu, cv = self._correct(self.gaze_u, self.gaze_v)
            pt = Point()
            pt.x, pt.y = float(cu), float(cv)
            pt.z = 0.0
            self.point_pub.publish(pt)
        else:
            cu, cv = self.gaze_u, self.gaze_v

        frame = self.frame
        if frame is None:
            return

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        deadzone_r = int(min(w, h) * self.deadzone_ratio)

        # Mouse gaze: convert the last click into normalized coordinates.
        if self.gaze_source == 'mouse' and self.mouse_click is not None:
            mx, my = self.mouse_click
            self.gaze_u = float(np.clip(mx / max(1, w), 0.0, 1.0))
            self.gaze_v = float(np.clip(my / max(1, h), 0.0, 1.0))
            self.gaze_valid = True

        # Everything downstream works on the CORRECTED point: that is where
        # the operator meant to look. The raw point is still drawn, hollow,
        # so it is obvious when the calibration is pulling hard in one
        # direction.
        gx = int(cu * w)
        gy = int(cv * h)
        rx = int(self.gaze_u * w)
        ry = int(self.gaze_v * h)
        dx, dy = gx - cx, gy - cy

        zone, dist = self._classify(dx, dy, deadzone_r)
        now = time.time()

        # ---- dwell state machine ----
        if zone != self.zone:
            self.zone = zone
            self.zone_since = now
            self.confirmed_zone = ZONE_CENTER

        twist = Twist()

        if zone == ZONE_CENTER:
            # Fine PTZ aiming only - the chassis never moves in the deadzone.
            if self.gaze_valid:
                pan_step = -(dx / max(1, deadzone_r)) * self.gain_pan
                tilt_step = -(dy / max(1, deadzone_r)) * self.gain_tilt
                if self.invert_pan:
                    pan_step = -pan_step
                if self.invert_tilt:
                    tilt_step = -tilt_step
                self.pan = float(np.clip(self.pan + pan_step, self.s1_min, self.s1_max))
                self.tilt = float(np.clip(self.tilt + tilt_step, self.s2_min, self.s2_max))
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

        # ---- HUD ----
        overlay = frame.copy()
        cv2.circle(overlay, (cx, cy), deadzone_r, (0, 200, 0), 2)
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
            cv2.putText(overlay, 'NO GAZE', (cx - 60, cy - deadzone_r - 14),
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
                self.pan = float(self.get_parameter('servo_s1_center').value)
                self.tilt = float(self.get_parameter('servo_s2_center').value)
                self._publish_servo()

    def destroy_node(self):
        if getattr(self, '_cap', None) is not None:
            self._cap.release()
        if self.show_window:
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
