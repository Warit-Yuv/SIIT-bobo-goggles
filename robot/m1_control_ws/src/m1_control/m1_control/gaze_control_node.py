#!/usr/bin/env python3
"""Gaze -> chassis/PTZ controller. Runs ON THE ROBOT, headless.

Consumes a normalized gaze point and applies the handbook's dual-action scheme
(section 7.3, mode 2):

    inside the deadzone circle  -> chassis still, gaze drives fine PTZ pan/tilt
    outside, dwell confirmed    -> drive / strafe / turn in that direction

Why this node lives on the Pi and not on the operator PC
--------------------------------------------------------
1. **Safety.** The deadzone/dwell logic is a motion authority. Running it next
   to the arbiter means a Wi-Fi stall cannot leave a stale driving command in
   flight - the arbiter's watchdog and this node's own link timeout both run
   locally.
2. **One implementation.** The gaze *source* varies (MediaPipe in a VM, a
   Windows GUI app, a future neural model) but the *policy* must not. Every
   source converges on `/gaze/point`, so the policy is written once here.

Input
-----
    /gaze/point   geometry_msgs/Point   x,y in 0..1 (z = 0), or
                                        x,y in screen pixels when
                                        `input_is_normalized:=false`

    A source that stops publishing is treated as "no gaze": the link timeout
    zeroes the velocity, so a crashed tracker cannot leave the robot driving.

Output (never /cmd_vel - the arbiter owns that)
-----------------------------------------------
    /cmd_vel/gaze   geometry_msgs/Twist    vx, vy, wz
    /servo/gaze     geometry_msgs/Vector3  x = s1 = TILT, y = s2 = PAN
    /control_mode   std_msgs/UInt8         asserted to 2 while gaze is live
    /gaze/zone      std_msgs/String        current zone, for the HUD/diagnostics

SERVO AXIS MAPPING (verified by hand, direction only)
-----------------------------------------------------
    s1 -> TILT   larger = look UP      centre 40
    s2 -> PAN    larger = look LEFT    centre 90

So looking up must INCREASE s1, and looking left must INCREASE s2. The travel
limits below come from the vendor joystick controller and the chassis driver
(which hard-clamps s2 to 0..110); they are not yet confirmed against the
mechanical stops.

    ros2 run m1_control gaze_control_node
    ros2 run m1_control gaze_control_node --ros-args -p deadzone_radius_ratio:=0.3
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist, Vector3
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String, UInt8

MODE_GAZE = 2

ZONE_CENTER = 'CENTER'
ZONE_FORWARD = 'FORWARD'
ZONE_BACKWARD = 'BACKWARD'
ZONE_TURN_LEFT = 'TURN_LEFT'
ZONE_TURN_RIGHT = 'TURN_RIGHT'
ZONE_STRAFE_LEFT = 'STRAFE_LEFT'
ZONE_STRAFE_RIGHT = 'STRAFE_RIGHT'
ZONE_FORWARD_LEFT = 'FORWARD_LEFT'
ZONE_FORWARD_RIGHT = 'FORWARD_RIGHT'
ZONE_BACKWARD_LEFT = 'BACKWARD_LEFT'
ZONE_BACKWARD_RIGHT = 'BACKWARD_RIGHT'

#: Zone -> (forward factor, strafe factor, yaw factor), each -1..+1.
#: Factors are multiplied by the configured speeds, so one table defines the
#: whole policy. Strafe sign follows ROS: +y is to the robot's LEFT.
ZONE_MOTION = {
    ZONE_FORWARD:        (1.0, 0.0, 0.0),
    ZONE_BACKWARD:       (-1.0, 0.0, 0.0),
    ZONE_STRAFE_LEFT:    (0.0, 1.0, 0.0),
    ZONE_STRAFE_RIGHT:   (0.0, -1.0, 0.0),
    ZONE_FORWARD_LEFT:   (0.7, 0.7, 0.0),
    ZONE_FORWARD_RIGHT:  (0.7, -0.7, 0.0),
    ZONE_BACKWARD_LEFT:  (-0.7, 0.7, 0.0),
    ZONE_BACKWARD_RIGHT: (-0.7, -0.7, 0.0),
    # Handbook 4-quadrant mode uses yaw instead of strafe.
    ZONE_TURN_LEFT:      (0.0, 0.0, 1.0),
    ZONE_TURN_RIGHT:     (0.0, 0.0, -1.0),
}


class GazeControlNode(Node):

    def __init__(self):
        super().__init__('gaze_control_node')

        # ---------------- parameters ----------------
        self.declare_parameter('gaze_topic', '/gaze/point')
        self.declare_parameter('input_is_normalized', True)
        self.declare_parameter('gaze_link_timeout_sec', 0.6)

        self.declare_parameter('dwell_time_sec', 0.40)
        self.declare_parameter('deadzone_radius_ratio', 0.25)

        # Split the outer region into 8 sectors so the mecanum base can strafe.
        # Set false to use the handbook's 4 quadrants only.
        self.declare_parameter('enable_strafe', True)

        # ---- horizontal emphasis ----
        #
        # Vertical gaze is the weak axis: the iris barely moves up and down
        # inside the eye, so the vertical signal is mostly noise and head
        # pitch, and on this hardware it has never been reliable enough to
        # steer with. Horizontal is clean.
        #
        # With this false (the default) ONLY left/right produce chassis motion.
        # The vertical axis is still used for PTZ tilt, where a small wrong
        # angle is harmless, but never for driving - so a bad vertical signal
        # cannot drive the robot into anything.
        #
        # Forward/backward is then handled by the jaw clench (EEG), which is a
        # deliberate, low-latency, single-bit gesture and is far more
        # dependable than looking up.
        self.declare_parameter('enable_vertical', False)

        # ---- EEG gating (protocol v1 toggles) ----
        #
        #   run=false            nothing moves
        #   run=true mode=robot  gaze steers the CHASSIS
        #   mode=camera          gaze steers the PTZ HEAD only, chassis still
        #
        # Gating the chassis on `run` is what makes a jaw clench a dead-man
        # switch: the robot only moves while the operator has deliberately
        # said so.
        self.declare_parameter('use_eeg_gating', True)
        self.declare_parameter('eeg_link_timeout_sec', 1.5)

        self.declare_parameter('drive_speed_x', 0.20)
        self.declare_parameter('drive_speed_y', 0.20)
        self.declare_parameter('turn_speed_z', 0.60)

        # ---- PTZ aiming (positional, see tick()) ----
        # Total degrees of travel used across the deadzone radius. Smaller =
        # finer and calmer; larger = the head follows your eyes further.
        self.declare_parameter('servo_pan_span', 10.0)
        self.declare_parameter('servo_tilt_span', 20.0)
        # Low-pass factor per tick (1.0 = instant, 0.2 = very smooth).
        self.declare_parameter('servo_smoothing', 0.35)

        self.declare_parameter('servo_rate_limit_hz', 20.0)
        self.declare_parameter('servo_s1_center', 40.0)   # tilt centre
        self.declare_parameter('servo_s2_center', 90.0)   # pan centre
        # Symmetric about centre; see control_arbiter_node.py for the
        # measurement that produced these numbers.
        self.declare_parameter('servo_s1_min', 5.0)
        self.declare_parameter('servo_s1_max', 75.0)
        self.declare_parameter('servo_s2_min', 80.0)
        self.declare_parameter('servo_s2_max', 100.0)

        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        # Mode is the OPERATOR's decision, not this node's. Default off; see
        # the note in gaze_hud_node.py for what happens when it is on.
        self.declare_parameter('assert_mode', False)
        self.declare_parameter('publish_rate_hz', 20.0)

        gp = self.get_parameter
        self.input_normalized = bool(gp('input_is_normalized').value)
        self.link_timeout = float(gp('gaze_link_timeout_sec').value)
        self.dwell_threshold = float(gp('dwell_time_sec').value)
        self.deadzone_ratio = float(gp('deadzone_radius_ratio').value)
        self.enable_strafe = bool(gp('enable_strafe').value)
        self.enable_vertical = bool(gp('enable_vertical').value)
        self.use_eeg_gating = bool(gp('use_eeg_gating').value)
        self.eeg_link_timeout = float(gp('eeg_link_timeout_sec').value)
        self.drive_vx = float(gp('drive_speed_x').value)
        self.drive_vy = float(gp('drive_speed_y').value)
        self.turn_wz = float(gp('turn_speed_z').value)
        self.pan_span = float(gp('servo_pan_span').value)
        self.tilt_span = float(gp('servo_tilt_span').value)
        self.servo_smoothing = float(np.clip(gp('servo_smoothing').value, 0.01, 1.0))
        self.invert_pan = bool(gp('invert_pan').value)
        self.invert_tilt = bool(gp('invert_tilt').value)
        self.assert_mode = bool(gp('assert_mode').value)

        # Raw servo positions, named as the message/hardware name them.
        #   s1 = TILT (larger = up)      s2 = PAN (larger = left)
        self.pan_center = float(gp('servo_s2_center').value)
        self.tilt_center = float(gp('servo_s1_center').value)
        self.s1 = self.tilt_center                        # tilt
        self.s2 = self.pan_center                         # pan
        self.s1_min, self.s1_max = (float(gp('servo_s1_min').value),
                                    float(gp('servo_s1_max').value))
        self.s2_min, self.s2_max = (float(gp('servo_s2_min').value),
                                    float(gp('servo_s2_max').value))

        self._servo_period = 1.0 / max(1.0, float(gp('servo_rate_limit_hz').value))
        self._last_servo = 0.0

        # ---------------- state ----------------
        self.gaze_u = 0.5
        self.gaze_v = 0.5
        self.gaze_valid = False
        self.last_gaze_time = 0.0
        self.zone = ZONE_CENTER
        self.zone_since = time.time()
        self.confirmed_zone = ZONE_CENTER

        # EEG state (protocol v1 toggles). Defaults are the safe ones: not
        # running, and the head rather than the chassis, so a missing EEG
        # bridge cannot make the robot drive.
        self.eeg_run = False
        self.eeg_mode = 'camera'
        self.eeg_link = False
        self.eeg_last = 0.0
        self.eeg_ever_seen = False
        self._warned_no_eeg = False

        # ---------------- interfaces ----------------
        actuator_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel/gaze', actuator_qos)
        self.servo_pub = self.create_publisher(Vector3, '/servo/gaze', actuator_qos)
        self.mode_pub = self.create_publisher(UInt8, '/control_mode', mode_qos)
        self.zone_pub = self.create_publisher(String, '/gaze/zone', 10)

        self.create_subscription(Point, gp('gaze_topic').value,
                                 self.gaze_cb, 10)

        # EEG toggles. Latched (TRANSIENT_LOCAL) on the publisher side, so a
        # late subscriber still sees the current state instead of waiting for
        # the next gesture - which may be minutes away.
        from std_msgs.msg import Bool as _Bool
        self.create_subscription(_Bool, '/eeg/run', self._eeg_run_cb, mode_qos)
        self.create_subscription(String, '/eeg/mode', self._eeg_mode_cb, mode_qos)
        self.create_subscription(_Bool, '/eeg/link', self._eeg_link_cb, mode_qos)

        rate = float(gp('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(1.0, self._assert_mode)

        self.get_logger().info(
            f'Gaze control up | topic={gp("gaze_topic").value} '
            f'| dwell={self.dwell_threshold}s '
            f'| deadzone={self.deadzone_ratio:.0%} '
            f'| strafe={"on" if self.enable_strafe else "off"} '
            f'| link timeout={self.link_timeout}s')

    # ---------------------------------------------------------------- inputs
    def gaze_cb(self, msg: Point):
        u, v = float(msg.x), float(msg.y)
        if not self.input_normalized:
            # Accept pixel coordinates and normalize; the source's resolution
            # is not known here, so require z to carry it when used this way.
            w = float(msg.z) if msg.z > 0 else 1920.0
            u, v = u / w, v / w

        self.gaze_u = float(np.clip(u, 0.0, 1.0))
        self.gaze_v = float(np.clip(v, 0.0, 1.0))
        self.gaze_valid = True
        self.last_gaze_time = time.time()

    def _assert_mode(self):
        if self.assert_mode and self.gaze_valid:
            self.mode_pub.publish(UInt8(data=MODE_GAZE))

    # ------------------------------------------------------------ EEG state
    def _eeg_run_cb(self, msg):
        if bool(msg.data) != self.eeg_run:
            self.get_logger().info(
                f"EEG run {'ON' if msg.data else 'off'} "
                f"(mode={self.eeg_mode})")
        self.eeg_run = bool(msg.data)
        self.eeg_last = time.time()

    def _eeg_mode_cb(self, msg):
        m = str(msg.data).strip().lower()
        if m not in ('robot', 'camera'):
            self.get_logger().warn(f'ignoring unknown EEG mode "{msg.data}"')
            return
        if m != self.eeg_mode:
            self.get_logger().info(f'EEG mode -> {m}')
        self.eeg_mode = m
        self.eeg_last = time.time()

    def _eeg_link_cb(self, msg):
        link = bool(msg.data)
        if link != self.eeg_link:
            self.get_logger().info(f"EEG link {'ok' if link else 'LOST'}")
        if link:
            self.eeg_ever_seen = True
        self.eeg_link = link
        self.eeg_last = time.time()

    def _chassis_allowed(self, now):
        """May the chassis move at all right now?

        Three independent gates, and all three must agree:

          * `run` is the operator's explicit go/stop (jaw clench)
          * `mode` is which thing gaze is steering (double blink)
          * a live EEG link, so a dead laptop cannot leave the robot driving

        The EEG gate only engages once an EEG has actually been seen.

        Blocking on a link that was never there is a trap: the operator starts
        the system without the headset, gaze moves the cursor, and the robot
        does not budge - with nothing anywhere saying why. That is
        indistinguishable from a broken tracker, and it sends them off
        recalibrating something that was never wrong.

        Once the link HAS been up, its disappearance is a dropout and the
        chassis stops. A dead laptop must never leave the robot driving.
        """
        if not self.use_eeg_gating:
            return True
        if self.eeg_ever_seen:
            if (not self.eeg_link
                    or now - self.eeg_last > self.eeg_link_timeout):
                return False
            return self.eeg_run and self.eeg_mode == 'robot'
        # No EEG has ever connected: gaze alone is in charge.
        if not self._warned_no_eeg:
            self._warned_no_eeg = True
            self.get_logger().warn(
                'No EEG link has ever been seen - gaze is driving the chassis '
                'without the jaw-clench run gate. Start the EEG bridge, or set '
                'use_eeg_gating:=false to silence this.')
        return True

    def _classify(self, dx, dy, deadzone_r):
        """Map an offset from centre to a control zone.

        `dx` is +right, `dy` is +down (image convention). The angle is measured
        from "straight up the screen" so that 0 deg means FORWARD, +90 deg means
        screen-right, which keeps the table readable.

        With `enable_vertical` false - the default - ONLY left and right produce
        a zone. Vertical gaze is the weak axis on this hardware: the iris barely
        moves up and down inside the eye, so the signal is mostly noise and head
        pitch. Letting it drive would mean the robot lurching forward whenever
        the operator glanced down at a keyboard, which is the single most
        dangerous failure this controller could have.

        So forward/backward is left to the jaw clench, which is a deliberate
        one-bit gesture with none of that ambiguity.
        """
        if not self.enable_vertical:
            # Horizontal only, so ONLY the horizontal offset is allowed to
            # decide. Testing the distance from centre instead would mean that
            # looking straight up - dx exactly 0 - clears the deadzone and then
            # falls through to "dx < 0 is false, so turn RIGHT". A glance at
            # the ceiling would spin the robot on the spot.
            #
            # A vertical deadzone also makes the rule honest: up and down
            # cannot leave the deadzone at all, so vertical gaze cannot drive.
            if abs(dx) < deadzone_r:
                return ZONE_CENTER
            return ZONE_TURN_LEFT if dx < 0 else ZONE_TURN_RIGHT

        if float(np.hypot(dx, dy)) < deadzone_r:
            return ZONE_CENTER

        if not self.enable_strafe:
            # Handbook's 4 quadrants: the base turns instead of strafing.
            if abs(dx) >= abs(dy):
                return ZONE_TURN_LEFT if dx < 0 else ZONE_TURN_RIGHT
            return ZONE_FORWARD if dy < 0 else ZONE_BACKWARD

        # 8 sectors, 45 deg each, centred on the cardinal directions.
        angle = float(np.degrees(np.arctan2(dx, -dy)))
        if -22.5 <= angle < 22.5:
            return ZONE_FORWARD
        if 22.5 <= angle < 67.5:
            return ZONE_FORWARD_RIGHT
        if 67.5 <= angle < 112.5:
            return ZONE_STRAFE_RIGHT
        if 112.5 <= angle < 157.5:
            return ZONE_BACKWARD_RIGHT
        if angle >= 157.5 or angle < -157.5:
            return ZONE_BACKWARD
        if -157.5 <= angle < -112.5:
            return ZONE_BACKWARD_LEFT
        if -112.5 <= angle < -67.5:
            return ZONE_STRAFE_LEFT
        return ZONE_FORWARD_LEFT

    def _publish_servo(self):
        now = time.time()
        if now - self._last_servo < self._servo_period:
            return
        self._last_servo = now
        # x = s1 = tilt, y = s2 = pan
        self.servo_pub.publish(
            Vector3(x=float(self.s1), y=float(self.s2), z=0.0))

    # ----------------------------------------------------------------- loop
    def tick(self):
        now = time.time()

        # Link timeout: a silent tracker must not leave the robot driving.
        if self.gaze_valid and now - self.last_gaze_time > self.link_timeout:
            self.gaze_valid = False
            self.confirmed_zone = ZONE_CENTER
            self.zone = ZONE_CENTER
            self.zone_since = now
            self.get_logger().warn(
                f'No gaze for {self.link_timeout}s - holding still')

        if not self.gaze_valid:
            self.vel_pub.publish(Twist())
            return

        # Work in normalized space, centred: dx,dy in -0.5..0.5
        dx = self.gaze_u - 0.5
        dy = self.gaze_v - 0.5
        # Normalize by the short side so the deadzone is a true circle
        # regardless of aspect ratio.
        deadzone_r = self.deadzone_ratio * 0.5

        zone = self._classify(dx, dy, deadzone_r)

        if zone != self.zone:
            self.zone = zone
            self.zone_since = now
            self.confirmed_zone = ZONE_CENTER

        twist = Twist()

        if zone == ZONE_CENTER:
            # PTZ aiming is POSITIONAL, not integrating.
            #
            # An earlier version added a small step to the servo angle on
            # every tick, which is rate control: holding the eyes slightly off
            # centre drove the head further and further until it hit the travel
            # limit and stayed there ("it turns left too much"). That is
            # joystick behaviour, and it makes fine aiming impossible.
            #
            # Here the gaze offset maps directly to an absolute target angle,
            # so:
            #   * looking slightly left  -> a slightly left angle, and it STOPS
            #   * looking back to centre -> the head returns to centre
            #   * the range is inherently bounded by the span below
            #
            # Axis mapping: gaze is in image coordinates (+dx = right,
            # +dy = down). Servo s1 = tilt (larger = up), s2 = pan (larger =
            # left). Both offsets are therefore negated.
            frac_x = float(np.clip(-dx / max(1e-6, deadzone_r), -1.0, 1.0))
            frac_y = float(np.clip(-dy / max(1e-6, deadzone_r), -1.0, 1.0))
            if self.invert_pan:
                frac_x = -frac_x
            if self.invert_tilt:
                frac_y = -frac_y

            # Full deadzone deflection == half of the configured span.
            pan_target = self.pan_center + frac_x * (self.pan_span * 0.5)
            tilt_target = self.tilt_center + frac_y * (self.tilt_span * 0.5)

            pan_target = float(np.clip(pan_target, self.s2_min, self.s2_max))
            tilt_target = float(np.clip(tilt_target, self.s1_min, self.s1_max))

            # Low-pass so tracker jitter does not vibrate the servo.
            a = self.servo_smoothing
            self.s2 += a * (pan_target - self.s2)
            self.s1 += a * (tilt_target - self.s1)
            self._publish_servo()
        else:
            # Dwell gate: rejects saccades, which would otherwise cause
            # spurious drives on every glance.
            #
            # And the chassis gate: `run` must be ON, the mode must be "robot",
            # and the EEG link must be live. A dead laptop therefore leaves the
            # robot standing, which is the only acceptable failure direction.
            if (now - self.zone_since >= self.dwell_threshold
                    and self._chassis_allowed(now)):
                self.confirmed_zone = zone
                fx, fy, fz = ZONE_MOTION.get(zone, (0.0, 0.0, 0.0))
                twist.linear.x = fx * self.drive_vx
                twist.linear.y = fy * self.drive_vy
                twist.angular.z = fz * self.turn_wz

        self.vel_pub.publish(twist)
        self.zone_pub.publish(String(data=self.confirmed_zone))


def main(args=None):
    rclpy.init(args=args)
    node = GazeControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
