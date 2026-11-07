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

        self.declare_parameter('drive_speed_x', 0.20)
        self.declare_parameter('drive_speed_y', 0.20)
        self.declare_parameter('turn_speed_z', 0.60)

        self.declare_parameter('servo_gain_pan', 1.5)
        self.declare_parameter('servo_gain_tilt', 1.5)
        self.declare_parameter('servo_rate_limit_hz', 12.0)
        self.declare_parameter('servo_s1_center', 40.0)
        self.declare_parameter('servo_s2_center', 90.0)
        self.declare_parameter('servo_s1_min', 0.0)
        self.declare_parameter('servo_s1_max', 180.0)
        self.declare_parameter('servo_s2_min', 0.0)
        self.declare_parameter('servo_s2_max', 110.0)

        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('assert_mode', True)
        self.declare_parameter('publish_rate_hz', 20.0)

        gp = self.get_parameter
        self.input_normalized = bool(gp('input_is_normalized').value)
        self.link_timeout = float(gp('gaze_link_timeout_sec').value)
        self.dwell_threshold = float(gp('dwell_time_sec').value)
        self.deadzone_ratio = float(gp('deadzone_radius_ratio').value)
        self.enable_strafe = bool(gp('enable_strafe').value)
        self.drive_vx = float(gp('drive_speed_x').value)
        self.drive_vy = float(gp('drive_speed_y').value)
        self.turn_wz = float(gp('turn_speed_z').value)
        self.gain_pan = float(gp('servo_gain_pan').value)
        self.gain_tilt = float(gp('servo_gain_tilt').value)
        self.invert_pan = bool(gp('invert_pan').value)
        self.invert_tilt = bool(gp('invert_tilt').value)
        self.assert_mode = bool(gp('assert_mode').value)

        # Raw servo positions, named as the message/hardware name them.
        #   s1 = TILT (larger = up)      s2 = PAN (larger = left)
        self.s1 = float(gp('servo_s1_center').value)      # tilt
        self.s2 = float(gp('servo_s2_center').value)      # pan
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

    # ------------------------------------------------------------- geometry
    def _classify(self, dx, dy, deadzone_r):
        """Map an offset from centre to a control zone.

        `dx` is +right, `dy` is +down (image convention). The angle is measured
        from "straight up the screen" so that 0 deg means FORWARD, +90 deg means
        screen-right, which keeps the table readable.
        """
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
            # Fine PTZ aiming. Scale by the deadzone radius so full deflection
            # at the deadzone edge gives the maximum step.
            #
            # Axis mapping: gaze is in image coordinates (+dx = right,
            # +dy = down). Servo s1 = tilt (larger = up), s2 = pan (larger =
            # left). So both steps are negated:
            #   looking left  (dx < 0) -> s2 increases -> pan left
            #   looking up    (dy < 0) -> s1 increases -> tilt up
            pan_step = -(dx / max(1e-6, deadzone_r)) * self.gain_pan
            tilt_step = -(dy / max(1e-6, deadzone_r)) * self.gain_tilt
            if self.invert_pan:
                pan_step = -pan_step
            if self.invert_tilt:
                tilt_step = -tilt_step
            # pan drives s2, tilt drives s1
            self.s2 = float(np.clip(self.s2 + pan_step, self.s2_min, self.s2_max))
            self.s1 = float(np.clip(self.s1 + tilt_step, self.s1_min, self.s1_max))
            self._publish_servo()
        else:
            # Dwell gate: rejects saccades, which would otherwise cause
            # spurious drives on every glance.
            if now - self.zone_since >= self.dwell_threshold:
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
