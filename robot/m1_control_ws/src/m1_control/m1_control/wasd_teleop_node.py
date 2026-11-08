#!/usr/bin/env python3
"""Keyboard teleoperation for the ROSMASTER M1 (mecanum chassis + PTZ gimbal).

Publishes to the mode-specific topics consumed by the arbiter, never to
/cmd_vel directly:

    /cmd_vel/keyboard   geometry_msgs/Twist    (vx, vy, wz)
    /servo/keyboard     geometry_msgs/Vector3  (x = s1 = TILT, y = s2 = PAN)
    /control_mode       std_msgs/UInt8         (0 = E-STOP, 1/2/3 = modes)

SERVO AXIS MAPPING - verified by hand on this robot
---------------------------------------------------
The two servos are NOT what the labels suggest. Measured by sending single
commands and watching the head:

    s1  ->  TILT  (up / down)      0 = angled down, 30 = level, 70 = angled up
    s2  ->  PAN   (left / right)   60 = right,       90 = centre, 120 = left

So a LARGER s1 looks UP and a LARGER s2 looks LEFT. Centre pose: s1=40, s2=90.

Only the *directions* above were confirmed by observation. The travel limits
come from the vendor joystick controller (s1 0..180, s2 0..117) and from the
chassis driver, which hard-clamps s2 to 0..110. They are NOT yet confirmed
against the mechanical stops - move the servos slowly the first time and stop
before they buzz or stall.

Controls
--------
    W / S      forward / backward          (vx)
    Q / E      strafe left / right         (vy)
    A / D      rotate CCW / CW             (wz)
    I / K      camera tilt up / down       (s1)
    J / L      camera pan left / right     (s2)
    U          re-centre the gimbal
    SPACE      E-STOP (latched mode 0)
    1 / 2 / 3  select KEYBOARD / GAZE / EEG mode
    Ctrl+C     quit (sends a final zero velocity)

`q` is STRAFE LEFT, not quit. It used to be both, so the node exited the moment
you tried to strafe sideways. Quitting is Ctrl+C only.

A command stays alive for `hold_time_sec` after the last keypress and then
decays to zero, so holding a key gives smooth motion while a single tap
produces a short nudge.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import UInt8

INSTRUCTIONS = """
---------------------------------------------------------------
 ROSMASTER M1 - KEYBOARD TELEOP (mode 1)
---------------------------------------------------------------
 CHASSIS (mecanum)                 GIMBAL
        W                              I   tilt up    (s1 +)
   Q    S    E                         K   tilt down  (s1 -)
   A         D                J  pan left (s2 +)  L  pan right (s2 -)
                                      U   re-centre
   W/S  forward / back
   Q/E  strafe left / right
   A/D  rotate CCW / CW

 SPACE   E-STOP (latched - press 1 to resume)
 1 2 3   KEYBOARD / GAZE / EEG mode
 Ctrl+C  quit          (q is STRAFE, not quit)
---------------------------------------------------------------
"""


class WASDTeleopNode(Node):

    def __init__(self):
        super().__init__('wasd_teleop_node')

        self.declare_parameter('speed_trans', 0.20)
        self.declare_parameter('speed_strafe', 0.20)
        self.declare_parameter('speed_rot', 0.60)

        # s1 = TILT, s2 = PAN (see the module docstring). Limits are the
        # vendor's, not yet checked against the mechanical stops.
        self.declare_parameter('servo_step', 5.0)
        self.declare_parameter('servo_s1_center', 40.0)   # tilt, near level
        self.declare_parameter('servo_s2_center', 90.0)   # pan, straight ahead
        # Symmetric about centre (see control_arbiter_node.py for the
        # measurement). Pan 80..100 is a deliberately narrow window: a camera
        # that swings ~90 degrees loses the view straight ahead, which is
        # dangerous while driving forward.
        self.declare_parameter('servo_s2_min', 80.0)
        self.declare_parameter('servo_s2_max', 100.0)
        self.declare_parameter('hold_time_sec', 0.25)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.speed_trans = float(self.get_parameter('speed_trans').value)
        self.speed_strafe = float(self.get_parameter('speed_strafe').value)
        self.speed_rot = float(self.get_parameter('speed_rot').value)
        self.servo_step = float(self.get_parameter('servo_step').value)
        self.hold_time = float(self.get_parameter('hold_time_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.s1 = float(self.get_parameter('servo_s1_center').value)
        self.s2 = float(self.get_parameter('servo_s2_center').value)
        self.s1_min = float(self.get_parameter('servo_s1_min').value)
        self.s1_max = float(self.get_parameter('servo_s1_max').value)
        self.s2_min = float(self.get_parameter('servo_s2_min').value)
        self.s2_max = float(self.get_parameter('servo_s2_max').value)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel/keyboard', 10)
        self.servo_pub = self.create_publisher(Vector3, '/servo/keyboard', 10)
        self.mode_pub = self.create_publisher(UInt8, '/control_mode', 10)

        self.vel = Twist()
        self.last_key_time = self.get_clock().now()
        self.servo_dirty = True
        self.rate = rate

        print(INSTRUCTIONS)

    # ------------------------------------------------------------- terminal
    def _read_key(self, timeout=0.05):
        """Non-blocking single-character read, leaving the tty raw only briefly."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            key = sys.stdin.read(1) if ready else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    # ---------------------------------------------------------------- keys
    def _handle_key(self, key):
        """Return False when the loop should exit.

        `q` is deliberately NOT a quit key - it strafes left. Having it do both
        meant the node exited the instant you tried to strafe, which looked like
        the terminal "detaching" and left no obvious way back. Quit is Ctrl+C.
        """
        if key == '\x03':                      # Ctrl+C only
            return False

        self.vel = Twist()  # each key event re-derives the setpoint

        # ---- chassis ----
        if key == 'w':
            self.vel.linear.x = self.speed_trans
        elif key == 's':
            self.vel.linear.x = -self.speed_trans
        elif key == 'q':
            self.vel.linear.y = self.speed_strafe        # strafe LEFT
        elif key == 'e':
            self.vel.linear.y = -self.speed_strafe       # strafe RIGHT
        elif key == 'a':
            self.vel.angular.z = self.speed_rot
        elif key == 'd':
            self.vel.angular.z = -self.speed_rot

        # ---- gimbal: s1 = TILT (larger = up) ----
        elif key == 'i':
            self.s1 = min(self.s1_max, self.s1 + self.servo_step)
            self.servo_dirty = True
        elif key == 'k':
            self.s1 = max(self.s1_min, self.s1 - self.servo_step)
            self.servo_dirty = True

        # ---- gimbal: s2 = PAN (larger = left) ----
        elif key == 'j':
            self.s2 = min(self.s2_max, self.s2 + self.servo_step)
            self.servo_dirty = True
        elif key == 'l':
            self.s2 = max(self.s2_min, self.s2 - self.servo_step)
            self.servo_dirty = True

        elif key == 'u':
            self.s1 = float(self.get_parameter('servo_s1_center').value)
            self.s2 = float(self.get_parameter('servo_s2_center').value)
            self.servo_dirty = True

        elif key == ' ':
            self.mode_pub.publish(UInt8(data=0))
            self.vel = Twist()
            print('\n[E-STOP] latched. Press 1 to resume.\n', flush=True)
            return True
        elif key in ('1', '2', '3'):
            self.mode_pub.publish(UInt8(data=int(key)))
            self.vel = Twist()
            print(f'\n[INFO] mode -> {key}\n', flush=True)
            return True
        else:
            return True

        self.last_key_time = self.get_clock().now()
        return True

    # ---------------------------------------------------------------- loop
    def run(self):
        period = 1.0 / self.rate
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                key = self._read_key(period)
                if key and not self._handle_key(key):
                    break

                age = (self.get_clock().now() - self.last_key_time).nanoseconds / 1e9
                if age > self.hold_time:
                    self.vel = Twist()

                self.vel_pub.publish(self.vel)

                if self.servo_dirty:
                    # x = s1 = tilt, y = s2 = pan
                    self.servo_pub.publish(
                        Vector3(x=float(self.s1), y=float(self.s2), z=0.0)
                    )
                    self.servo_dirty = False
        finally:
            # Always leave the terminal usable and say how to get back in.
            self.vel_pub.publish(Twist())
            print('\n[teleop] stopped - robot halted.', flush=True)
            print('[teleop] restart with:  ./m1.sh teleop\n', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
