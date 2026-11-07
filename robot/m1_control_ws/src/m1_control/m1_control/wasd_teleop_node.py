#!/usr/bin/env python3
"""Keyboard teleoperation for the ROSMASTER M1 (mecanum chassis + PTZ gimbal).

Publishes to the mode-specific topics consumed by the arbiter, never to
/cmd_vel directly:

    /cmd_vel/keyboard   geometry_msgs/Twist    (vx, vy, wz)
    /servo/keyboard     geometry_msgs/Vector3  (x = pan s1, y = tilt s2)
    /control_mode       std_msgs/UInt8         (0 = E-STOP, 1/2/3 = modes)

Controls
--------
    W / S      forward / backward          (vx)
    Q / E      strafe left / right         (vy)
    A / D      rotate CCW / CW             (wz)
    I / K      camera tilt up / down
    J / L      camera pan left / right
    U          re-centre the gimbal
    SPACE      E-STOP (latched mode 0)
    1 / 2 / 3  select KEYBOARD / GAZE / EEG mode
    Ctrl+C     quit (sends a final zero velocity)

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
 Chassis (holonomic mecanum)      PTZ gimbal (2-axis)
        W                              I  tilt up
   Q    S    E          J  pan left  K  tilt down  L  pan right
   A         D                        U  re-centre
   W/S forward/back   Q/E strafe
   A/D rotate CCW/CW

 SPACE  E-STOP (latched)     1/2/3  KEYBOARD / GAZE / EEG
 Ctrl+C quit
---------------------------------------------------------------
"""


class WASDTeleopNode(Node):

    def __init__(self):
        super().__init__('wasd_teleop_node')

        self.declare_parameter('speed_trans', 0.20)
        self.declare_parameter('speed_strafe', 0.20)
        self.declare_parameter('speed_rot', 0.60)
        self.declare_parameter('servo_step', 5.0)
        self.declare_parameter('servo_s1_center', 40.0)
        self.declare_parameter('servo_s2_center', 90.0)
        self.declare_parameter('servo_s1_min', 0.0)
        self.declare_parameter('servo_s1_max', 180.0)
        self.declare_parameter('servo_s2_min', 0.0)
        self.declare_parameter('servo_s2_max', 110.0)
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
        """Return False when the loop should exit."""
        if key in ('\x03', 'q'):
            return False

        self.vel = Twist()  # each key event re-derives the setpoint

        if key == 'w':
            self.vel.linear.x = self.speed_trans
        elif key == 's':
            self.vel.linear.x = -self.speed_trans
        elif key == 'q':
            self.vel.linear.y = self.speed_strafe
        elif key == 'e':
            self.vel.linear.y = -self.speed_strafe
        elif key == 'a':
            self.vel.angular.z = self.speed_rot
        elif key == 'd':
            self.vel.angular.z = -self.speed_rot
        elif key == 'i':
            self.s2 = min(self.s2_max, self.s2 + self.servo_step)
            self.servo_dirty = True
        elif key == 'k':
            self.s2 = max(self.s2_min, self.s2 - self.servo_step)
            self.servo_dirty = True
        elif key == 'j':
            self.s1 = min(self.s1_max, self.s1 + self.servo_step)
            self.servo_dirty = True
        elif key == 'l':
            self.s1 = max(self.s1_min, self.s1 - self.servo_step)
            self.servo_dirty = True
        elif key == 'u':
            self.s1 = float(self.get_parameter('servo_s1_center').value)
            self.s2 = float(self.get_parameter('servo_s2_center').value)
            self.servo_dirty = True
        elif key == ' ':
            self.mode_pub.publish(UInt8(data=0))
            self.vel = Twist()
            print('\n[E-STOP] latched. Press 1/2/3 to resume.\n')
            return True
        elif key in ('1', '2', '3'):
            self.mode_pub.publish(UInt8(data=int(key)))
            self.vel = Twist()
            print(f'\n[INFO] mode -> {key}\n')
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
                    self.servo_pub.publish(
                        Vector3(x=float(self.s1), y=float(self.s2), z=0.0)
                    )
                    self.servo_dirty = False
        finally:
            self.vel_pub.publish(Twist())


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
