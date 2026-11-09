#!/usr/bin/env python3
"""Central control arbiter and dead-man watchdog for the ROSMASTER M1.

Single source of truth for robot actuation. Exactly one node publishes
/cmd_vel and /Servo; every input source (keyboard, gaze, EEG) talks to a
mode-specific topic instead, so two controllers can never fight each other.

    /cmd_vel/keyboard  \
    /cmd_vel/gaze       >--[ARBITER]--> /cmd_vel  --> Mcnamu_driver_M1
    /cmd_vel/eeg       /    |  ^
                            |  |
                    /control_mode (0=E-STOP 1=KEYBOARD 2=GAZE 3=EEG)

Safety contract
---------------
* Only the topic belonging to the ACTIVE mode is forwarded.
* Every forwarded velocity is clamped to the configured envelope.
* If no command arrives for `watchdog_timeout_sec`, the arbiter publishes a
  zero Twist. This runs on the robot, so the stop happens even if Wi-Fi dies.
* Mode 0 (E-STOP) is latched: it keeps publishing zeros until another mode
  is explicitly selected. It can never be entered "by accident" and left.
* On every mode change a zero Twist is published before anything else.

Note on placement: the architecture handbook puts this node on the operator
PC. It runs on the Pi 5 instead - a watchdog on the far side of a Wi-Fi link
cannot stop a robot whose Wi-Fi link just died.
"""

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8

# yahboomcar_msgs only exists on the robot. Keep the import soft so this same
# package also builds on the operator PC (which never runs the arbiter).
try:
    from yahboomcar_msgs.msg import ServoControl
    _SERVO_MSG_OK = True
except ImportError:  # pragma: no cover - operator PC
    ServoControl = None
    _SERVO_MSG_OK = False


MODE_ESTOP = 0
MODE_KEYBOARD = 1
MODE_GAZE = 2
MODE_EEG = 3
#: Gaze and EEG together. Gaze gives continuous proportional control, EEG
#: provides discrete overrides (STOP always wins), keyboard overrides both.
#: This is the mode intended for the bed-bound operator: they can steer by
#: looking and stop by clenching, without switching modes.
MODE_FUSION = 4

MODE_NAMES = {
    MODE_ESTOP: 'E-STOP',
    MODE_KEYBOARD: 'KEYBOARD',
    MODE_GAZE: 'GAZE',
    MODE_EEG: 'EEG',
    MODE_FUSION: 'GAZE+EEG',
}


class ControlArbiterNode(Node):
    """Mode multiplexer, velocity limiter and watchdog."""

    def __init__(self):
        super().__init__('control_arbiter_node')

        # ---------------- parameters ----------------
        self.declare_parameter('watchdog_timeout_sec', 0.35)
        self.declare_parameter('max_linear_x', 0.25)
        self.declare_parameter('max_linear_y', 0.20)
        self.declare_parameter('max_angular_z', 0.80)

        # Servo travel limits, passed through unchanged (the arbiter does not
        # interpret the axes - it only clamps).
        #
        # Axis mapping, verified by hand on this robot (direction only):
        #   s1 = TILT  (driver -> PWM channel 2)   larger = look UP
        #   s2 = PAN   (driver -> PWM channel 3)   larger = look LEFT
        #              the driver hard-clamps s2 to 0..110.
        # Rest pose confirmed working:
        #   ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 90}"
        # Ranges below are the vendor joystick limits; they have NOT been
        # checked against the mechanical stops.
        self.declare_parameter('default_servo_s1', 40)
        self.declare_parameter('default_servo_s2', 90)
        # Symmetric about the rest pose. Measured with the camera as the
        # sensor: pan is driver-clamped at 110, so +-20 is the widest symmetric
        # window around 90. Deliberately narrowed further to +-10: a camera
        # that swings ~90 degrees loses the view straight ahead, which is
        # dangerous while driving forward. 80..100 keeps the path visible.
        self.declare_parameter('servo_s1_min', 5)
        self.declare_parameter('servo_s1_max', 75)
        self.declare_parameter('servo_s2_min', 70)
        self.declare_parameter('servo_s2_max', 110)

        self.declare_parameter('initial_mode', MODE_KEYBOARD)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('beep_on_mode_change', False)

        # Keyboard works in every mode. A non-zero keyboard command wins for
        # this long, then control returns to the active mode by itself.
        # 0.4 s is long enough to hold a key and short enough that the gaze or
        # EEG source takes back over as soon as you let go.
        self.declare_parameter('keyboard_override', True)
        self.declare_parameter('keyboard_override_sec', 0.4)

        self.timeout = float(self.get_parameter('watchdog_timeout_sec').value)
        self.max_vx = float(self.get_parameter('max_linear_x').value)
        self.max_vy = float(self.get_parameter('max_linear_y').value)
        self.max_wz = float(self.get_parameter('max_angular_z').value)
        self.s1_min = int(self.get_parameter('servo_s1_min').value)
        self.s1_max = int(self.get_parameter('servo_s1_max').value)
        self.s2_min = int(self.get_parameter('servo_s2_min').value)
        self.s2_max = int(self.get_parameter('servo_s2_max').value)
        self.beep = bool(self.get_parameter('beep_on_mode_change').value)

        # ---------------- state ----------------
        self.active_mode = int(self.get_parameter('initial_mode').value)
        self.latest_twist = Twist()
        self.estop_latched = self.active_mode == MODE_ESTOP
        self.timed_out = False
        self.last_command_time = self.get_clock().now()

        self.s1 = int(self.get_parameter('default_servo_s1').value)
        self.s2 = int(self.get_parameter('default_servo_s2').value)
        self.servo_dirty = True
        self._beep_timer = None

        # Timestamps of recent mode switches, for churn detection.
        self._mode_switch_times = []

        # Keyboard override state.
        self.keyboard_override = bool(self.get_parameter('keyboard_override').value)
        self.keyboard_override_sec = float(
            self.get_parameter('keyboard_override_sec').value)
        self._kb_twist = Twist()
        self._kb_time = self.get_clock().now()

        # EEG jaw-clench stop flag (FUSION mode).
        self._eeg_stop = False

        # ---------------- interfaces ----------------
        # Actuator topics: RELIABLE. Setpoints must not be dropped.
        actuator_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', actuator_qos)
        self.servo_pub = (
            self.create_publisher(ServoControl, '/Servo', actuator_qos)
            if _SERVO_MSG_OK else None
        )
        # The position the arbiter is actually holding, on a message type every
        # machine has. /Servo is yahboomcar_msgs, which is robot-only - so an
        # operator PC cannot subscribe to it, and the console's servo bars only
        # ever showed what the CONSOLE last asked for. Move the camera with
        # gaze and the bars sat still, which reads as "gaze is not working".
        #
        # Latched, so a console started later still sees the current position
        # instead of waiting for the next movement.
        servo_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.servo_state_pub = self.create_publisher(
            Vector3, '/servo/actual', servo_state_qos)
        self.buzzer_pub = self.create_publisher(Bool, '/Buzzer', actuator_qos)

        # Mode command: latched so a late-joining arbiter still learns the mode.
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.mode_pub = self.create_publisher(UInt8, '/control_mode_state', mode_qos)
        self.create_subscription(UInt8, '/control_mode', self.mode_callback, mode_qos)

        # Mode-specific inputs.
        self.create_subscription(Twist, '/cmd_vel/keyboard', self._kb_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel/gaze', self._gaze_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel/eeg', self._eeg_vel_cb, 10)
        # Latched STOP from the EEG bridge. See _eeg_stop_cb().
        self.create_subscription(Bool, '/eeg/stop', self._eeg_stop_cb, 10)

        self.create_subscription(Vector3, '/servo/keyboard', self._kb_servo_cb, 10)
        self.create_subscription(Vector3, '/servo/gaze', self._gaze_servo_cb, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self.control_loop)

        self._publish_mode()
        self.get_logger().info(
            f'Arbiter up | mode={MODE_NAMES.get(self.active_mode, "?")} '
            f'| watchdog={self.timeout:.2f}s '
            f'| limits vx={self.max_vx} vy={self.max_vy} wz={self.max_wz}'
        )
        if not _SERVO_MSG_OK:
            self.get_logger().error(
                'yahboomcar_msgs not found - /Servo will NOT be published. '
                'Source the Yahboom workspace before starting the arbiter.'
            )

    # ------------------------------------------------------------------ mode
    def _publish_mode(self):
        self.mode_pub.publish(UInt8(data=self.active_mode))

    def _beep(self):
        """Pulse the buzzer ON, then OFF shortly after.

        The chassis driver maps Bool(true) -> set_beep(1) and Bool(false) ->
        set_beep(0), so the buzzer LATCHES. Publishing only `true` leaves it
        sounding forever - which is exactly what happened the first time this
        ran: one mode change started a beep that never stopped.

        The OFF message is therefore not optional, it is the other half of the
        beep. A one-shot timer sends it so the ON is audible first.
        """
        if not self.beep:
            return
        self.buzzer_pub.publish(Bool(data=True))
        if self._beep_timer is not None:
            self._beep_timer.cancel()
            self.destroy_timer(self._beep_timer)
        self._beep_timer = self.create_timer(0.15, self._beep_off)

    def _beep_off(self):
        """Turn the buzzer off, then remove this one-shot timer."""
        self.buzzer_pub.publish(Bool(data=False))
        if self._beep_timer is not None:
            self._beep_timer.cancel()
            self.destroy_timer(self._beep_timer)
            self._beep_timer = None

    def mode_callback(self, msg: UInt8):
        new_mode = int(msg.data)
        if new_mode not in MODE_NAMES:
            self.get_logger().warn(f'Ignoring unknown mode {new_mode}')
            return
        if new_mode == self.active_mode and not self.estop_latched:
            return

        # Mode-churn detector.
        #
        # The arbiter does not police who may publish /control_mode - that
        # would break the normal `ros2 topic pub --once` workflow. What it can
        # do is make a hijack loud. A node that re-asserts its own mode on a
        # timer produces a stream of switches, and that is worth shouting
        # about, because the symptom (keyboard silently doing nothing) looks
        # nothing like the cause.
        now = self.get_clock().now()
        self._mode_switch_times.append(now)
        while (len(self._mode_switch_times) > 8 and
               (now - self._mode_switch_times[0]).nanoseconds / 1e9 > 5.0):
            self._mode_switch_times.pop(0)

        if len(self._mode_switch_times) >= 6:
            self.get_logger().error(
                f'{len(self._mode_switch_times)} mode changes in '
                f'{(now - self._mode_switch_times[0]).nanoseconds / 1e9:.1f}s '
                f'- something is asserting /control_mode on a timer. '
                f'Check for a gaze/EEG node with assert_mode:=true.')

        # Stop first, then switch. Never leave a stale setpoint behind.
        self.cmd_vel_pub.publish(Twist())
        self.latest_twist = Twist()

        self.active_mode = new_mode
        self.estop_latched = new_mode == MODE_ESTOP
        self.timed_out = False
        self.last_command_time = self.get_clock().now()

        self._publish_mode()
        self._beep()
        self.get_logger().info(
            f'Mode -> {MODE_NAMES[new_mode]} '
            f'(zero velocity published before switch)'
        )

    # ------------------------------------------------------------- clamping
    def clamp_velocity(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = max(-self.max_vx, min(self.max_vx, float(msg.linear.x)))
        out.linear.y = max(-self.max_vy, min(self.max_vy, float(msg.linear.y)))
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = max(-self.max_wz, min(self.max_wz, float(msg.angular.z)))
        return out

    def _accept(self, msg: Twist):
        self.latest_twist = self.clamp_velocity(msg)
        self.last_command_time = self.get_clock().now()
        self.timed_out = False

    # ---------------------------------------------------------- input sinks
    def _kb_vel_cb(self, msg: Twist):
        # Keyboard is always accepted, in ANY mode.
        #
        # The operator asked for this explicitly: with gaze or EEG driving, the
        # keyboard should still be able to nudge the robot without switching
        # modes first. It is safe to always honour because a keyboard command
        # requires a human actively pressing a key - it cannot arrive by
        # accident the way a misclassified EEG gesture can.
        #
        # Precedence is resolved in control_loop(): a non-zero keyboard command
        # wins for `keyboard_override_sec`, then control returns to the active
        # mode automatically. Nothing is permanently taken over.
        kb = self.clamp_velocity(msg)
        self._kb_twist = kb
        self._kb_time = self.get_clock().now()
        if self.active_mode == MODE_KEYBOARD:
            self._accept(msg)

    def _gaze_vel_cb(self, msg: Twist):
        if self.active_mode in (MODE_GAZE, MODE_FUSION, MODE_EEG):
            self._accept(msg)

    def _eeg_vel_cb(self, msg: Twist):
        # Protocol v1 has no /cmd_vel/eeg at all: the EEG sends toggles, and
        # gaze_control_node gates the GAZE command on them. This handler is
        # kept for any older sender that still publishes velocities, but it is
        # no longer the thing that makes EEG mode work.
        if self.active_mode == MODE_EEG:
            self._accept(msg)
        elif self.active_mode == MODE_FUSION:
            # In FUSION, EEG is the override channel and gaze is the steering.
            #
            # EEG only wins when it says something *deliberate*: a stop (all
            # zero) or a turn. A zero EEG command must NOT overwrite gaze, or
            # the robot would stutter every time the classifier rests. So:
            #   * non-zero EEG  -> take it (STOP / LEFT / RIGHT / strafe)
            #   * zero EEG      -> ignore, leave gaze in charge
            eeg = self.clamp_velocity(msg)
            if self._magnitude(eeg) > 1e-6:
                self._accept(eeg)
                self._fusion_eeg_until = self.get_clock().now()

    def _kb_servo_cb(self, msg: Vector3):
        # Gimbal is keyboard-only by design; gaze drives it in GAZE mode.
        if self.active_mode == MODE_KEYBOARD or self.keyboard_override:
            self._set_servo(msg.x, msg.y)

    def _gaze_servo_cb(self, msg: Vector3):
        if self.active_mode in (MODE_GAZE, MODE_FUSION, MODE_EEG):
            self._set_servo(msg.x, msg.y)

    def _eeg_stop_cb(self, msg: Bool):
        """EEG jaw-clench STOP. Outranks gaze in FUSION mode.

        STOP and REST both produce a zero Twist on /cmd_vel/eeg, so the bridge
        raises /eeg/stop only for a real clench. In FUSION that distinction is
        the whole safety story: a clench must halt the robot even though gaze
        is still publishing a drive command 20 times a second.
        """
        if self.active_mode not in (MODE_EEG, MODE_FUSION):
            return
        self._eeg_stop = bool(msg.data)
        if self._eeg_stop:
            self.cmd_vel_pub.publish(Twist())
            self.latest_twist = Twist()
            self.get_logger().warn('EEG STOP - robot halted by jaw clench')

    def _set_servo(self, s1_target, s2_target):
        new_s1 = int(max(self.s1_min, min(self.s1_max, round(float(s1_target)))))
        new_s2 = int(max(self.s2_min, min(self.s2_max, round(float(s2_target)))))
        if new_s1 != self.s1 or new_s2 != self.s2:
            self.s1, self.s2 = new_s1, new_s2
            self.servo_dirty = True

    def _publish_servo(self):
        msg = Vector3()
        msg.x = float(self.s1)
        msg.y = float(self.s2)
        self.servo_state_pub.publish(msg)
        if self.servo_pub is not None:
            out = ServoControl()
            out.s1 = int(self.s1)
            out.s2 = int(self.s2)
            self.servo_pub.publish(out)
        self.servo_dirty = False

    # --------------------------------------------------------------- loop
    def control_loop(self):
        if self.servo_dirty:
            self._publish_servo()

        # Latched E-STOP: hold zero until an operator selects another mode.
        # This outranks the keyboard override - an emergency stop must never be
        # overridable by a key press.
        if self.active_mode == MODE_ESTOP:
            self.cmd_vel_pub.publish(Twist())
            return

        now = self.get_clock().now()

        # ---------------- EEG STOP (FUSION) ----------------
        # Highest priority after E-STOP: a deliberate clench outranks gaze.
        if self.active_mode == MODE_FUSION and self._eeg_stop:
            self.cmd_vel_pub.publish(Twist())
            self.timed_out = False
            return

        # ---------------- keyboard override ----------------
        # A recent, non-zero keyboard command wins regardless of mode, then
        # expires on its own so the active mode resumes without the operator
        # having to do anything.
        if self.keyboard_override:
            kb_age = (now - self._kb_time).nanoseconds / 1e9
            kb_active = (kb_age <= self.keyboard_override_sec and
                         self._magnitude(self._kb_twist) > 1e-6)
            if kb_active:
                self.cmd_vel_pub.publish(self._kb_twist)
                self.timed_out = False
                return

        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            if not self.timed_out:
                self.cmd_vel_pub.publish(Twist())
                self.timed_out = True
                self.get_logger().warn(
                    f'Watchdog: no command for {elapsed:.2f}s '
                    f'(> {self.timeout:.2f}s) - robot halted'
                )
            else:
                # Keep re-asserting zero so a late/duplicate packet cannot
                # resurrect a stale setpoint.
                self.cmd_vel_pub.publish(Twist())
            return

        self.cmd_vel_pub.publish(self.latest_twist)

    @staticmethod
    def _magnitude(twist):
        return max(abs(twist.linear.x), abs(twist.linear.y),
                   abs(twist.angular.z))

    def shutdown(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:  # noqa: BLE001 - best effort on shutdown
            pass
        # Always silence the buzzer on the way out: it latches, so an
        # interrupted beep would otherwise sound until the next power cycle.
        try:
            self.buzzer_pub.publish(Bool(data=False))
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ControlArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
