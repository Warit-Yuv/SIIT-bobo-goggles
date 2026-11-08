#!/usr/bin/env python3
"""EEG state bridge: Muse 2 gesture toggles -> ROS state topics.

Companion to github.com/koonzeerkung/Muse2-RosmasterPi-A1 (protocol v1).

    laptop --UDP:9000--> eeg_udp_bridge --> /eeg/run, /eeg/mode, /eeg/music
                                            |
                                     gaze_control_node
                                            |
                                      /cmd_vel/gaze --> arbiter --> motors

WHY THIS IS NOT A VELOCITY BRIDGE ANY MORE
------------------------------------------
The previous version mapped GO/STOP/LEFT/RIGHT onto /cmd_vel/eeg, so the EEG
side had to hold a direction for as long as the robot should keep moving. That
is a bad fit for a headband: the classifier emits discrete events, not a
continuous signal, and every missed or late packet is a wrong movement.

Protocol v1 replaced it with three TOGGLES and a state block:

    JAW_CLENCH     -> TOGGLE_RUN    run:  false <-> true
    DOUBLE_BLINK   -> TOGGLE_MODE   mode: "robot" <-> "camera"
    EYEBROW_RAISE  -> TOGGLE_MUSIC  music: false <-> true

Every message - event or heartbeat - carries the FULL state after it. So this
bridge follows `state` rather than flipping its own copy on each event. A lost
UDP packet is then corrected by the next heartbeat within 0.5 s, and the two
sides cannot drift apart. Reacting to `event.command` instead would be simpler
and would silently desynchronise the moment one packet goes missing.

WHAT THE STATE MEANS HERE
-------------------------
    run   false  nothing moves; gaze only aims the camera
          true   the robot moves according to `mode`
    mode  robot  gaze steers the CHASSIS
          camera gaze steers the PTZ head

So a jaw clench is "go / stop" and a double blink is "steer the robot or look
around" - which is exactly the split that makes gaze usable without needing a
reliable vertical axis.

Safety
------
* Link watchdog: nothing for `link_timeout` (1.5 s per the protocol) and `run`
  is forced false. The EEG laptop being asleep must never leave the robot
  driving.
* `seq` goes backwards only when the sender restarted, so a lower sequence
  number is ignored unless the link had already dropped.
* Nothing here publishes /cmd_vel. The arbiter is the only /cmd_vel publisher,
  so EEG can never fight the keyboard or the gaze controller.

Run (inside the container, with the Yahboom workspace sourced):
    ros2 run m1_control eeg_udp_bridge
    ros2 run m1_control eeg_udp_bridge --ros-args -p port:=9000

Dry run (no ROS needed - prints the state it would publish):
    python3 eeg_udp_bridge.py --dry-run
"""

import argparse
import json
import socket
import threading
import time

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

PROTOCOL_VERSION = 1

#: The only commands protocol v1 defines. Anything else is ignored rather than
#: guessed at: every one of these is a toggle, so acting on a misread would
#: flip the wrong thing.
TOGGLE_RUN = 'TOGGLE_RUN'
TOGGLE_MODE = 'TOGGLE_MODE'
TOGGLE_MUSIC = 'TOGGLE_MUSIC'

MODE_ROBOT = 'robot'
MODE_CAMERA = 'camera'

#: Gesture -> command, so an event carrying only a gesture still works. The
#: sender always includes `command`, but a hand-typed test packet may not.
GESTURE_COMMANDS = {
    'JAW_CLENCH': TOGGLE_RUN,
    'DOUBLE_BLINK': TOGGLE_MODE,
    'EYEBROW_RAISE': TOGGLE_MUSIC,
}


# ---------------------------------------------------------------------------
# Pure logic (no ROS) so it can be unit tested and dry-run on any machine.
# ---------------------------------------------------------------------------

class EEGCore:
    """Holds the EEG-side state, following the sender's `state` block."""

    def __init__(self, link_timeout=1.5, start_run=False,
                 start_mode=MODE_ROBOT, auto_stop_after=0.0):
        self.link_timeout = float(link_timeout)
        self.auto_stop_after = float(auto_stop_after)

        self.run = bool(start_run)
        self.mode = start_mode if start_mode in (MODE_ROBOT, MODE_CAMERA) else MODE_ROBOT
        self.music = False

        self.link_ok = False
        self.last_msg = 0.0
        self.last_seq = None
        self.msg_count = 0
        self.seq_gaps = 0
        self.dropped = 0
        self.bad_version = None
        self.last_gesture = ''
        self.last_command = ''
        self._run_since = 0.0

        self.lock = threading.Lock()

    # -- input ------------------------------------------------------------
    def on_message(self, msg, now):
        """Apply one decoded packet. Returns a short description, or None.

        Follows `state` when present, because that is the whole point of
        protocol v1: the sender repeats the full state in every heartbeat, so
        a lost packet is self-healing.
        """
        if not isinstance(msg, dict):
            with self.lock:
                self.dropped += 1
            return None

        # A sender speaking a different protocol version must be rejected and
        # COUNTED. Silently interpreting v0 as v1 gives a bridge that looks
        # alive, reports a link, and toggles nothing - and there is no counter
        # anywhere that would tell the operator why.
        ver = msg.get('v')
        if isinstance(ver, int) and ver != PROTOCOL_VERSION:
            with self.lock:
                self.dropped += 1
                self.bad_version = ver
            return None

        with self.lock:
            was_ok = self.link_ok
            self.last_msg = now
            self.link_ok = True
            self.msg_count += 1

            # A lower `seq` means the sender restarted. That is only believable
            # if the link had already dropped - otherwise it is a late or
            # duplicated datagram and must be ignored.
            seq = msg.get('seq')
            if isinstance(seq, int):
                if self.last_seq is not None:
                    if seq < self.last_seq and was_ok:
                        self.dropped += 1
                        return None
                    if seq > self.last_seq + 1 and was_ok:
                        self.seq_gaps += seq - self.last_seq - 1
                self.last_seq = seq

            note = None
            ev = msg.get('event')
            if isinstance(ev, dict):
                gesture = str(ev.get('gesture', '')).upper()
                command = str(ev.get('command', '')).upper()
                self.last_gesture = gesture
                if not command:
                    command = GESTURE_COMMANDS.get(gesture, '')
                self.last_command = command
                note = '%s -> %s' % (gesture or '?', command or '?')

                # Fallback: if the sender did not include a state block, apply
                # the toggle ourselves. Only reached for packets that are not
                # protocol v1, e.g. a hand-typed test datagram.
                if not isinstance(msg.get('state'), dict):
                    self._apply_toggle(command, now)

            state = msg.get('state')
            if isinstance(state, dict):
                if 'run' in state:
                    new_run = bool(state['run'])
                    if new_run and not self.run:
                        self._run_since = now
                    self.run = new_run
                if 'mode' in state:
                    m = str(state['mode']).lower()
                    if m in (MODE_ROBOT, MODE_CAMERA):
                        self.mode = m
                if 'music' in state:
                    self.music = bool(state['music'])

            return note

    def _apply_toggle(self, command, now):
        if command == TOGGLE_RUN:
            self.run = not self.run
            if self.run:
                self._run_since = now
        elif command == TOGGLE_MODE:
            self.mode = MODE_CAMERA if self.mode == MODE_ROBOT else MODE_ROBOT
        elif command == TOGGLE_MUSIC:
            self.music = not self.music

    # -- output -----------------------------------------------------------
    def tick(self, now):
        """Apply the watchdogs. Returns True if `run` was forced off."""
        with self.lock:
            changed = False
            if self.link_ok and now - self.last_msg > self.link_timeout:
                self.link_ok = False
                if self.run:
                    self.run = False
                    changed = True

            # Optional dead-man timer. Off by default: a toggle that silently
            # expires is surprising, and the link watchdog already covers the
            # real hazard (the EEG side going away).
            if (self.auto_stop_after and self.run
                    and now - self._run_since > self.auto_stop_after):
                self.run = False
                changed = True
            return changed

    def status(self):
        with self.lock:
            return {
                'run': self.run,
                'mode': self.mode,
                'music': self.music,
                'link': self.link_ok,
                'msgs': self.msg_count,
                'gaps': self.seq_gaps,
                'dropped': self.dropped,
                'last': '%s/%s' % (self.last_gesture, self.last_command),
            }


def udp_loop(core, port, stop_event, log=print):
    """Receive protocol v1 packets, ack each one, and log gestures."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(0.1)
    log(f'[eeg] listening on UDP :{port}')
    peer = None
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        now = time.monotonic()
        try:
            msg = json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            # Not JSON. Ignore rather than guessing: every command is a
            # toggle, so acting on a malformed packet could start the robot.
            continue
        if not isinstance(msg, dict):
            continue

        if addr != peer:
            log(f'[eeg] EEG side connected from {addr[0]}:{addr[1]}')
            peer = addr

        note = core.on_message(msg, now)
        if note:
            st = core.status()
            log(f"[eeg] {note}   run={'ON' if st['run'] else 'off'} mode={st['mode']}")

        # The protocol says any reply is taken as "receiver alive". Sending the
        # state back is more useful than a bare ack and costs nothing.
        try:
            sock.sendto(json.dumps({'type': 'ack',
                                    'state': core.status()}).encode(), addr)
        except OSError:
            pass
    sock.close()


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

def run_ros(args, core):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import Bool, String

    class EEGBridge(Node):

        def __init__(self):
            super().__init__('eeg_udp_bridge')

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1)

            # Latched: a late subscriber must see the current state, not wait
            # for the next change. These are toggles, so "the next change" may
            # be minutes away.
            self.run_pub = self.create_publisher(Bool, '/eeg/run', qos)
            self.mode_pub = self.create_publisher(String, '/eeg/mode', qos)
            self.music_pub = self.create_publisher(Bool, '/eeg/music', qos)
            self.link_pub = self.create_publisher(Bool, '/eeg/link', qos)

            self._prev = None
            self.create_timer(1.0 / args.rate, self.tick)
            self.get_logger().info(
                f'EEG bridge (protocol v{PROTOCOL_VERSION}) | UDP :{args.port} | '
                f'link timeout {core.link_timeout}s | '
                f'jaw=TOGGLE_RUN  blink=TOGGLE_MODE  brow=TOGGLE_MUSIC')

        def tick(self):
            now = time.monotonic()
            if core.tick(now):
                self.get_logger().warn(
                    'EEG link lost - run forced OFF')

            st = core.status()
            cur = (st['run'], st['mode'], st['music'], st['link'])
            if cur != self._prev:
                self._prev = cur
                self.run_pub.publish(Bool(data=st['run']))
                self.mode_pub.publish(String(data=st['mode']))
                self.music_pub.publish(Bool(data=st['music']))
                self.link_pub.publish(Bool(data=st['link']))
                self.get_logger().info(
                    f"EEG state  run={'ON' if st['run'] else 'off'}  "
                    f"mode={st['mode']}  music={'on' if st['music'] else 'off'}  "
                    f"link={'ok' if st['link'] else 'LOST'}")

    rclpy.init()
    node = EEGBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leaving `run` true after this process dies would be the worst
        # possible last message.
        try:
            node.run_pub.publish(Bool(data=False))
            node.link_pub.publish(Bool(data=False))
        except Exception:  # noqa: BLE001 - context may already be gone
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--link-timeout', type=float, default=1.5)
    ap.add_argument('--start-run', action='store_true',
                    help='start with run=true (default false)')
    ap.add_argument('--start-mode', default=MODE_ROBOT,
                    choices=[MODE_ROBOT, MODE_CAMERA])
    ap.add_argument('--auto-stop', type=float, default=0.0,
                    help='force run=false after N seconds of running (0 = off)')
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--dry-run', action='store_true')
    args, _ = ap.parse_known_args(argv)

    core = EEGCore(link_timeout=args.link_timeout, start_run=args.start_run,
                   start_mode=args.start_mode, auto_stop_after=args.auto_stop)

    stop_event = threading.Event()
    threading.Thread(target=udp_loop, args=(core, args.port, stop_event),
                     daemon=True).start()

    if args.dry_run:
        print('[eeg] dry-run: no ROS, printing state only (Ctrl+C to stop)')
        last = None
        try:
            while True:
                core.tick(time.monotonic())
                st = core.status()
                cur = (st['run'], st['mode'], st['music'], st['link'])
                if cur != last:
                    print(f"[dry-run] run={'ON' if st['run'] else 'off'} "
                          f"mode={st['mode']} music={st['music']} "
                          f"link={st['link']} msgs={st['msgs']}")
                    last = cur
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            stop_event.set()
        return

    run_ros(args, core)


if __name__ == '__main__':
    main()
