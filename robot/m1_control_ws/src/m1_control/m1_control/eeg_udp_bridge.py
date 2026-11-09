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
    EYEBROW_RAISE  -> TOGGLE_MUSIC  music: false <-> true  (sound effect)

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
#:
#: Assigned by how easily the operator can PERFORM each gesture, which is the
#: property that matters when they have to do it many times per session:
#:
#:   JAW_CLENCH     the easiest to produce deliberately and the least tiring,
#:                  so it gets the job used most often - go / stop
#:   DOUBLE_BLINK   reliable, but takes more effort and is more tiring to
#:                  repeat, so it gets the less frequent job - switching what
#:                  gaze steers
#:   EYEBROW_RAISE  the least reliable of the three (the classifier reports
#:                  UNCLEAR_MUSCLE when it cannot separate it from a clench),
#:                  so it gets the only job where being wrong costs nothing at
#:                  all: a sound effect
#:
#: This ordering came from the operator actually using it. An earlier version
#: put the blink on the run toggle on the assumption that it was the most
#: CONSISTENT gesture; in practice the clench is the easier one to perform, and
#: frequency of use matters more than raw accuracy for a toggle that has to be
#: hit several times a minute.
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
        #: Increments on every gesture the sender reports, INCLUDING one that
        #: toggles a value back to where it already was. Without a counter the
        #: only way to notice a gesture is to see the state change, and a
        #: gesture that produced no change (or was rejected) would be
        #: invisible - which is exactly the case where the operator most wants
        #: to know whether the headset saw them.
        self.gesture_seq = 0
        #: True while a keyboard gesture outranks the sender's state.
        #:
        #: Protocol v1 makes the SENDER authoritative: every packet repeats the
        #: full state so a lost datagram heals itself. That is right for the
        #: headset and wrong for a keyboard press, because the sender knows
        #: nothing about it - so the very next heartbeat, 100 ms later, puts
        #: the state straight back and the keypress appears to do nothing.
        #:
        #: Measured: a keyboard blink set mode=camera and the sender's next
        #: packet reverted it to robot 0.2 s later, which looks exactly like
        #: the key being broken.
        #:
        #: The first fix was a five-second window, which is the wrong trigger.
        #: A connected-but-idle headset sends the SAME state forever, so every
        #: keypress was silently undone five seconds later - long after the
        #: operator had stopped watching the screen and started believing the
        #: key had worked. The question is not "how long ago did the key go
        #: down" but "has the headset said anything NEW since".
        #:
        #: So the override lasts until the sender's own state CHANGES. A
        #: repeating packet is a heartbeat and is ignored; a packet that
        #: differs from the sender's last report is a real headset action and
        #: takes over immediately. The headset can never be locked out, because
        #: the gesture that would take it back is by definition a change.
        self._kb_pending = False
        #: The sender's last reported state, or None if it has not spoken yet.
        #: A heartbeat is compared against this rather than against the live
        #: state, which the keyboard may have moved.
        self._sender_state = None
        #: How many state packets a keyboard override suppressed, so the
        #: bridge can say so once instead of silently ignoring input.
        self.kb_suppressed = 0

        #: False until the first state block has been adopted. The very first
        #: packet always differs from the bridge's starting defaults, and
        #: counting that as a gesture would make the GUI announce a blink
        #: nobody performed every time the bridge or the sender restarts.
        self._state_seen = False
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
            gesture = ''
            command = ''

            # A gesture can arrive three ways. All three are accepted because
            # the sender is not ours to change, and a sender that reports its
            # gesture in a shape this bridge ignores produces a system that
            # looks connected and does nothing - the worst failure mode here.
            ev = msg.get('event')
            if isinstance(ev, dict):
                gesture = str(ev.get('gesture', '')).upper()
                command = str(ev.get('command', '')).upper()
            elif msg.get('gesture'):
                # Top-level form: {"gesture": "JAW_CLENCH"}
                gesture = str(msg.get('gesture', '')).upper()
                command = str(msg.get('command', '')).upper()

            prev = (self.run, self.mode, self.music)

            if gesture:
                if not command:
                    command = GESTURE_COMMANDS.get(gesture, '')
                self.last_gesture = gesture
                self.last_command = command
                self.gesture_seq += 1
                note = '%s -> %s' % (gesture or '?', command or '?')

                # Fallback: if the sender did not include a state block, apply
                # the toggle ourselves. Only reached for packets that are not
                # protocol v1, e.g. a hand-typed test datagram.
                if not isinstance(msg.get('state'), dict):
                    self._apply_toggle(command, now)

            state = msg.get('state')
            if isinstance(state, dict):
                # Reduce the packet to the three fields the bridge cares about,
                # so a sender that adds a new key does not look like it changed
                # something every single packet.
                sender_now = {}
                if 'run' in state:
                    sender_now['run'] = bool(state['run'])
                if 'mode' in state:
                    sender_now['mode'] = str(state['mode']).lower()
                if 'music' in state:
                    sender_now['music'] = bool(state['music'])

                # A packet identical to the sender's previous report is a
                # HEARTBEAT: it exists to prove the link is alive and to heal a
                # lost datagram, not to instruct anything. If the keyboard is
                # still in force, it must not be allowed to undo a keypress -
                # and because we do NOT record it as the new baseline, the next
                # heartbeat is also ignored. The override therefore lasts
                # exactly as long as the headset stays quiet.
                if self._kb_pending and sender_now == self._sender_state:
                    self.kb_suppressed += 1
                    self._state_seen = True
                    return note

                # Either no keyboard override, or the sender genuinely moved.
                # A first packet always counts as a change: the sender has just
                # connected and knows its own state better than we do.
                self._kb_pending = False
                self._sender_state = sender_now

                if 'run' in sender_now:
                    new_run = sender_now['run']
                    if new_run and not self.run:
                        self._run_since = now
                    self.run = new_run
                if 'mode' in sender_now and sender_now['mode'] in (MODE_ROBOT,
                                                                  MODE_CAMERA):
                    self.mode = sender_now['mode']
                if 'music' in sender_now:
                    self.music = sender_now['music']

            # No gesture field at all, but the state moved: work the gesture
            # out from WHAT changed. Every toggle maps to exactly one field, so
            # the delta identifies the gesture unambiguously.
            #
            # This is not a guess. It is the only way to support a sender that
            # publishes state without labelling the event - and without it, the
            # GUI shows nothing at all for a gesture that plainly happened,
            # which is indistinguishable from the headset being disconnected.
            if not gesture and self._state_seen:
                after = (self.run, self.mode, self.music)
                if after != prev:
                    if after[0] != prev[0]:
                        self.last_gesture = 'JAW_CLENCH'
                        self.last_command = TOGGLE_RUN
                    elif after[1] != prev[1]:
                        self.last_gesture = 'DOUBLE_BLINK'
                        self.last_command = TOGGLE_MODE
                    else:
                        self.last_gesture = 'EYEBROW_RAISE'
                        self.last_command = TOGGLE_MUSIC
                    self.gesture_seq += 1
                    note = '%s -> %s (from state)' % (self.last_gesture,
                                                      self.last_command)

            if isinstance(state, dict):
                self._state_seen = True

            return note

    def inject_gesture(self, gesture, now):
        """Apply a gesture that did NOT come from the headset.

        This is the keyboard fallback: when the EEG classifier is unusable, the
        operator can still drive the same three toggles from the console. It
        goes through the SAME toggle code as a real gesture, so the two paths
        cannot drift apart - a fallback that behaves differently from the thing
        it is standing in for is not a fallback.

        Deliberately does NOT touch `link_ok`. Marking the link up would make
        the bridge's own watchdog force `run` back off one timeout later, so a
        single keypress would start the robot and then stop it again with no
        explanation. The keyboard is a local input, not a remote link.
        """
        with self.lock:
            gesture = str(gesture or '').strip().upper()
            command = GESTURE_COMMANDS.get(gesture, '')
            if not command:
                self.dropped += 1
                return None
            self.last_msg = now
            self.last_gesture = gesture
            self.last_command = command
            self.gesture_seq += 1
            self._apply_toggle(command, now)
            self._kb_pending = True
            self.kb_suppressed = 0
            return '%s -> %s (keyboard)' % (gesture, command)

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
                # The sender is gone, so it has no state to defend. Drop the
                # baseline too, so when it comes back its first packet is
                # treated as a fresh statement rather than compared against a
                # stale one - otherwise a keyboard override made while the
                # headset was unplugged would outlive the reconnect.
                self._sender_state = None
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
                'gesture_seq': self.gesture_seq,
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
            # The gesture itself, not just the state it produced. A GUI cannot
            # reliably tell a double blink from a jaw clench by watching the
            # resulting toggles - and a gesture that changes nothing is
            # invisible entirely. This topic says what the headset actually
            # saw, which is the only honest basis for "action detected".
            self.gesture_pub = self.create_publisher(
                String, '/eeg/gesture', qos)
            self.link_pub = self.create_publisher(Bool, '/eeg/link', qos)

            # Keyboard fallback. The console publishes a gesture name here and
            # this bridge applies it exactly as if the headset had reported it.
            # Routing it through the bridge rather than publishing /eeg/run
            # directly keeps ONE owner of the EEG state: two publishers on a
            # latched topic would both be "true" to a subscriber, and the last
            # writer would win in a way nobody could predict.
            self.create_subscription(String, '/eeg/simulate',
                                     self._simulate_cb, 10)

            self._prev = None
            self._prev_gesture = 0
            self._last_beat = 0.0
            self.create_timer(1.0 / args.rate, self.tick)
            self.get_logger().info(
                f'EEG bridge (protocol v{PROTOCOL_VERSION}) | UDP :{args.port} | '
                f'link timeout {core.link_timeout}s | '
                f'jaw=TOGGLE_RUN  blink=TOGGLE_MODE  brow=TOGGLE_MUSIC')

        def _simulate_cb(self, msg):
            note = core.inject_gesture(msg.data, time.monotonic())
            if note is None:
                self.get_logger().warn(
                    f'ignoring unknown simulated gesture "{msg.data}"')
            else:
                self.get_logger().info(f'[keyboard] {note}')

        def tick(self):
            now = time.monotonic()
            if core.tick(now):
                self.get_logger().warn(
                    'EEG link lost - run forced OFF')

            st = core.status()
            # Publish the gesture BEFORE the state, and independently of it, so
            # a gesture that changed nothing still shows up on the console.
            if st['gesture_seq'] != self._prev_gesture:
                self._prev_gesture = st['gesture_seq']
                if st['last'] and not st['last'].startswith('/'):
                    self.gesture_pub.publish(String(data=st['last']))
            cur = (st['run'], st['mode'], st['music'], st['link'])
            # Republish on every change, AND once a second regardless.
            #
            # The periodic repeat is not redundant. These topics are latched,
            # so a subscriber that joins later already sees the current value -
            # but a subscriber has no way to tell "the value has not changed"
            # from "the publisher has died", and those need very different
            # responses. The repeat gives them a heartbeat.
            changed = cur != self._prev
            beat = now - self._last_beat >= 1.0
            if changed or beat:
                self._last_beat = now
                self._prev = cur
                self.run_pub.publish(Bool(data=st['run']))
                self.mode_pub.publish(String(data=st['mode']))
                self.music_pub.publish(Bool(data=st['music']))
                self.link_pub.publish(Bool(data=st['link']))
            if changed:
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
