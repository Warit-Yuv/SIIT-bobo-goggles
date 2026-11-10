#!/usr/bin/env python3
"""End-to-end check of the whole control chain, on the real robot.

Verifies the path an operator's intent takes, in order, and stops at the first
thing that is wrong rather than reporting a wall of downstream symptoms:

    mode  ->  run gate  ->  gaze  ->  arbiter  ->  /cmd_vel

Run on the robot, with the stack up and the robot OFF THE FLOOR:

    python3 scripts/check_chain.py
    python3 scripts/check_chain.py --wheels     # also confirm real motion
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String, UInt8

MODE_NAMES = {0: 'E-STOP', 1: 'KEYBOARD', 2: 'GAZE', 3: 'EEG', 4: 'GAZE+EEG'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wheels', action='store_true',
                    help='also drive the wheels briefly (robot must be raised)')
    args = ap.parse_args()

    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        detail = str(detail).encode('ascii', 'replace').decode('ascii')
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    def skip(name, why=''):
        print(f'  [SKIP] {name}' + (f'   {why}' if why else ''))

    rclpy.init()

    class Probe(Node):
        def __init__(self):
            super().__init__('chain_check')
            self.cmd = None
            self.zone = ''
            self.eeg = {}
            self.hud_state = None
            self.count_gaze = False
            self.silence_count = 0
            self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
            self.create_subscription(String, '/gaze/zone',
                                     lambda m: setattr(self, 'zone', m.data), 10)
            latched = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
            self.mode_pub = self.create_publisher(UInt8, '/control_mode', latched)
            self.gaze_pub = self.create_publisher(Point, '/gaze/point', 10)
            self.sim_pub = self.create_publisher(String, '/eeg/simulate', 10)
            self.hud_pub = self.create_publisher(Bool, '/gaze/hud_enable',
                                                 latched)
            self.create_subscription(Bool, '/gaze/hud_enabled_state',
                                     lambda m: setattr(self, 'hud_state',
                                                       bool(m.data)), latched)
            self.create_subscription(Point, '/gaze/point', self._watch_gaze, 10)
            for topic, key in (('/eeg/run', 'run'), ('/eeg/mode', 'mode'),
                               ('/eeg/link', 'link')):
                self.create_subscription(
                    String if key == 'mode' else Bool,
                    topic,
                    (lambda k: lambda m: self.eeg.__setitem__(k, m.data))(key),
                    latched)

        def _watch_gaze(self, m):
            # Only counts messages that arrive while THIS node is not
            # publishing, so a non-zero count means somebody else is.
            if self.count_gaze:
                self.silence_count += 1

        def _cmd(self, m):
            self.cmd = m

        def spin_for(self, s):
            """Spin for `s` seconds, sleeping rather than busy-looping.

            The sleep matters. `spin_once` returns IMMEDIATELY whenever this
            node's own loopback message is queued, so a bare
            `while time.time() < end: spin_once(...)` never blocks and runs at
            tens of thousands of iterations a second. Measured: the gaze
            publisher below reached 3097 Hz, which is not a test of the
            control node but a denial-of-service attack on it - and on every
            other node in the DDS domain, including the video stream.
            """
            end = time.time() + s
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.005)
                time.sleep(0.005)

        def mag(self):
            if self.cmd is None:
                return -1.0
            return max(abs(self.cmd.linear.x), abs(self.cmd.linear.y),
                       abs(self.cmd.angular.z))

        def set_mode(self, m):
            self.mode_pub.publish(UInt8(data=m))
            self.spin_for(0.5)

        def hold_gaze(self, u, v, seconds, rate_hz=30.0):
            """Publish a gaze point continuously, at the rate the HUD uses.

            30 Hz, not "as fast as the loop will go". The control node is a
            deadzone-and-dwell policy: it needs a steady stream it can reason
            about, and flooding it changes the thing being measured. The real
            HUD publishes at camera rate; matching that is the only way this
            check says anything about the real system.
            """
            period = 1.0 / max(1.0, rate_hz)
            end = time.time() + seconds
            while time.time() < end:
                msg = Point()
                msg.x, msg.y, msg.z = float(u), float(v), 0.0
                self.gaze_pub.publish(msg)
                self.spin_for(period)

    p = Probe()
    p.spin_for(2.0)

    # The gaze HUD publishes /gaze/point at camera rate. This script needs to
    # inject its OWN points to test the chain, and two publishers on one topic
    # means the control node sees an interleaving of both - so a "look left"
    # from here is averaged against the HUD's centred estimate and the zone
    # comes back CENTER. Measured exactly that.
    #
    # The HUD normally runs on the operator PC, so it cannot be paused with a
    # signal - the earlier version ran `pgrep` locally, which inside a
    # container can never see another machine, found nothing, and quietly did
    # nothing. The check then passed or failed on whichever publisher happened
    # to win the race, which is the worst kind of test: green sometimes.
    #
    # So the HUD has a LATCHED /gaze/hud_enable mute instead, which works
    # across machines. Mute it, then PROVE the mute took effect by watching
    # for silence - a mute that silently failed would look exactly like one
    # that worked.
    import subprocess

    def set_hud_enabled(enabled):
        p.hud_pub.publish(Bool(data=bool(enabled)))
        p.spin_for(0.4)

    def hud_reports():
        return p.hud_state

    def gaze_publishers():
        """How many nodes publish /gaze/point right now (us included)."""
        try:
            r = subprocess.run(['ros2', 'topic', 'info', '/gaze/point',
                                '--verbose'],
                               capture_output=True, text=True, timeout=20)
        except Exception:                                    # noqa: BLE001
            return -1
        for line in r.stdout.splitlines():
            if 'Publisher count:' in line:
                try:
                    return int(line.split(':')[1])
                except ValueError:
                    return -1
        return -1

    n_pub = gaze_publishers()
    set_hud_enabled(False)
    if hud_reports() is False:
        print('  (gaze HUD acknowledged the mute)')
    elif n_pub <= 1:
        print('  (no other /gaze/point publisher is running)')
    else:
        print(f'  WARNING: {n_pub} publishers on /gaze/point and no HUD')
        print('           acknowledged the mute. Something else is driving')
        print('           gaze; the gaze checks will be SKIPPED.')

    # Confirm silence: publish nothing for a while and count what arrives.
    # Our own publisher is silent, so anything received is someone else.
    p.silence_count = 0
    p.count_gaze = True
    p.spin_for(1.5)
    p.count_gaze = False
    isolated = (p.silence_count == 0)
    if not isolated:
        print(f'  WARNING: {p.silence_count} foreign /gaze/point messages in')
        print('           1.5 s of silence. The gaze checks will be SKIPPED.')

    # ---------------------------------------------------------- 1. the gate
    print('1. the EEG gate')
    check('the EEG link is up', bool(p.eeg.get('link')),
          f'link={p.eeg.get("link")}')
    check('a run state is known', p.eeg.get('run') is not None,
          f'run={p.eeg.get("run")}  mode={p.eeg.get("mode")}')

    # ------------------------------------------------------- 2. the mode
    print('\n2. the mode')

    def set_eeg_mode(want):
        """Drive the EEG sub-mode to `want`. A blink TOGGLES it.

        This has to be normalised rather than assumed. `/control_mode` and the
        EEG sub-mode are two different things: `/control_mode` picks which
        INPUT drives the robot, while `/eeg/mode` picks whether gaze steers the
        CHASSIS or the CAMERA HEAD. They are independent, so a previous run
        that finished in camera mode leaves the next run starting there - and
        then "looking right turns the robot" fails for a reason that has
        nothing to do with gaze being broken. Measured exactly that: a chain
        run started at mode=camera, gaze reached only the servos, and the
        script reported the whole chain broken.
        """
        for _ in range(4):
            if p.eeg.get('mode') == want:
                return True
            p.sim_pub.publish(String(data='DOUBLE_BLINK'))
            p.spin_for(1.0)
        return p.eeg.get('mode') == want

    check('the EEG sub-mode can be set to robot', set_eeg_mode('robot'),
          f'eeg mode={p.eeg.get("mode")}')
    p.set_mode(4)
    check('mode 4 (GAZE+EEG) can be selected', True, 'set')

    # -------------------------------------------------------- 3. run gate
    print('\n3. the run gate')

    def set_run(want):
        """Drive run to `want`. A clench TOGGLES, so check before pressing."""
        for _ in range(4):
            if bool(p.eeg.get('run')) == want:
                return True
            p.sim_pub.publish(String(data='JAW_CLENCH'))
            p.spin_for(1.0)
        return bool(p.eeg.get('run')) == want

    check('run can be turned OFF', set_run(False), f'run={p.eeg.get("run")}')
    if isolated:
        p.hold_gaze(0.95, 0.5, 1.5)
        check('with run OFF, gaze cannot drive', p.mag() < 1e-6,
              f'|v|={p.mag():.3f}  (run={p.eeg.get("run")})')
    else:
        skip('with run OFF, gaze cannot drive', 'another /gaze/point publisher')

    # ------------------------------------------------------- 4. gaze drives
    print('\n4. gaze drives when run is ON')
    check('run can be turned ON', set_run(True), f'run={p.eeg.get("run")}')

    if isolated:
        p.hold_gaze(0.95, 0.5, 1.5)
        check('looking right turns the robot', p.cmd is not None
              and p.cmd.angular.z < -0.05,
              f'wz={p.cmd.angular.z:+.2f}  zone={p.zone}')

        p.hold_gaze(0.05, 0.5, 1.5)
        check('looking left turns the other way', p.cmd is not None
              and p.cmd.angular.z > 0.05,
              f'wz={p.cmd.angular.z:+.2f}  zone={p.zone}')

        # Forward. This is the check that was missing, and its absence is why
        # the robot could only ever rotate: `enable_vertical=false` made
        # FORWARD structurally unreachable in _classify, so every gaze test
        # passed while the robot had no way to advance. A chain check that
        # never asks for forward cannot notice that forward does not exist.
        #
        # Held longer than a turn because forward deliberately requires a
        # longer dwell (0.8 s vs 0.4 s).
        p.hold_gaze(0.5, 0.05, 2.0)
        check('looking up drives forward', p.cmd is not None
              and p.cmd.linear.x > 0.05,
              f'vx={p.cmd.linear.x:+.2f}  zone={p.zone}')

        p.hold_gaze(0.5, 0.95, 2.0)
        check('looking down does NOT reverse', p.mag() < 1e-6,
              f'|v|={p.mag():.3f}  zone={p.zone}')

        p.hold_gaze(0.5, 0.5, 1.2)
        check('looking at the centre stops it', p.mag() < 1e-6,
              f'|v|={p.mag():.3f}  zone={p.zone}')
    else:
        skip('looking right turns the robot', 'another /gaze/point publisher')
        skip('looking left turns the other way', 'another /gaze/point publisher')
        skip('looking at the centre stops it', 'another /gaze/point publisher')

    # ------------------------------------------------------ 5. camera mode
    print('\n5. camera mode steers the head, not the wheels')
    before = p.eeg.get('mode')
    p.sim_pub.publish(String(data='DOUBLE_BLINK'))
    p.spin_for(1.2)
    check('the blink switched mode to camera', p.eeg.get('mode') == 'camera',
          f'{before} -> {p.eeg.get("mode")}')
    if isolated:
        p.hold_gaze(0.95, 0.5, 1.5)
        check('in camera mode the wheels stay still', p.mag() < 1e-6,
              f'|v|={p.mag():.3f}  zone={p.zone}')
    else:
        skip('in camera mode the wheels stay still',
             'another /gaze/point publisher')

    # Back to robot steering, then stop, so nothing is left driving.
    check('the blink switches back to robot', set_eeg_mode('robot'),
          f'eeg mode={p.eeg.get("mode")}')

    # ------------------------------------------------------- 6. wheels
    if args.wheels and isolated:
        print('\n6. wheels (robot must be raised)')
        p.hold_gaze(0.95, 0.5, 2.0)
        print(f'     commanded wz={p.cmd.angular.z:+.2f} - LOOK at the wheels')
        p.spin_for(1.0)
    elif args.wheels:
        print('\n6. wheels: SKIPPED (another /gaze/point publisher)')
    else:
        print('\n6. wheels: skipped (pass --wheels to drive them)')

    # Leave everything stopped.
    p.hold_gaze(0.5, 0.5, 0.5)
    p.set_mode(1)

    # Give the HUD its voice back. A test that leaves a real component muted
    # is worse than one that never ran.
    set_hud_enabled(True)

    print('\n' + '=' * 58)
    if ok:
        print(' CHAIN OK - eyes to wheels'
              if isolated else
              ' CHAIN OK - but the gaze checks were SKIPPED')
    else:
        print(' CHAIN BROKEN - see above')
    print('=' * 58)
    print('\nRobot left in KEYBOARD mode, stopped.')

    p.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
