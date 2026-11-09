#!/usr/bin/env python3
"""Protocol v1 of the EEG bridge, tested without a headset or a network.

Replaces the old `test_eeg_udp.py`, which spoke protocol v0
(`{"command": "GO", "gesture": "DOUBLE_BLINK"}`) against a bridge that now
expects a repeated `state` block. That mismatch meant the old test could only
ever fail, and a test that can only fail is a test nobody runs.

What protocol v1 actually promises, and what this file checks:

  * the sender repeats its FULL state in every packet, so a single lost
    packet heals itself on the next one
  * a packet with a LOWER seq than one already seen is stale and must be
    ignored - UDP reorders
  * a gesture TOGGLES something; it does not carry a velocity
  * when the link goes quiet the run flag is forced off, so a dead laptop
    cannot leave the robot driving
  * a malformed packet is dropped, not crashed on

Run:
    python3 scripts/test_eeg_protocol.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

from eeg_udp_bridge import (GESTURE_COMMANDS, MODE_CAMERA, MODE_ROBOT,  # noqa: E402
                            PROTOCOL_VERSION, TOGGLE_MODE, TOGGLE_RUN,
                            EEGCore)


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    print('constants:')
    check('the protocol version is 1', PROTOCOL_VERSION == 1,
          str(PROTOCOL_VERSION))
    # Ordered by how reliably the classifier separates them: the most
    # consistent gesture gets the go/stop, the least gets a sound effect.
    check('double blink is the run toggle (most reliable)',
          GESTURE_COMMANDS.get('DOUBLE_BLINK') == TOGGLE_RUN,
          str(GESTURE_COMMANDS.get('DOUBLE_BLINK')))
    check('jaw clench is the mode toggle',
          GESTURE_COMMANDS.get('JAW_CLENCH') == TOGGLE_MODE,
          str(GESTURE_COMMANDS.get('JAW_CLENCH')))

    print('\nfollowing the sender state:')
    c = EEGCore(link_timeout=1.5)
    c.on_message({'v': 1, 'seq': 1, 'state': {'run': True,
                                              'mode': MODE_ROBOT}}, now=10.0)
    check('run comes from the state block', c.run is True)
    check('mode comes from the state block', c.mode == MODE_ROBOT)
    check('the link is marked up', c.link_ok is True)

    c.on_message({'v': 1, 'seq': 2, 'state': {'run': True,
                                              'mode': MODE_CAMERA}}, now=10.1)
    check('mode follows the sender', c.mode == MODE_CAMERA)

    print('\nself-healing:')
    # One lost packet must not matter - the next one carries the full state.
    c.on_message({'v': 1, 'seq': 3, 'state': {'run': False,
                                              'mode': MODE_CAMERA}}, now=10.2)
    check('a dropped packet cannot leave run stuck on', c.run is False)
    c.on_message({'v': 1, 'seq': 4, 'state': {'run': False,
                                              'mode': MODE_ROBOT}}, now=10.3)
    check('the next packet restores the full state',
          c.run is False and c.mode == MODE_ROBOT)

    print('\nstale packets:')
    before = (c.run, c.mode)
    c.on_message({'v': 1, 'seq': 2, 'state': {'run': True,
                                              'mode': MODE_CAMERA}}, now=10.4)
    check('a lower seq is ignored', (c.run, c.mode) == before,
          f'{before} -> {(c.run, c.mode)}')

    print('\ngestures toggle:')
    c2 = EEGCore(link_timeout=1.5, start_run=False)
    c2.on_message({'v': 1, 'seq': 1, 'event': {'gesture': 'DOUBLE_BLINK'},
                   'state': {'run': True}}, now=20.0)
    check('double blink can turn run ON', c2.run is True)
    c2.on_message({'v': 1, 'seq': 2, 'event': {'gesture': 'DOUBLE_BLINK'},
                   'state': {'run': False}}, now=20.1)
    check('double blink can turn run OFF again', c2.run is False)

    # A packet with no state block at all - a hand-typed datagram - must still
    # toggle, otherwise debugging over netcat is impossible.
    c2b = EEGCore(link_timeout=1.5, start_run=False)
    c2b.on_message({'event': {'gesture': 'DOUBLE_BLINK'}}, now=21.0)
    check('a state-less packet still toggles', c2b.run is True)
    check('the gesture is remembered',
          c2.last_gesture == 'DOUBLE_BLINK', c2.last_gesture)

    print('\nthe link watchdog:')
    c3 = EEGCore(link_timeout=1.5, start_run=True)
    c3.on_message({'v': 1, 'seq': 1, 'state': {'run': True}}, now=30.0)
    check('run is on while the link is alive', c3.run is True)
    c3.tick(now=30.5)
    check('a short gap does not stop it', c3.run is True)
    c3.tick(now=32.0)
    check('a silent link forces run off', c3.run is False)
    check('a silent link is reported as down', c3.link_ok is False)

    print('\nmalformed input:')
    c4 = EEGCore(link_timeout=1.5)
    for junk in ({}, {'v': 99}, {'state': 'not a dict'},
                 {'state': {'run': 'maybe'}}, {'seq': 'x'}):
        try:
            c4.on_message(junk, now=40.0)
            survived = True
        except Exception as exc:                            # noqa: BLE001
            survived = False
            print(f'      raised on {junk}: {exc}')
        check(f'junk {junk} does not crash', survived)
    check('junk is counted, not silently swallowed', c4.dropped > 0,
          f'{c4.dropped} dropped')
    check('a wrong protocol version is remembered, not just dropped',
          c4.bad_version == 99, str(c4.bad_version))

    print('\ngestures inferred from state (sender sends no event block):')
    # A sender is not ours to change. If it publishes state but does not label
    # the gesture, the GUI would show nothing at all - indistinguishable from a
    # disconnected headset. The state DELTA identifies the gesture unambiguously
    # because every toggle owns exactly one field.
    c6 = EEGCore(link_timeout=1.5)
    c6.on_message({'v': 1, 'seq': 1, 'state': {'run': False,
                                               'mode': 'robot',
                                               'music': False}}, now=50.0)
    check('adopting the initial state is not a gesture',
          c6.gesture_seq == 0,
          'the first packet always differs from the defaults')

    c6.on_message({'v': 1, 'seq': 2, 'state': {'run': True,
                                               'mode': 'robot',
                                               'music': False}}, now=50.5)
    check('a run flip is read as a double blink',
          c6.last_gesture == 'DOUBLE_BLINK', c6.last_gesture)
    check('and it is counted', c6.gesture_seq == 1, str(c6.gesture_seq))

    c6.on_message({'v': 1, 'seq': 3, 'state': {'run': True,
                                               'mode': 'camera',
                                               'music': False}}, now=51.0)
    check('a mode flip is read as a jaw clench',
          c6.last_gesture == 'JAW_CLENCH', c6.last_gesture)

    c6.on_message({'v': 1, 'seq': 4, 'state': {'run': True,
                                               'mode': 'camera',
                                               'music': True}}, now=51.5)
    check('a music flip is read as an eyebrow raise',
          c6.last_gesture == 'EYEBROW_RAISE', c6.last_gesture)

    before = c6.gesture_seq
    c6.on_message({'v': 1, 'seq': 5, 'state': {'run': True,
                                               'mode': 'camera',
                                               'music': True}}, now=52.0)
    check('a heartbeat with no change is NOT a gesture',
          c6.gesture_seq == before,
          f'{before} -> {c6.gesture_seq}')

    # An explicit gesture field must win over inference, and both shapes must
    # be accepted: nested in `event`, or at the top level.
    c7 = EEGCore(link_timeout=1.5)
    c7.on_message({'v': 1, 'seq': 1, 'gesture': 'JAW_CLENCH'}, now=60.0)
    check('a top-level gesture field is accepted',
          c7.last_gesture == 'JAW_CLENCH', c7.last_gesture)

    print('\nstartup state:')
    c5 = EEGCore(link_timeout=1.5, start_run=False)
    check('it does not start running', c5.run is False,
          'a bridge that boots into RUN is a robot that moves on power-up')
    c6 = EEGCore(link_timeout=1.5, start_run=True)
    check('it can be told to start running', c6.run is True)

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
