#!/usr/bin/env python3
"""The keyboard fallback for the three EEG gestures: B, N, M.

Why this exists
---------------
The EEG classifier on this headset confuses a jaw clench with an eyebrow raise
often enough that it reports UNCLEAR_MUSCLE and refuses to guess. When it does,
the operator has no way to say "go", "switch what gaze steers", or "play a
sound" - so the same three actions get a key each.

The design rule this file pins down: the keyboard does NOT publish /eeg/run,
/eeg/mode or /eeg/music itself. It asks the BRIDGE to apply the gesture, and
the bridge runs exactly the code it would have run for the headset. Two
independent paths would let one rot silently, and then the fallback would be
testing something the real input never touches.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_eeg_keys.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

import m1_console                                               # noqa: E402
from eeg_udp_bridge import (EEGCore, GESTURE_COMMANDS,           # noqa: E402
                            MODE_CAMERA, MODE_ROBOT)


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        detail = str(detail).encode('ascii', 'replace').decode('ascii')
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    # ------------------------------------------------------------- the keys
    print('the keys:')
    keys = m1_console.EEG_KEYS
    check('B is the blink', keys.get('b') == 'DOUBLE_BLINK', str(keys.get('b')))
    check('N is the jaw', keys.get('n') == 'JAW_CLENCH', str(keys.get('n')))
    check('M is the brow', keys.get('m') == 'EYEBROW_RAISE',
          str(keys.get('m')))
    check('every key maps to a gesture the bridge understands',
          all(g in GESTURE_COMMANDS for g in keys.values()),
          str(sorted(keys.values())))

    # The keys must not shadow anything the operator already uses.
    drive = set(m1_console.DRIVE_KEYS)
    modes = set(m1_console.MODE_KEYS)
    servo = {'i', 'k', 'j', 'l', 'u'}
    check('no key collides with the drive keys', not (set(keys) & drive),
          str(sorted(set(keys) & drive)))
    check('no key collides with the mode digits', not (set(keys) & modes))
    check('no key collides with the servo keys', not (set(keys) & servo),
          str(sorted(set(keys) & servo)))

    # ------------------------------------------------------- through the core
    print('\nthe bridge applies them exactly like a headset gesture:')

    def press(core, gesture, at):
        return core.inject_gesture(gesture, at)

    c = EEGCore(link_timeout=1.5)
    note = press(c, 'DOUBLE_BLINK', 1.0)
    check('a blink is accepted', note is not None, str(note))
    check('it turns run ON', c.run is True)
    check('the note says it came from the keyboard',
          note and 'keyboard' in note, str(note))
    check('the gesture is recorded for the GUI',
          c.status()['last'].startswith('DOUBLE_BLINK'), c.status()['last'])

    press(c, 'DOUBLE_BLINK', 2.0)
    check('pressing it again turns run OFF', c.run is False)

    press(c, 'JAW_CLENCH', 3.0)
    check('N switches what gaze steers, from robot to camera',
          c.mode == MODE_CAMERA, c.mode)
    press(c, 'JAW_CLENCH', 4.0)
    check('and back again', c.mode == MODE_ROBOT, c.mode)

    press(c, 'EYEBROW_RAISE', 5.0)
    check('M turns music on', c.music is True)
    press(c, 'EYEBROW_RAISE', 6.0)
    check('and off again', c.music is False)

    check('each press is counted as a gesture', c.gesture_seq == 6,
          str(c.gesture_seq))

    # ------------------------------------------------- the watchdog must not eat it
    print('\nthe link watchdog must not undo a keypress:')
    # This is the trap: marking the link "up" on a keypress would make the
    # watchdog force run back off one timeout later, so B would start the robot
    # and then stop it again for no visible reason.
    c2 = EEGCore(link_timeout=1.5)
    press(c2, 'DOUBLE_BLINK', 10.0)
    check('run is on after the keypress', c2.run is True)
    c2.tick(now=10.5)
    check('still on after half a timeout', c2.run is True)
    c2.tick(now=30.0)
    check('STILL on after 20 s of no input', c2.run is True,
          'the keyboard is a local input, not a remote link')
    check('the link is still reported down', c2.link_ok is False,
          'a keypress must not claim the headset is alive')

    # ---------------------------------------------- a real gesture still works
    print('\nand a real headset gesture still works the same way:')
    c3 = EEGCore(link_timeout=1.5)
    c3.on_message({'v': 1, 'seq': 1, 'event': {'gesture': 'DOUBLE_BLINK'}},
                  now=1.0)
    check('a UDP blink also turns run on', c3.run is True)
    c3.on_message({'v': 1, 'seq': 2, 'state': {'run': True}}, now=1.2)
    check('a UDP packet marks the link up', c3.link_ok is True)

    # --------------------------------------------------------- unknown input
    print('\nunknown input is rejected, not guessed:')
    c4 = EEGCore(link_timeout=1.5)
    before = (c4.run, c4.mode, c4.music, c4.gesture_seq)
    note = press(c4, 'SOMETHING_ELSE', 1.0)
    check('an unknown gesture is refused', note is None, str(note))
    check('and changes nothing',
          (c4.run, c4.mode, c4.music, c4.gesture_seq) == before)
    check('and is counted as dropped', c4.dropped == 1, str(c4.dropped))

    # --------------------------------------------------------- the console side
    print('\nthe console routes keys to the bridge, not to /eeg/run:')
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'src', 'm1_control', 'm1_control',
                            'm1_console.py'), encoding='utf-8').read()
    check('the console publishes /eeg/simulate',
          "'/eeg/simulate'" in src)
    check('the console does NOT publish /eeg/run itself',
          "create_publisher(Bool, '/eeg/run'" not in src,
          'two publishers on a latched topic would both look true')
    check('the console does NOT publish /eeg/mode itself',
          "create_publisher(String, '/eeg/mode'" not in src)
    check('the key handler is wired', '_eeg_key' in src)
    check('the keys are shown in the on-screen help',
          'B blink' in src and 'N jaw' in src and 'M brow' in src)

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
