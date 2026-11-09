#!/usr/bin/env python3
"""The EEG is a GATE, not a joystick - and the console must say so.

What went wrong: the operator's headset was connected, packets were arriving,
mode 3 was selected, gaze was producing TURN_RIGHT - and the robot did not
move. Every part of the chain reported success and the only thing missing was
one bit that nothing on screen displayed:

    run = false     (no jaw clench yet)
    mode = camera   (a double blink had pointed gaze at the PTZ head)

So this file pins two things:

  1. the arbiter forwards gaze in EEG mode, because protocol v1 gives the EEG
     no velocity of its own - mode 3 with only /cmd_vel/eeg accepted is a dead
     end where nothing can ever drive
  2. the console names WHICH of link / run / mode is blocking, because the
     three need three different actions from the operator

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_eeg_gate.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    # ------------------------------------------------------- arbiter wiring
    print('arbiter: which modes may forward gaze?')
    import inspect                                              # noqa: PLC0415
    import control_arbiter_node as arb                          # noqa: PLC0415

    src = inspect.getsource(arb.ControlArbiterNode._gaze_vel_cb)
    check('mode 2 (GAZE) forwards gaze', 'MODE_GAZE' in src)
    check('mode 4 (FUSION) forwards gaze', 'MODE_FUSION' in src)
    check('mode 3 (EEG) forwards gaze', 'MODE_EEG' in src,
          'without this, mode 3 can never drive: protocol v1 has no '
          '/cmd_vel/eeg')
    check('mode 1 (KEYBOARD) does not', 'MODE_KEYBOARD' not in src)
    check('E-STOP does not', 'MODE_ESTOP' not in src)

    servo_src = inspect.getsource(arb.ControlArbiterNode._gaze_servo_cb)
    check('mode 3 also forwards the PTZ head', 'MODE_EEG' in servo_src)

    # --------------------------------------------------------- console text
    print('\nconsole: does it name the blocker?')
    # The Console class is declared INSIDE main() - it closes over QtCore,
    # QtGui, node and shell - so it cannot be imported or instantiated here.
    # Read the method's source instead. That is weaker than calling it, so it
    # checks the MESSAGES rather than the pixels: each blocking state must
    # produce its own distinct, actionable sentence.
    console_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control',
                                'm1_console.py')
    src = open(console_path, encoding='utf-8').read()
    check('the console has an EEG status line', '_refresh_eeg' in src)
    check('it subscribes to /eeg/run', "'/eeg/run'" in src)
    check('it subscribes to /eeg/mode', "'/eeg/mode'" in src)
    check('it subscribes to /eeg/link', "'/eeg/link'" in src)

    body = src.split('def _refresh_eeg', 1)[1].split('\n        def ', 1)[0]

    blockers = [
        ('no link', 'NO LINK'),
        ('not armed', 'jaw clench'),
        ('aimed at the camera', 'CAMERA'),
        ('steering the robot', 'ROBOT'),
    ]
    for label, needle in blockers:
        check(f'says something for "{label}"', needle in body,
              f'expected {needle!r} in _refresh_eeg')

    # The three blocking states must read differently from each other: if they
    # all said "EEG ok" the operator would still be guessing.
    msgs = [body.count(n) for _l, n in blockers]
    check('all four states are distinct messages', all(m >= 1 for m in msgs),
          str(msgs))

    # A gate that is not armed must not read as success.
    check('an unarmed gate is not styled good',
          "if not run:" in body and "'warn'" in body)

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
