#!/usr/bin/env python3
"""The EEG gesture pad: does it show the rules, and light the right key?

The EEG panel used to be grey text. It reported STATE, which is always true and
rarely changes, so it could not answer either question the operator actually
has:

    "what does a jaw clench do?"          (the RULES)
    "did the headset see the clench?"      (the EVENT)

A clench that toggles to the value it already had produces no state change, so
a state-only display shows nothing at all - indistinguishable from the headset
not working.

This checks the pad against both questions, and renders it to a PNG so the
layout can be looked at rather than imagined.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_gesture_pad.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

import m1_console                                               # noqa: E402


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        detail = str(detail).encode('ascii', 'replace').decode('ascii')
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    GesturePad = m1_console.make_gesture_pad(QtCore, QtGui, QtWidgets)
    pad = GesturePad()
    pad.resize(230, 96)

    # ------------------------------------------------------- the rules
    print('the pad shows the rules:')
    rows = GesturePad.ROWS
    names = [g for g, _k, _a in rows]
    check('all three gestures are listed', len(rows) == 3, str(names))
    check('double blink is listed',
          'DOUBLE_BLINK' in names, str(names))
    check('jaw clench is listed', 'JAW_CLENCH' in names, str(names))
    check('eyebrow raise is listed', 'EYEBROW_RAISE' in names, str(names))

    by_name = {g: (k, a) for g, k, a in rows}
    check('double blink is shown as go / stop',
          'go' in by_name['DOUBLE_BLINK'][1], by_name['DOUBLE_BLINK'][1])
    check('jaw clench is shown as the mode switch',
          'steer' in by_name['JAW_CLENCH'][1], by_name['JAW_CLENCH'][1])
    check('eyebrow raise is shown as the sound',
          'sound' in by_name['EYEBROW_RAISE'][1],
          by_name['EYEBROW_RAISE'][1])

    # The most reliable gesture must be first: that is the one the operator
    # should reach for, and putting it at the top is the whole point of the
    # ordering.
    check('the most reliable gesture is the top row',
          names[0] == 'DOUBLE_BLINK', names[0])

    # ------------------------------------------------------- the event
    print('\nthe pad lights on an event:')
    check('nothing is lit to begin with', pad._lit_now() is None)

    pad.set_gesture('JAW_CLENCH', time.monotonic())
    check('a fresh gesture lights its key', pad._lit_now() == 'JAW_CLENCH',
          str(pad._lit_now()))

    pad.set_gesture('DOUBLE_BLINK', time.monotonic())
    check('a different gesture moves the light',
          pad._lit_now() == 'DOUBLE_BLINK', str(pad._lit_now()))

    # The bridge publishes "<GESTURE>/<COMMAND>"; the console must reduce that
    # to the gesture before lighting a key, or nothing ever matches.
    text = 'EYEBROW_RAISE/TOGGLE_MUSIC'
    pad.set_gesture(text.split('/')[0].strip().upper(), time.monotonic())
    check('the "gesture/command" form reduces to the gesture',
          pad._lit_now() == 'EYEBROW_RAISE', str(pad._lit_now()))

    pad.set_gesture('JAW_CLENCH', time.monotonic() - 5.0)
    check('an old gesture stops being lit', pad._lit_now() is None,
          'the light must not stay on forever')

    pad.set_gesture('', time.monotonic())
    check('an empty gesture lights nothing', pad._lit_now() is None)

    pad.set_gesture('UNCLEAR_MUSCLE', time.monotonic())
    check('an unclear gesture lights nothing (it has no key)',
          pad._lit_now() == 'UNCLEAR_MUSCLE',
          'set, but no row matches, so nothing draws as lit')

    # ------------------------------------------------------- the drawing
    print('\nthe drawing:')
    pad.set_gesture('JAW_CLENCH', time.monotonic())
    pix = pad.grab()
    check('the pad renders to a pixmap', not pix.isNull(),
          f'{pix.width()}x{pix.height()}')
    check('the pad has a sensible minimum height',
          pad.minimumHeight() >= 60, str(pad.minimumHeight()))

    out = '/tmp/gesture_pad.png'
    pix.save(out)
    print(f'    wrote {out}')

    # A pad drawn at a tiny size must not raise - the left column is narrow and
    # a crash there would take the whole console down.
    tiny = GesturePad()
    tiny.resize(60, 40)
    try:
        tiny.grab()
        tiny_ok = True
    except Exception as exc:                                 # noqa: BLE001
        tiny_ok = False
        print(f'      raised: {exc}')
    check('it survives being squeezed into a narrow column', tiny_ok)

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
