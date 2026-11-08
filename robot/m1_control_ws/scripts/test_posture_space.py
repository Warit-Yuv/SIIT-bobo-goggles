#!/usr/bin/env python3
"""Press SPACE on a posture card and check the calibration continues.

This is a regression test for a real failure: pressing SPACE on the first
posture card CLOSED the dialog instead of starting the posture.

The cause was Qt's button semantics, not the handler. In a QDialog every
QPushButton is an `autoDefault` button, so SPACE and Return activate whichever
button currently holds focus. `start()` disables btn_go, focus falls to
btn_cancel, and btn_cancel is connected to `reject` - so the key that was meant
to begin a posture dismissed the whole calibration. The dialog's
`keyPressEvent` never ran, because the focused button consumed the key first.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_posture_space.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

import m1_console                                               # noqa: E402


class FakeNode:
    deadzone_ratio = 0.30
    deadzone_ratio_y = 0.22

    def latest_raw(self):
        return (0.5, 0.5, True, 0.0)

    def features(self, max_age=0.5):
        return [0.0] * 10


def press(widget, key):
    """Deliver a key press the way the window system would."""
    ev = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, key, QtCore.Qt.NoModifier)
    QtWidgets.QApplication.sendEvent(widget, ev)


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    dlg = m1_console.CalibDialog.build(QtCore, QtGui, QtWidgets)(
        FakeNode(), QtCore.QRect(0, 0, 1920, 1080))
    dlg.profile = 'posture'

    print('setup:')
    check('the buttons are not autoDefault', not dlg.btn_go.autoDefault()
          and not dlg.btn_cancel.autoDefault(),
          'autoDefault is what let SPACE press Cancel')
    check('the buttons are not the default button', not dlg.btn_go.isDefault()
          and not dlg.btn_cancel.isDefault())
    check('the dialog can hold focus',
          dlg.focusPolicy() == QtCore.Qt.StrongFocus)

    print('\nstarting:')
    dlg.start()
    app.processEvents()
    check('the first posture shows a card', dlg.phase == 'intro', dlg.phase)
    check('the card names the posture', dlg.lbl.text() == 'Sit normally',
          dlg.lbl.text())
    check('it says how many postures there are',
          '1 of 4' in dlg.status.text(), dlg.status.text())
    check('the Start button is disabled during the run',
          not dlg.btn_go.isEnabled())
    check('the profile cannot be changed mid-run',
          not dlg.cmb_profile.isEnabled())

    print('\nthe SPACE key:')
    result = {'value': None}
    dlg.finished.connect(lambda r: result.__setitem__('value', r))

    press(dlg, QtCore.Qt.Key_Space)
    app.processEvents()

    check('SPACE does NOT close the dialog', result['value'] is None,
          f'finished with {result["value"]!r} - it was dismissed')
    check('SPACE starts the posture', dlg.phase != 'intro', dlg.phase)
    check('a dot is now shown', dlg.u is not None,
          f'u={dlg.u} v={dlg.v}')
    check('the dot belongs to the first posture',
          0.04 <= dlg.u <= 0.96 and 0.04 <= dlg.v <= 0.96,
          f'({dlg.u:.2f}, {dlg.v:.2f})')
    check('the intro text is cleared', dlg.intro_text is None)
    check('the timer is running', dlg.timer.isActive())

    print('\nthe second posture card:')
    # Walk to the end of block 1 and make sure block 2 shows a card.
    dlg.idx = 20
    dlg.block_idx = 0
    dlg.phase = 'idle'
    dlg._begin_point()
    app.processEvents()
    check('posture 2 shows a card', dlg.phase == 'intro', dlg.phase)
    check('the card names the second posture', dlg.lbl.text() == 'Lean back',
          dlg.lbl.text())
    check('the counter says 2 of 4', '2 of 4' in dlg.status.text(),
          dlg.status.text())

    press(dlg, QtCore.Qt.Key_Space)
    app.processEvents()
    check('SPACE does not close it the second time either',
          result['value'] is None, f'finished with {result["value"]!r}')
    check('the second posture starts', dlg.phase != 'intro', dlg.phase)

    print('\nother keys:')
    # A stray key mid-run must not dismiss a calibration three minutes in.
    dlg.phase = 'sample'
    for key, name in ((QtCore.Qt.Key_Return, 'Return'),
                      (QtCore.Qt.Key_Enter, 'Enter')):
        press(dlg, key)
        app.processEvents()
        check(f'{name} does not close the dialog mid-run',
              result['value'] is None, f'finished with {result["value"]!r}')

    print('\nEscape still cancels:')
    press(dlg, QtCore.Qt.Key_Escape)
    app.processEvents()
    check('Escape cancels the calibration',
          result['value'] == QtWidgets.QDialog.Rejected,
          f'finished with {result["value"]!r}')

    dlg.timer.stop()
    dlg.anim.stop()

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
