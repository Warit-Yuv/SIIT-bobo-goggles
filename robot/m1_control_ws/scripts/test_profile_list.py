#!/usr/bin/env python3
"""Confirm the Profile dropdown actually contains every profile in PROFILES.

The dropdown used to be filled from a hardcoded tuple of names that had to be
kept in step with `PROFILES` by hand. Adding the 'posture' profile did not add
it to the tuple, so the profile existed, was tested, and was unreachable from
the GUI - with nothing anywhere reporting a problem.

This checks the widget, not the dict, because the dict was never the problem.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_profile_list.py
"""

from __future__ import annotations

import os
import sys

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

    combo = dlg.cmb_profile
    items = [(combo.itemText(i), combo.itemData(i)) for i in range(combo.count())]
    keys = [k for _t, k in items]

    print('the dropdown contains:')
    for text, key in items:
        print(f'    {key:<9} {text}')

    print('\nchecks:')
    check('the dropdown is populated', combo.count() > 0, str(combo.count()))
    check('every PROFILES key is in the dropdown',
          set(keys) == set(m1_console.CalibDialog.PROFILES),
          f'{sorted(keys)} vs {sorted(m1_console.CalibDialog.PROFILES)}')
    check('there are no extra entries',
          len(keys) == len(set(keys)) == len(m1_console.CalibDialog.PROFILES))
    check('the Posture profile is reachable', 'posture' in keys,
          'the whole point of this file')
    check('the Posture label says how long it takes',
          'min' in dict((k, t) for t, k in items)['posture'],
          dict((k, t) for t, k in items)['posture'])
    check('every label is non-empty', all(t for t, _k in items))

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
