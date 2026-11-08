"""Checks that the calibration dialog never hides a dot behind its own bar.

    QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_calib_layout.py

The bottom dots used to be drawn underneath the control bar and simply
disappeared. That is not a cosmetic problem: while a dot is invisible the
operator has no idea whether to keep holding their gaze there, and the samples
taken during that phase are taken looking at whatever happened to be in front
of them. The calibration is then fitted from data that does not mean what it
claims to.

This measures the drawn geometry, so it fails if the bar ever grows into the
dot area again.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

from m1_console import CalibDialog                             # noqa: E402


class FakeNode:
    deadzone_ratio = 0.30
    deadzone_ratio_y = 0.16

    def latest_raw(self):
        return (0.5, 0.5, True, 0.0)


failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


def main():
    app = QtWidgets.QApplication(sys.argv)
    dialog_cls = CalibDialog.build(QtCore, QtGui, QtWidgets)

    # A maximised console on a 1080p screen, which is the real case.
    screen = QtCore.QRect(0, 0, 1920, 1080)
    dlg = dialog_cls(FakeNode(), screen, None)
    dlg.show()
    app.processEvents()
    app.processEvents()

    area = dlg._dot_area()
    bar = dlg.bar
    bar_top = bar.mapTo(dlg, QtCore.QPoint(0, 0)).y()
    bar_bottom = bar_top + bar.height()

    check('the bar is inside the window', bar_top > 0 and bar_bottom <= dlg.height(),
          'bar %d..%d of %d' % (bar_top, bar_bottom, dlg.height()))
    check('the dot area ends above the bar', area.bottom() <= bar_top,
          'area ends %d, bar starts %d' % (area.bottom(), bar_top))
    check('the dot area still uses most of the window',
          area.height() > dlg.height() * 0.6,
          '%d of %d px' % (area.height(), dlg.height()))

    # Every dot of every profile must be fully inside the area, including its
    # 26 px glow ring.
    radius = 26
    for profile, spec in CalibDialog.PROFILES.items():
        worst = None
        for name, u, v, _text in spec['dots']:
            x = area.left() + area.width() * u
            y = area.top() + area.height() * v
            if (y + radius > area.bottom() or y - radius < area.top()
                    or x + radius > area.right() or x - radius < area.left()):
                worst = name
                break
        check('%s profile dots are all fully visible' % profile,
              worst is None, ('%s is clipped' % worst) if worst else '')

    # The extreme dots must be far apart, or the fit has no signal.
    fast = CalibDialog.PROFILES['fast']['dots']
    us = [d[1] for d in fast]
    vs = [d[2] for d in fast]
    check('dots span at least 80% of the area',
          (max(us) - min(us)) >= 0.80 and (max(vs) - min(vs)) >= 0.80,
          'u %.2f..%.2f  v %.2f..%.2f' % (min(us), max(us), min(vs), max(vs)))

    dlg.close()
    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
