"""Checks the super-long profile's shape and the shrinking calibration dot.

    QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_calib_pulse.py

Two things are checked here that the other calibration tests do not cover:

1. The 'super' profile is weighted the way it claims. If the dots drift back
   to an even grid, the vertical axis loses the extra samples it was given and
   up/down gets worse again - silently, because the profile still runs.

2. The dot actually shrinks, and reaches its smallest exactly when sampling
   starts. A dot that does not shrink gives the operator no cue about when to
   look, so every dot's first samples land on the previous dot's position.
"""

import sys
import time
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

    # ---------------- the super profile's shape -------------------------
    super_dots = CalibDialog.PROFILES['super']['dots']
    check('super profile has 20 dots', len(super_dots) == 20,
          '%d dots' % len(super_dots))

    us = [d[1] for d in super_dots]
    vs = [d[2] for d in super_dots]
    check('super profile still reaches the screen edges',
          min(us) <= 0.10 and max(us) >= 0.90
          and min(vs) <= 0.10 and max(vs) >= 0.90,
          'u %.2f..%.2f  v %.2f..%.2f' % (min(us), max(us), min(vs), max(vs)))

    # The centre COLUMN: this is what carries vertical.
    centre_column = sum(1 for d in super_dots if abs(d[1] - 0.50) < 0.02)
    check('centre column is heavily sampled', centre_column >= 7,
          '%d of 20 dots at u=0.50' % centre_column)

    # The centre BAND: reinforces the middle, where people actually look.
    centre_band = sum(1 for d in super_dots if abs(d[2] - 0.50) < 0.20)
    check('middle band is heavily sampled', centre_band >= 8,
          '%d of 20 dots within 0.20 of centre' % centre_band)

    # Corners should be sparse: they cost time and teach the fit little.
    corners = sum(1 for d in super_dots
                  if (d[1] < 0.25 or d[1] > 0.75) and (d[2] < 0.25 or d[2] > 0.75))
    check('corners are not over-sampled', corners <= 4,
          '%d corner dots' % corners)

    # Vertical must be sampled more densely than a plain 3x3 grid would.
    distinct_v = len(set(round(d[2], 2) for d in super_dots))
    check('vertical axis has many distinct levels', distinct_v >= 8,
          '%d distinct v levels' % distinct_v)

    # ---------------- the shrinking dot ---------------------------------
    dlg = dialog_cls(FakeNode(), QtCore.QRect(0, 0, 1920, 1080), None)
    dlg.show()
    app.processEvents()

    check('dot starts at its largest', dlg._pulse_radius() == CalibDialog.PULSE_MIN,
          'before start it holds the small size')

    dlg.profile = 'super'
    dlg.pulse_t0 = time.monotonic()
    r_start = dlg._pulse_radius()

    settle = CalibDialog.PROFILES['super']['settle']
    dlg.pulse_t0 = time.monotonic() - settle * 0.5
    r_mid = dlg._pulse_radius()

    dlg.pulse_t0 = time.monotonic() - settle
    r_end = dlg._pulse_radius()

    check('dot is large at the start of a point',
          abs(r_start - CalibDialog.PULSE_MAX) < 1.0, 'r %.1f' % r_start)
    check('dot shrinks through the settle phase', r_mid < r_start - 1.0,
          'r %.1f halfway' % r_mid)
    check('dot is smallest exactly when sampling begins',
          abs(r_end - CalibDialog.PULSE_MIN) < 1.0, 'r %.1f' % r_end)
    check('dot never grows back while sampling',
          dlg._pulse_radius() <= CalibDialog.PULSE_MIN + 0.01)

    # It must be red, since that is the whole point of finding it instantly.
    # Sample the RING, not the exact centre: the centre carries a small white
    # core that marks the aim point, so sampling (200,200) reads white.
    dlg.profile = 'fast'
    dlg.pulse_t0 = time.monotonic() - 1.0
    pix = QtGui.QPixmap(400, 400)
    pix.fill(QtGui.QColor('#101010'))
    painter = QtGui.QPainter(pix)
    dlg._draw_dot(painter, QtCore.QPointF(200.0, 200.0))
    painter.end()
    img = pix.toImage()
    ring = img.pixelColor(200 + int(CalibDialog.PULSE_MIN * 0.6), 200)
    check('the dot is red', ring.red() > 180 and ring.green() < 160,
          'ring rgb(%d,%d,%d)' % (ring.red(), ring.green(), ring.blue()))
    core = img.pixelColor(200, 200)
    check('the dot has a white centre marking the aim point',
          core.red() > 200 and core.green() > 200 and core.blue() > 200,
          'core rgb(%d,%d,%d)' % (core.red(), core.green(), core.blue()))

    dlg.close()
    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
