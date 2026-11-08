"""Runs every calibration profile end to end against a known-good tracker.

    QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_all_profiles.py

All four profiles must complete. This exists because they were all reported as
broken at once, which is the signature of a shared fault rather than four
separate ones - so the check runs the real dialog over every profile instead of
reasoning about them one at a time.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

from m1_console import CalibDialog                             # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


class Tracker:
    """A perfect tracker: reports exactly what a real eye+head would."""

    deadzone_ratio = 0.30
    deadzone_ratio_y = 0.16

    def __init__(self):
        self.u, self.v = 0.5, 0.5

    def latest_raw(self):
        return ((self.u - 0.5) / 2.0 + 0.62,
                (self.v - 0.5) / 1.2 + 0.55,
                True, 0.0)


def run(app, dialog_cls, profile, limit=180):
    tracker = Tracker()
    dlg = dialog_cls(tracker, QtCore.QRect(0, 0, 1920, 1080), None)
    dlg.profile = profile
    dlg.start()
    deadline = time.time() + limit
    seen_dots = 0
    while dlg.phase != 'done' and time.time() < deadline:
        if dlg.u is not None:
            tracker.u, tracker.v = dlg.u, dlg.v
        seen_dots = max(seen_dots, len(dlg.captured))
        app.processEvents()
        time.sleep(0.001)
    dlg.timer.stop()
    dlg.anim.stop()
    return dlg, seen_dots


def main():
    app = QtWidgets.QApplication(sys.argv)
    dialog_cls = CalibDialog.build(QtCore, QtGui, QtWidgets)

    # Shorten for speed. The floor has to come down with it, or the fast dots
    # collect fewer than MIN_SAMPLES and every profile fails for the same
    # uninteresting reason.
    CalibDialog.PROFILES = {k: dict(v) for k, v in CalibDialog.PROFILES.items()}
    for k in CalibDialog.PROFILES:
        CalibDialog.PROFILES[k]['settle'] = 0.04
        CalibDialog.PROFILES[k]['sample'] = 0.30
    CalibDialog.MIN_SAMPLE_SEC = 0.25

    for profile in ('fast', 'long', 'super', 'ultra'):
        expected = len(CalibDialog.PROFILES[profile]['dots'])
        dlg, seen = run(app, dialog_cls, profile)
        ok = dlg.phase == 'done' and len(dlg.captured) == expected
        check('%s completes all %d dots' % (profile, expected), ok,
              'phase=%s  captured=%d/%d'
              % (dlg.phase, len(dlg.captured), expected))
        if ok:
            check('%s produces a calibration' % profile,
                  dlg.result_calib is not None,
                  str(tuple(round(v, 3) for v in dlg.result_calib))
                  if dlg.result_calib else 'none')

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
