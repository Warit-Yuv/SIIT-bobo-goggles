"""Checks the ultra profile and the calibration quality report.

    QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_quality.py

The quality report exists because "is this calibration any good?" had no
answer. Four numbers that look plausible whatever they are is how a broken
calibration runs to completion, reports success, and still steers wrong.

So the report is checked in both directions:

  * a tracker with a real signal must report a SMALL fit error and a good
    verdict
  * a tracker whose vertical signal barely moves must report a LARGE error and
    say the signal is too weak - not silently return a clamped gain and a
    cheerful "complete"

A report that only ever says "fine" would be worse than no report at all.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

from m1_console import CalibDialog, _ultra_dots                # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


class Tracker:
    """Reports the gaze a real operator would produce looking at (u, v)."""

    deadzone_ratio = 0.30
    deadzone_ratio_y = 0.16

    def __init__(self, ox, oy, gx, gy, noise=0.0):
        self.u, self.v = 0.5, 0.5
        self.ox, self.oy, self.gx, self.gy = ox, oy, gx, gy
        self.noise = noise
        self.n = 0

    def latest_raw(self):
        # Deterministic, bounded "noise": a sawtooth, so the test does not
        # depend on a random seed.
        self.n += 1
        j = ((self.n % 7) - 3) / 3.0 * self.noise
        return ((self.u - 0.5) / self.gx + self.ox + j,
                (self.v - 0.5) / self.gy + self.oy - j,
                True, 0.0)


def run(dialog_cls, app, tracker, profile='fast'):
    dlg = dialog_cls(tracker, QtCore.QRect(0, 0, 1920, 1080), None)
    dlg.profile = profile
    dlg.start()
    deadline = time.time() + 60
    while dlg.result_calib is None and time.time() < deadline:
        if dlg.u is not None:
            tracker.u, tracker.v = dlg.u, dlg.v
        app.processEvents()
        time.sleep(0.003)
    dlg.timer.stop()
    dlg.anim.stop()
    return dlg


def main():
    app = QtWidgets.QApplication(sys.argv)
    dialog_cls = CalibDialog.build(QtCore, QtGui, QtWidgets)

    # Capture the REAL timings before anything shortens them for the test run.
    real_spec = dict(CalibDialog.PROFILES['ultra'])
    real_seconds = len(real_spec['dots']) * (real_spec['settle']
                                             + real_spec['sample'])

    # Speed the profiles up; the timing is not what is under test.
    CalibDialog.PROFILES = {k: dict(v) for k, v in CalibDialog.PROFILES.items()}
    for k in CalibDialog.PROFILES:
        CalibDialog.PROFILES[k]['settle'] = 0.05
        CalibDialog.PROFILES[k]['sample'] = 0.40

    # ---------------- the ultra profile's shape -------------------------
    ultra = real_spec['dots']
    check('ultra really is over three minutes',
          real_seconds >= 180, '%.0f s' % real_seconds)
    check('ultra profile has 47 dots', len(ultra) == 47, '%d dots' % len(ultra))

    us = [d[1] for d in ultra]
    vs = [d[2] for d in ultra]
    check('ultra reaches the screen edges',
          min(us) <= 0.10 and max(us) >= 0.90
          and min(vs) <= 0.10 and max(vs) >= 0.90,
          'u %.2f..%.2f  v %.2f..%.2f' % (min(us), max(us), min(vs), max(vs)))
    check('ultra is weighted to the centre column',
          sum(1 for u in us if abs(u - 0.50) < 0.02) >= 12,
          '%d centre-column dots' % sum(1 for u in us if abs(u - 0.50) < 0.02))
    check('ultra samples many vertical levels',
          len(set(round(v, 3) for v in vs)) >= 12,
          '%d distinct v' % len(set(round(v, 3) for v in vs)))

    check('ultra has a long per-dot settle',
          real_spec['settle'] >= 1.5, '%.1f s' % real_spec['settle'])

    # Every move between consecutive dots must be short, or the operator is
    # still catching up when sampling starts.
    import math
    jumps = [math.hypot(ultra[i + 1][1] - ultra[i][1],
                        ultra[i + 1][2] - ultra[i][2])
             for i in range(len(ultra) - 1)]
    check('consecutive dots are neighbours, not corner jumps',
          max(jumps) < 0.35, 'max move %.2f of the screen' % max(jumps))

    # ---------------- a good tracker must report a good fit --------------
    good = Tracker(0.62, 0.55, 2.0, 1.2)
    dlg = run(dialog_cls, app, good, 'long')
    if dlg.result_calib is None:
        check('good tracker completes', False, 'phase=%s' % dlg.phase)
    else:
        q = dlg.quality
        check('good tracker completes', True)
        check('good tracker reports a small fit error',
              q['rms_u'] < 0.02 and q['rms_v'] < 0.02,
              'u %.3f  v %.3f' % (q['rms_u'], q['rms_v']))
        check('good tracker is not flagged as clamped',
              not q['clamped_u'] and not q['clamped_v'])
        verdict = dlg._verdict(q['rms_u'], q['rms_v'],
                               dlg.result_calib[2], dlg.result_calib[3])
        check('good tracker gets a positive verdict',
              'Apply' in verdict, verdict)

    # ---------------- a tracker with no vertical range must be caught -----
    # `g` here is the tracker's raw-per-target ratio, so the fitted gain is
    # exactly g. A WEAK vertical signal is one where the raw value barely moves
    # for a large change in where the operator looks - so g is LARGE, the
    # fitted gain is large, and it pins at GAIN_MAX. That is the condition that
    # produced "gain 3.04, 0.25" on the real hardware.
    weak = Tracker(0.62, 0.55, 2.0, 40.0)
    dlg = run(dialog_cls, app, weak, 'long')
    if dlg.result_calib is None:
        check('weak-vertical tracker completes', False, 'phase=%s' % dlg.phase)
    else:
        q = dlg.quality
        check('weak-vertical tracker completes', True)
        check('weak vertical signal is reported as clamped', q['clamped_v'],
              'gain %.2f' % dlg.result_calib[3])
        verdict = dlg._verdict(q['rms_u'], q['rms_v'],
                               dlg.result_calib[2], dlg.result_calib[3])
        check('weak vertical signal is called out, not passed',
              'too weak' in verdict, verdict)
        check('a clamped fit does not claim to be excellent',
              'Excellent' not in verdict)

    # And the opposite failure: a wild, noise-dominated signal pins at GAIN_MIN.
    wild = Tracker(0.62, 0.55, 2.0, 0.01)
    dlg = run(dialog_cls, app, wild, 'long')
    if dlg.result_calib is not None:
        q = dlg.quality
        check('wild vertical signal is reported as clamped', q['clamped_v'],
              'gain %.2f' % dlg.result_calib[3])
        verdict = dlg._verdict(q['rms_u'], q['rms_v'],
                               dlg.result_calib[2], dlg.result_calib[3])
        check('wild vertical signal is called out, not passed',
              'wild' in verdict, verdict)

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
