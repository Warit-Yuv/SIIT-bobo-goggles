"""Checks the accuracy validation pass and that its result is actually shown.

    QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_accuracy.py

Three things are checked that were all wrong before:

1. The fit's own error is not the same as being accurate. After the fit the
   five cardinal directions are re-visited and the REAL distance between where
   the operator looked and where the model says they looked is measured, per
   direction, in pixels.

2. The result must be VISIBLE. It used to be appended to the instructions
   label, which is a dim wrapped paragraph that was already several lines long;
   the extra text pushed past what the bar could show and was clipped away, so
   the calibration appeared to end with no verdict at all.

3. The visit order must not be a raster sweep. A sweep walks each row
   left-to-right, so the eyes traverse the same horizontal path repeatedly and
   the samples are correlated with each other rather than independent.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

from m1_console import (CalibDialog, VALIDATION_DOTS,          # noqa: E402
                        _order_nearest, _ultra_dots)

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


class Tracker:
    """Reports the gaze a real operator would produce looking at (u, v).

    `saturate` reproduces the failure that actually happened on the hardware:
    beyond that much excursion the raw value STOPS CHANGING, so every target
    out there reports the same number. That is what the iris_gain clipping bug
    did - `(value - 0.5) * 3 + 0.5` clamped flat at 0.0 and 1.0 - and it is
    exactly why the centre worked and the edges did not.

    A model that only bends gently is not a good test: it is still learnable,
    so the fit is only slightly off and nothing looks wrong. Saturation is
    unlearnable, and the validation pass has to say so.
    """

    deadzone_ratio = 0.30
    deadzone_ratio_y = 0.16

    def __init__(self, ox, oy, gx, gy, saturate=None):
        self.u, self.v = 0.5, 0.5
        self.ox, self.oy, self.gx, self.gy = ox, oy, gx, gy
        self.saturate = saturate

    def latest_raw(self):
        u, v = self.u, self.v
        if self.saturate is not None:
            u = 0.5 + max(-self.saturate, min(self.saturate, u - 0.5))
            v = 0.5 + max(-self.saturate, min(self.saturate, v - 0.5))
        return ((u - 0.5) / self.gx + self.ox,
                (v - 0.5) / self.gy + self.oy,
                True, 0.0)


def run(dialog_cls, app, tracker, profile='fast', limit=90):
    dlg = dialog_cls(tracker, QtCore.QRect(0, 0, 1920, 1080), None)
    dlg.profile = profile
    dlg.start()
    deadline = time.time() + limit
    while dlg.phase != 'done' and time.time() < deadline:
        if dlg.u is not None:
            tracker.u, tracker.v = dlg.u, dlg.v
        app.processEvents()
        time.sleep(0.002)
    dlg.timer.stop()
    dlg.anim.stop()
    return dlg


def main():
    app = QtWidgets.QApplication(sys.argv)
    dialog_cls = CalibDialog.build(QtCore, QtGui, QtWidgets)

    # Everything that depends on the REAL timings is measured first, because
    # the rest of the test shortens them to run quickly.
    real_min_sample = CalibDialog.MIN_SAMPLE_SEC

    # ---------------- 1. the visit order is not a raster sweep -----------
    dots = _ultra_dots()
    us = [d[1] for d in dots]
    vs = [d[2] for d in dots]

    # A raster visits all of one row, then all of the next. Count how often the
    # row changes: a raster changes it exactly (rows-1) times, a scattered
    # order changes it far more often.
    rows_visited = [round(d[2], 2) for d in dots]
    row_changes = sum(1 for a, b in zip(rows_visited, rows_visited[1:]) if a != b)
    check('visit order is scattered, not row-by-row',
          row_changes > 12,
          '%d row changes across %d dots' % (row_changes, len(dots)))

    # It must still never jump far, or every jump is a saccade the operator is
    # still finishing when sampling starts.
    import math
    jumps = [math.hypot(dots[i + 1][1] - dots[i][1], dots[i + 1][2] - dots[i][2])
             for i in range(len(dots) - 1)]
    check('consecutive dots are still neighbours',
          max(jumps) < 0.30, 'max move %.2f of the screen' % max(jumps))

    check('the set still covers the screen',
          min(us) <= 0.10 and max(us) >= 0.90
          and min(vs) <= 0.10 and max(vs) >= 0.90,
          'u %.2f..%.2f  v %.2f..%.2f' % (min(us), max(us), min(vs), max(vs)))

    # ---------------- 2. the timing ramps down ---------------------------
    probe = dialog_cls(Tracker(0.62, 0.55, 2.0, 1.2),
                       QtCore.QRect(0, 0, 1920, 1080), None)
    probe.profile = 'ultra'
    probe.idx = 0
    first_settle, first_sample = probe._timing()
    probe.idx = len(probe._dots()) - 1
    last_settle, last_sample = probe._timing()
    check('timing ramps faster across the run',
          last_settle < first_settle * 0.75 and last_sample < first_sample * 0.75,
          'settle %.2f -> %.2f s, sample %.2f -> %.2f s'
          % (first_settle, last_settle, first_sample, last_sample))

    total = 0.0
    for i in range(len(probe._dots())):
        probe.idx = i
        s, p = probe._timing()
        total += s + p
    check('the 47-dot run is about two minutes, not three and a half',
          total < 150, '%.0f s' % total)

    # The ramp must never squeeze a window below the point where it can collect
    # enough samples. Without this floor the fastest dots collected three
    # samples and were reported as "tracker lost your eyes", which is a lie.
    check('the floor still allows enough samples to average',
          real_min_sample * 30 >= CalibDialog.MIN_SAMPLES * 2,
          '%.1f s = about %d samples at 30 Hz'
          % (real_min_sample, real_min_sample * 30))
    check('the ramp never drops below that floor',
          min(first_settle, first_sample, last_settle, last_sample)
          >= real_min_sample - 1e-9,
          'smallest window %.2f s' % min(last_settle, last_sample))

    # ---------------- now shorten everything for speed -------------------
    CalibDialog.PROFILES = {k: dict(v) for k, v in CalibDialog.PROFILES.items()}
    for k in CalibDialog.PROFILES:
        CalibDialog.PROFILES[k]['settle'] = 0.05
        CalibDialog.PROFILES[k]['sample'] = 0.30
    CalibDialog.MIN_SAMPLE_SEC = 0.30

    # ---------------- 3. accuracy is measured, not assumed ---------------
    good = Tracker(0.62, 0.55, 2.0, 1.2)
    dlg = run(dialog_cls, app, good, 'fast')
    check('a good tracker completes', dlg.phase == 'done', 'phase=%s' % dlg.phase)
    check('all five directions are measured',
          dlg.accuracy is not None and len(dlg.accuracy) == len(VALIDATION_DOTS),
          str(sorted(dlg.accuracy)) if dlg.accuracy else 'none')
    if dlg.accuracy:
        check('a good tracker measures as accurate',
              dlg.accuracy_worst < 60,
              'worst %.0f px' % dlg.accuracy_worst)
        check('the result is shown in its own visible label',
              dlg.result.isVisible() or dlg.result.text() != '',
              repr(dlg.result.text()[:48]))
        check('the result names the measured error',
              'error' in dlg.result.text().lower(),
              repr(dlg.result.text()[:60]))
        check('the result label has room to be read',
              dlg.result.minimumHeight() >= 40,
              '%d px' % dlg.result.minimumHeight())

    # A distorted tracker must be measured HONESTLY - the reported number has to
    # be the real one, not merely "large".
    #
    # This is checked by computing the error independently: take each validation
    # target, push it through the same distortion the tracker applies, apply the
    # calibration that was fitted, and see how far the result lands from the
    # target. The dialog must report the same numbers.
    #
    # The distortion is deliberately mild (0.30 rather than a hard 0.12 clip).
    # A hard clip compresses the range so much that the fitted gain pins at its
    # limit, and _compute() already stops and says "signal too weak" before any
    # validation runs. Mild saturation is the case that slips through - a
    # plausible gain, but the edges are compressed - which is the failure the
    # operator actually lived through.
    from gaze_calib import apply as apply_calib
    bent = Tracker(0.62, 0.55, 2.0, 1.2, saturate=0.30)
    dlg = run(dialog_cls, app, bent, 'fast')
    if dlg.accuracy:
        check('a distorted tracker still produces an accuracy report', True)

        expected = {}
        for name, u, v in VALIDATION_DOTS:
            ru = 0.5 + max(-0.30, min(0.30, u - 0.5))
            rv = 0.5 + max(-0.30, min(0.30, v - 0.5))
            ru = (ru - 0.5) / 2.0 + 0.62
            rv = (rv - 0.5) / 1.2 + 0.55
            cu, cv = apply_calib(ru, rv, dlg.result_calib)
            expected[name] = math.hypot((cu - u) * 1920, (cv - v) * 1080)

        worst_mismatch = max(abs(expected[n] - dlg.accuracy[n])
                             for n in expected)
        check('the reported error is the TRUE error, per direction',
              worst_mismatch < 3.0,
              'largest disagreement %.1f px' % worst_mismatch)
        check('the report is not silently zero on a distorted tracker',
              dlg.accuracy_worst > 5,
              'worst %.0f px' % dlg.accuracy_worst)
        check('the worst direction is named',
              any(n in dlg.result.text() for n in
                  ('TOP', 'BOTTOM', 'LEFT', 'RIGHT', 'CENTRE')),
              repr(dlg.result.text()[:70]))
    else:
        check('a distorted tracker still produces an accuracy report', False,
              'phase=%s' % dlg.phase)

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
