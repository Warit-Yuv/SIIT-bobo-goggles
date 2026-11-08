#!/usr/bin/env python3
"""Checks the gaze calibration maths against trackers with known behaviour.

Runs on the Pi with plain python3 - no ROS, no Qt, no webcam - because
gaze_calib.py deliberately imports nothing.

    python3 scripts/test_gaze_calib.py

The point is not that the numbers look plausible. It is that a tracker whose
offset and gain we CHOSE comes back out of the fit with those exact values,
for both profiles. A fit that is merely close would still leave the operator
fighting a constant drift, which is the failure this is meant to end.
"""

import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'
                              / 'm1_control' / 'm1_control'))

from gaze_calib import (GAIN_MAX, GAIN_MIN, NEUTRAL, apply, fit)  # noqa: E402

FAST = [(0.50, 0.50), (0.50, 0.15), (0.85, 0.50),
        (0.50, 0.85), (0.15, 0.50)]
LONG = [(0.15, 0.15), (0.50, 0.15), (0.85, 0.15),
        (0.15, 0.50), (0.50, 0.50), (0.85, 0.50),
        (0.15, 0.85), (0.50, 0.85), (0.85, 0.85)]

failures = []


def check(name, ok, detail=''):
    print(f'{"PASS" if ok else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not ok:
        failures.append(name)


def raw_for(targets, ox, oy, gx, gy):
    """Invert  corrected = (raw - offset) * gain + 0.5  to get tracker output."""
    return [((t[0] - 0.5) / gx + ox, (t[1] - 0.5) / gy + oy) for t in targets]


def main():
    # ---- 1. exact recovery over a grid of plausible trackers ----------------
    worst = 0.0
    for ox, oy, gx, gy in itertools.product(
            (0.50, 0.58, 0.65),      # resting bias, incl. the u=0.62 case
            (0.45, 0.55),
            (0.60, 1.00, 2.00, 3.50),  # comfortable glance .. narrow range
            (0.80, 1.00, 1.80)):
        for dots in (FAST, LONG):
            raw = raw_for(dots, ox, oy, gx, gy)
            calib = fit(raw, dots)
            for r, t in zip(raw, dots):
                cu, cv = apply(r[0], r[1], calib)
                worst = max(worst, abs(cu - t[0]), abs(cv - t[1]))
    check('fit recovers the tracker it was given', worst < 1e-9,
          f'worst residual {worst:.2e} of full range')

    # ---- 2. a perfect tracker must give the neutral calibration ------------
    for dots in (FAST, LONG):
        got = fit(dots, dots)
        check(f'perfect tracker -> neutral ({len(dots)} dots)',
              all(abs(a - b) < 1e-9 for a, b in zip(got, NEUTRAL)),
              f'got {tuple(round(v, 4) for v in got)}')

    # ---- 3. a dead tracker must not explode --------------------------------
    dead = [(0.50, 0.50)] * len(FAST)
    got = fit(dead, FAST)
    check('dead tracker is refused, not amplified',
          all(abs(a - b) < 1e-9 for a, b in zip(got, NEUTRAL)),
          f'got {tuple(round(v, 4) for v in got)}')

    # ---- 4. one bad dot: long profile must survive it ----------------------
    truth = (0.60, 0.50, 2.00, 1.00)
    clean = raw_for(LONG, *truth)
    bad = list(clean)
    bad[4] = (0.95, 0.05)          # operator looked away for the centre dot
    got = fit(bad, LONG)
    err = max(abs(got[2] - truth[2]), abs(got[3] - truth[3]))
    check('long profile rejects one bad dot', err < 0.05,
          f'gain {got[2]:.3f}/{got[3]:.3f} vs true {truth[2]:.2f}/{truth[3]:.2f}')

    # ---- 5. fast profile keeps all four (no redundancy to judge with) ------
    fast_bad = raw_for(FAST, *truth)
    fast_bad[0] = (0.95, 0.05)
    got = fit(fast_bad, FAST)
    check('fast profile still returns usable numbers with a bad dot',
          GAIN_MIN <= got[2] <= GAIN_MAX and GAIN_MIN <= got[3] <= GAIN_MAX,
          f'gain {got[2]:.2f}/{got[3]:.2f} (degraded on purpose, not refused)')

    # ---- 6. clamping keeps neutral centred --------------------------------
    # A tracker that barely moves gives a gain far above the cap. The cap must
    # not shift the point where "straight ahead" lands.
    tiny = raw_for(LONG, 0.60, 0.50, 0.02, 0.02)
    got = fit(tiny, LONG)
    cu, cv = apply(0.60, 0.50, got)      # the operator's resting gaze
    check('clamped gain still puts rest at centre',
          abs(cu - 0.5) < 1e-6 and abs(cv - 0.5) < 1e-6,
          f'rest maps to ({cu:.4f}, {cv:.4f}), gain {got[2]:.1f}/{got[3]:.1f}')

    # ---- 7. the historical bug: offset 0.5 subtraction --------------------
    # corrected = (raw - 0.5) + 0.5 would report 1.0 for a raw of 1.0 and pin
    # the robot to one side. Confirm the fit never produces that.
    got = fit(raw_for(LONG, 0.62, 0.50, 1.0, 1.0), LONG)
    cu, _ = apply(0.62, 0.50, got)
    check('resting gaze maps to centre, not to an edge', abs(cu - 0.5) < 1e-6,
          f'raw 0.62 -> {cu:.4f} (offset fitted {got[0]:+.3f})')

    print()
    if failures:
        print(f'{len(failures)} FAILED: ' + ', '.join(failures))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
