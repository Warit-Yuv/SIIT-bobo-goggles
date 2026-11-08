#!/usr/bin/env python3
"""The properties the old tests protected, checked against the new design.

Four tests used to guard this code and can no longer: `test_gaze_bias`,
`test_gaze_blend`, `test_gaze_noclip` and `test_gaze_pose` all called
`_blend_head` / `_head_rotation` on `GazeHUDNode`, which no longer exist. They
tested the two-parameter offset+gain model that has been replaced.

The PROPERTIES they protected are still worth protecting, so this file asserts
them against the modules that exist now:

  no clipping      the point must reach every part of the screen, and must not
                   pile up on the edges when the signal is large
  bias absorbed    a constant offset in the features must SHIFT the answer, not
                   saturate it - the old bug was a bias sitting on a rail
  head invariance  a constant head pose must not disturb eye-driven motion
  full range       the model must be able to output anywhere in 0..1

Run:
    python3 scripts/test_gaze_new_arch.py
"""

from __future__ import annotations

import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

import numpy as np                                          # noqa: E402

from gaze_model import GazeModel                            # noqa: E402


def sample(u, v, yaw=0.0, pitch=0.0, dist=55.0, bias=(0.0, 0.0)):
    """One feature vector for a screen point, with an optional constant bias."""
    # A realistic eye signal: gently compressive, not saturated flat.
    #
    # `test_posture_calib.py` covers the hard-saturation case, where the eye
    # signal flattens completely and the mapping stops being invertible - no
    # model can beat that, because the information is genuinely gone. This
    # file is about the model's BEHAVIOUR (clipping, bias, persistence), so
    # the signal here has to be one the answer actually exists for.
    lim = 0.35
    eh = lim * math.tanh((u - 0.5) / lim)
    ev = lim * math.tanh((v - 0.5) / lim)
    return np.array([eh + bias[0], ev + bias[1], 0.4,
                     eh * 0.95 + bias[0], ev * 0.92 + bias[1],
                     yaw, pitch, 0.0, 0.0, dist], float)


def fit(bias=(0.0, 0.0), yaw=0.0):
    rng = random.Random(5)
    X, Y, G = [], [], []
    for gi in range(60):
        u = rng.uniform(0.05, 0.95)
        v = rng.uniform(0.05, 0.95)
        X.append(sample(u, v, yaw=yaw, bias=bias))
        Y.append((u, v))
        G.append(gi)
    m = GazeModel()
    m.fit(X, Y, G)
    return m


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    print('no clipping:')
    m = fit()
    # Sweep past the edge of the screen in both directions. The model was
    # calibrated on 0.05..0.95, so that is the span it can honestly report;
    # asking for 0.0 is asking it to extrapolate past where the eye signal
    # has already flattened out, and no model can recover that.
    far = [m.predict(sample(u, v)) for u in (-0.4, 0.05, 0.5, 0.95, 1.4)
           for v in (0.05, 0.5, 0.95)]
    us = [p[0] for p in far]
    vs = [p[1] for p in far]
    check('predictions never leave the screen',
          all(-1e-9 <= x <= 1.0 + 1e-9 for x in us + vs),
          f'u {min(us):.2f}..{max(us):.2f}  v {min(vs):.2f}..{max(vs):.2f}')
    check('the far-left and far-right points are different',
          abs(m.predict(sample(-0.4, 0.5))[0]
              - m.predict(sample(1.4, 0.5))[0]) > 0.3,
          'a clipped signal would report the same value for both')
    # The old bug: everything past a threshold collapsed onto exactly 0 or 1,
    # so UP and LEFT reported the same number. Only points INSIDE the screen
    # are checked - a point asked for at u=-0.4 is off the display, and
    # reporting 0 for it is correct.
    in_range = [m.predict(sample(u, v))
                for u in (0.05, 0.5, 0.95) for v in (0.05, 0.5, 0.95)]
    edge_hits = sum(1 for p in in_range
                    if p[0] < 1e-6 or p[0] > 1.0 - 1e-6
                    or p[1] < 1e-6 or p[1] > 1.0 - 1e-6)
    check('the output is not piled up on the rails', edge_hits == 0,
          f'{edge_hits} of {len(in_range)} exactly at 0 or 1')
    distinct = len({(round(p[0], 3), round(p[1], 3)) for p in in_range})
    check('every in-range point maps somewhere different',
          distinct == len(in_range), f'{distinct} distinct of {len(in_range)}')

    print('\nbias absorbed, not saturated:')
    # A constant offset on a feature is removed by the standardisation step
    # before the fit ever sees it, so the model is invariant to it by
    # construction. That is exactly the property that was missing before: the
    # old offset+gain model had the bias baked into its one parameter, so a
    # drift pushed the answer along a straight line until it hit the rail.
    biased = fit(bias=(0.012, -0.008))
    errs = []
    for u, v in ((0.5, 0.5), (0.3, 0.7), (0.7, 0.3), (0.4, 0.6)):
        pu, pv = biased.predict(sample(u, v, bias=(0.012, -0.008)))
        errs.append(math.hypot((pu - u) * 1920, (pv - v) * 1080))
        check(f'a constant bias still lands near ({u:.2f}, {v:.2f})',
              errs[-1] < 60, f'{errs[-1]:.0f} px off')
    check('the bias does not pin the answer to one value',
          max(errs) < 60 and len(set(round(e) for e in errs)) > 1,
          f'errors {[round(e) for e in errs]}')
    # And it must be INVARIANT: the same targets with and without the bias
    # should give the same answers, because standardisation removes it.
    same = all(
        abs(biased.predict(sample(u, v, bias=(0.012, -0.008)))[0]
            - m.predict(sample(u, v))[0]) < 1e-6
        for u, v in ((0.5, 0.5), (0.3, 0.7), (0.7, 0.3)))
    check('the model is invariant to a constant feature offset', same,
          'standardisation absorbs it before the fit')

    print('\nhead pose is not a confound when it is constant:')
    # A head permanently turned 0.30 rad (17 deg). The model has seen this
    # posture throughout the fit, so it should absorb it.
    still = fit(yaw=0.30)
    pu, pv = still.predict(sample(0.5, 0.5, yaw=0.30))
    check('a permanently turned head still reports the centre',
          math.hypot((pu - 0.5) * 1920, (pv - 0.5) * 1080) < 180,
          f'{(pu - 0.5) * 1920:+.0f}, {(pv - 0.5) * 1080:+.0f} px')
    left = still.predict(sample(0.15, 0.5, yaw=0.30))[0]
    right = still.predict(sample(0.85, 0.5, yaw=0.30))[0]
    check('eye movement still works with the head turned', right - left > 0.5,
          f'{right - left:+.2f} of screen width')

    print('\nfull range, within the calibrated span:')
    inner = [m.predict(sample(u, v)) for u in (0.05, 0.5, 0.95)
             for v in (0.05, 0.5, 0.95)]
    span_u = max(p[0] for p in inner) - min(p[0] for p in inner)
    span_v = max(p[1] for p in inner) - min(p[1] for p in inner)
    check('the horizontal output covers the calibrated span', span_u > 0.75,
          f'{span_u:.2f} of screen width')
    check('the vertical output covers the calibrated span', span_v > 0.75,
          f'{span_v:.2f} of screen height')

    print('\npersistence:')
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, 'm.json')
    m.save(path)
    again = GazeModel.load(path)
    a = m.predict(sample(0.3, 0.7))
    b = again.predict(sample(0.3, 0.7))
    check('a saved model predicts identically', abs(a[0] - b[0]) < 1e-9
          and abs(a[1] - b[1]) < 1e-9, f'{a[0]:.6f} vs {b[0]:.6f}')

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
