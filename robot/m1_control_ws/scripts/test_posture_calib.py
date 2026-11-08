#!/usr/bin/env python3
"""Does multi-posture calibration actually beat single-posture?

This is the question the whole rewrite hinges on, so it is worth answering
with a number rather than an opinion.

A synthetic operator is built with the properties that make gaze hard:

  * the irises move a little, and SATURATE - they cannot leave the eye, so
    beyond about 15 degrees of look angle the eye signal stops growing
  * the head turns to cover the rest of the range, so the same screen point is
    reached by a different mix of eye and head depending on where the head
    happens to be
  * the head sits at four different distances and offsets, and it MOVES during
    the run

Then two calibrations are fitted and compared on points neither has seen:

  single posture   20 dots, head held still      (the old approach)
  four postures    80 dots, head at four places  (the new approach)

Both use the SAME ridge model, so any difference is the calibration data and
not the maths. That is the point: the maths was never the problem.

Run:
    python3 scripts/test_posture_calib.py
"""

from __future__ import annotations

import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'm1_control',
                                'm1_control'))

import numpy as np                                          # noqa: E402

from gaze_model import GazeModel                            # noqa: E402

# ---------------------------------------------------------------------------
# The synthetic operator
# ---------------------------------------------------------------------------

#: Beyond this look angle the iris cannot move any further inside the eye.
#: Real eyes saturate hard; this single fact is why a two-parameter line
#: through the eye signal cannot work.
SATURATION_RAD = math.radians(14.0)

#: How far the head turns, at most.
HEAD_YAW_RAD = math.radians(20.0)
HEAD_PITCH_RAD = math.radians(12.0)


def gaze_angles(target_u, target_v, head_yaw, head_pitch):
    """Where the EYES have to point, given where the head already points.

    The screen point is reached by the head and the eyes together, so the eye
    angle is whatever the head did not cover. This is the coupling that makes
    the problem non-linear in every individual feature.
    """
    want_yaw = (target_u - 0.5) * math.radians(46.0)
    want_pitch = (target_v - 0.5) * math.radians(30.0)
    return want_yaw - head_yaw, want_pitch - head_pitch


def saturate(x, limit=SATURATION_RAD):
    """Iris travel is bounded. This is the physical non-linearity."""
    return limit * math.tanh(x / limit)


def make_sample(rng, target_u, target_v, posture):
    """One frame of the ten features, for a given screen point and posture."""
    # Posture: (yaw, pitch, distance, offset_x, offset_y)
    pyaw, ppitch, dist, offx, offy = posture
    # The head is never perfectly still, and the operator is not a statue.
    yaw = pyaw + rng.gauss(0.0, math.radians(1.5))
    pitch = ppitch + rng.gauss(0.0, math.radians(1.0))
    dist = dist + rng.gauss(0.0, 0.4)
    offx = offx + rng.gauss(0.0, 0.2)
    offy = offy + rng.gauss(0.0, 0.2)

    eye_yaw, eye_pitch = gaze_angles(target_u, target_v, yaw, pitch)

    # Iris offset inside the eye, normalised by eye width. Saturating.
    eye_h = saturate(eye_yaw) / math.radians(40.0)
    eye_v = saturate(eye_pitch) / math.radians(40.0)

    # The blendshape the tracker reports for eye direction - it sees the same
    # thing the iris does, so it saturates the same way, with its own noise.
    look_h = eye_h * 0.95 + rng.gauss(0.0, 0.004)
    look_v = eye_v * 0.92 + rng.gauss(0.0, 0.004)

    # Eyelid opening: narrows when the eye turns down or the head pitches down.
    eye_open = 0.42 - 0.25 * abs(eye_pitch) / math.radians(30.0)
    eye_open = max(0.05, eye_open + rng.gauss(0.0, 0.01))

    # Eye WIDTH is a proxy for distance: closer face, bigger eyes in frame.
    eye_w = 0.085 * (55.0 / dist) + rng.gauss(0.0, 0.0008)

    return np.array([eye_h, eye_v, eye_open, look_h, look_v,
                     yaw, pitch,
                     offx, offy, dist], dtype=float)


#: The four postures the dialog asks for.
POSTURES = [
    (0.0, 0.0, 55.0, 0.0, 0.0),                              # sit normally
    (0.0, math.radians(-6.0), 72.0, 0.0, 0.0),               # lean back
    (0.0, math.radians(5.0), 42.0, 0.0, 0.0),                # lean in
    (math.radians(9.0), 0.0, 55.0, 9.0, 0.0),                # shift sideways
]


def jittered_grid(seed, cols=5, rows=4, margin=0.06):
    """Same generator the dialog uses, so the test calibrates like the run."""
    rng = random.Random(seed)
    dots = []
    span_x = (1.0 - 2.0 * margin) / cols
    span_y = (1.0 - 2.0 * margin) / rows
    cells = [(c, r) for r in range(rows) for c in range(cols)]
    cells.sort(key=lambda cr: ((cr[0] - (cols - 1) / 2.0) ** 2
                               + (cr[1] - (rows - 1) / 2.0) ** 2))
    for c, r in cells:
        u = margin + span_x * (c + rng.uniform(0.15, 0.85))
        v = margin + span_y * (r + rng.uniform(0.15, 0.85))
        dots.append((u, v))
    return dots


def collect(rng, dots, posture, per_dot=12):
    """Sample one dot: `per_dot` frames, exactly as the dialog does."""
    X, Y, G = [], [], []
    for gi, (u, v) in enumerate(dots):
        frames = [make_sample(rng, u, v, posture) for _ in range(per_dot)]
        # The dialog medians the frames, so the test must too.
        X.append(np.median(np.array(frames), axis=0))
        Y.append((u, v))
        G.append(gi)
    return X, Y, G


def held_out_error(model, rng, n=60):
    """Error on points the model never saw, in pixels of a 1920-wide screen."""
    errs = []
    for _ in range(n):
        u = rng.uniform(0.06, 0.94)
        v = rng.uniform(0.06, 0.94)
        # The test looks at all four postures, because the operator will be in
        # all four - that is the whole point.
        for posture in POSTURES:
            x = np.median(np.array([make_sample(rng, u, v, posture)
                                    for _ in range(12)]), axis=0)
            pu, pv = model.predict(x)
            errs.append(math.hypot((pu - u) * 1920.0, (pv - v) * 1080.0))
    return float(np.mean(errs)), float(np.percentile(errs, 95))


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    print('=' * 68)
    print('single posture vs four postures, same model, same maths')
    print('=' * 68)

    rng = random.Random(7)

    # ---- single posture: 20 dots, sitting normally -----------------------
    dots1 = jittered_grid(seed=1000)
    X1, Y1, G1 = collect(rng, dots1, POSTURES[0])
    m1 = GazeModel()
    m1.fit(X1, Y1, G1)
    mean1, p95_1 = held_out_error(m1, rng)
    print(f'\nsingle posture ({len(X1)} dots, head still):')
    print(f'  held-out error  mean {mean1:6.1f} px   p95 {p95_1:6.1f} px')

    # ---- four postures: 80 dots ------------------------------------------
    X4, Y4, G4 = [], [], []
    gi = 0
    for pi, posture in enumerate(POSTURES):
        Xp, Yp, _Gp = collect(rng, jittered_grid(seed=1000 + pi), posture)
        for x, y in zip(Xp, Yp):
            X4.append(x)
            Y4.append(y)
            G4.append(gi)
            gi += 1
    m4 = GazeModel()
    m4.fit(X4, Y4, G4)
    mean4, p95_4 = held_out_error(m4, rng)
    print(f'\nfour postures ({len(X4)} dots, head in four places):')
    print(f'  held-out error  mean {mean4:6.1f} px   p95 {p95_4:6.1f} px')

    # ---- the comparison ---------------------------------------------------
    gain = mean1 / max(mean4, 1e-6)
    print(f'\nfour postures is {gain:.1f}x better on unseen points\n')

    print('checks:')
    check('four postures beats one posture', mean4 < mean1,
          f'{mean4:.1f} px vs {mean1:.1f} px')
    check('four postures is at least 2x better', gain >= 2.0,
          f'{gain:.1f}x')
    check('four-posture error is usable (<80 px mean)', mean4 < 80.0,
          f'{mean4:.1f} px')
    check('four-posture p95 is usable (<160 px)', p95_4 < 160.0,
          f'{p95_4:.1f} px')
    check('single posture is NOT usable', mean1 > 80.0,
          f'{mean1:.1f} px - this is what the operator was living with')

    # The head columns must survive: the whole gain comes from them.
    kept = [i for i in range(10) if i not in m4.ignored_head]
    print(f'\n  features the model kept: {kept}')
    check('model kept the head-pose columns', 5 in kept and 6 in kept,
          'yaw, pitch')
    check('model kept the eye-direction columns', 0 in kept and 1 in kept,
          'eye_h, eye_v')
    check('model kept distance', 9 in kept, 'head_z / distance')

    print()
    print('=' * 68)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 68)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
