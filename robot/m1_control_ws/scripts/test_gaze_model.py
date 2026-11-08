"""Compares the ten-feature ridge model against the old offset+gain model.

    python3 scripts/test_gaze_model.py

This is the test that answers "why does my friend's work and mine does not".

The old model was two parameters per axis:

    corrected = (raw - offset) * gain + 0.5

a straight line through one input. The new model is a ridge regression over ten
features including head rotation and head POSITION.

On a synthetic operator whose eye signal is LINEAR in where they are looking,
the old model is merely worse (15 px vs 3 px). On an operator whose eye signal
SATURATES - which is what real eyes do, they rotate about 30 degrees and then
the head takes over - the old model is **110 px** wrong while the ridge model
is **8 px**.

That is the difference between "the calibration is a bit off" and "the robot
turns the wrong way". And it is why four rewrites of the calibration maths
never helped: the maths was fine, the input was insufficient.

The second thing checked here is that head columns which did NOT move during
calibration are ignored rather than given a coefficient fitted to noise.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

import numpy as np                                               # noqa: E402

from gaze_model import GazeModel                                 # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


def synth(n_dots=60, samples=20, head_moves=True, saturate=True, seed=11):
    """A fake operator.

    `saturate` models the real eye: it rotates about 30 degrees and then stops,
    so past that the eye signal is flat and only head movement carries
    information. That is precisely what a straight line cannot represent.
    """
    rng = np.random.default_rng(seed)
    X, Y, G = [], [], []
    for g in range(n_dots):
        u, v = rng.uniform(0.06, 0.94, 2)
        yaw = (u - 0.5) * 0.55
        pitch = (0.5 - v) * 0.42
        hz = 55.0 + (rng.uniform(-14, 14) if head_moves else 0.0)
        for _ in range(samples):
            if saturate:
                eye_h = 0.30 * np.tanh((u - 0.5) * 6.0)
                eye_v = 0.24 * np.tanh((0.5 - v) * 6.0)
            else:
                eye_h = (u - 0.5) * 0.6
                eye_v = (0.5 - v) * 0.5
            X.append([eye_h + rng.normal(0, 0.003),
                      eye_v + rng.normal(0, 0.003),
                      0.3,
                      eye_h * 1.1 + rng.normal(0, 0.006),
                      eye_v * 1.1 + rng.normal(0, 0.006),
                      yaw + rng.normal(0, 0.004),
                      pitch + rng.normal(0, 0.004),
                      rng.normal(0, 0.2), rng.normal(0, 0.2), hz])
            Y.append([u, v])
            G.append(g)
    return np.array(X), np.array(Y), np.array(G)


def old_model_error(X, Y):
    """Offset+gain per axis, fitted on the two eye columns. The old approach."""
    raw = X[:, [3, 4]]
    total = 0.0
    for ax in (0, 1):
        p, t = raw[:, ax], Y[:, ax]
        gain, off = np.polyfit(p, t, 1)
        total += float(np.abs(gain * p + off - t).mean())
    return total / 2.0


def main():
    PX = 1920.0     # a maximised window, so errors read in pixels

    # ---------------- 1. the saturating case: the real one --------------
    X, Y, G = synth(saturate=True)
    ridge = GazeModel()
    loo = ridge.fit(X, Y, G)
    old = old_model_error(X, Y)

    print('saturating eye signal (real eyes):')
    print('  ridge (10 features)   %.4f = %3.0f px' % (loo, loo * PX))
    print('  old offset+gain       %.4f = %3.0f px' % (old, old * PX))
    check('ridge beats offset+gain on a saturating eye signal',
          loo < old / 4.0, '%.0f px vs %.0f px' % (loo * PX, old * PX))
    check('the ridge model is accurate in absolute terms',
          loo * PX < 25, '%.0f px' % (loo * PX))

    # ---------------- 2. the easy case: still better --------------------
    X2, Y2, G2 = synth(saturate=False)
    r2 = GazeModel()
    loo2 = r2.fit(X2, Y2, G2)
    old2 = old_model_error(X2, Y2)
    print('\nlinear eye signal:')
    print('  ridge                 %.4f = %3.0f px' % (loo2, loo2 * PX))
    print('  old offset+gain       %.4f = %3.0f px' % (old2, old2 * PX))
    check('ridge also wins when the eye signal is linear', loo2 < old2,
          '%.0f px vs %.0f px' % (loo2 * PX, old2 * PX))

    # ---------------- 3. a still head must be ignored -------------------
    X3, Y3, G3 = synth(head_moves=False)
    r3 = GazeModel()
    r3.fit(X3, Y3, G3)
    print('\nhead held still during calibration:')
    print('  ignored columns: %s  (7,8,9 = head_x, head_y, head_z)'
          % r3.ignored_head)
    check('head position is ignored when it never moved',
          set(r3.ignored_head) == {7, 8, 9}, str(r3.ignored_head))
    check('head rotation is still used when it did move',
          5 not in r3.ignored_head and 6 not in r3.ignored_head,
          str(r3.ignored_head))

    # ---------------- 4. validation is honest --------------------------
    # Fit on some dots, validate on others. The validation error must be in
    # the same ballpark as the leave-one-out figure - if it is wildly larger,
    # the model is fitting noise.
    Xv, Yv, Gv = synth(n_dots=80, seed=23)
    train = Gv < 60
    model = GazeModel()
    model.fit(Xv[train], Yv[train], Gv[train])
    err, dots = model.validate(Xv[~train], Yv[~train], Gv[~train])
    print('\nheld-out dots: %d   mean error %.4f = %.0f px   worst %.0f px'
          % (len(dots), err.mean(), err.mean() * PX, err.max() * PX))
    check('held-out error is close to the leave-one-out estimate',
          err.mean() < max(model.error * 3.0, 0.02),
          'held-out %.4f vs loo %.4f' % (err.mean(), model.error))
    check('no held-out dot is wildly wrong', err.max() * PX < 120,
          'worst %.0f px' % (err.max() * PX))

    # ---------------- 5. save and load round-trip ----------------------
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / 'calib.json'
        ridge.save(p)
        back = GazeModel.load(p)
        a = ridge.predict(X[0])
        b = back.predict(X[0])
        check('save/load preserves predictions',
              abs(a[0] - b[0]) < 1e-9 and abs(a[1] - b[1]) < 1e-9,
              '%.6f,%.6f vs %.6f,%.6f' % (a[0], a[1], b[0], b[1]))
        check('a missing file loads as "not calibrated"',
              not GazeModel.load(Path(d) / 'nope.json').is_ready())

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
