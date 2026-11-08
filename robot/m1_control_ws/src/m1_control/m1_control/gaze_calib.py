#!/usr/bin/env python3
"""The gaze calibration maths, on its own.

No ROS, no Qt, no numpy - so it can be reasoned about and tested without a
robot, a webcam or a display. Both the console (which runs the calibration
UI) and the HUD (which applies the result) use these functions, which is the
only way the two are guaranteed to agree on what the four numbers mean.

The model, per axis:

    corrected = (raw - offset) * gain + 0.5

`offset` is where this operator's eyes report "straight ahead" - a resting
bias, typically 0.55-0.65 rather than 0.5, because nobody's iris sits exactly
centred. `gain` rescales the eye travel: a comfortable glance may only move
the tracker across 0.3 of its range, and the robot needs 0.7.

Fitting
-------
For a dot shown at target `t`:

    t = gain * raw + b        =>   gain = slope,  offset = (0.5 - b) / gain

so a least-squares line of target against raw gives both numbers, for any
number of dots. That is why the 4-dot "fast" profile is a real calibration and
not an approximation: the model only has two unknowns per axis, and four dots
over-determine them.
"""

#: Gains outside this range are refused. A tracker that barely moved between
#: the dots produces a huge gain, and a huge gain turns tracking noise into
#: servo twitch and phantom driving.
GAIN_MIN = 0.25
GAIN_MAX = 5.0

#: corrected = (raw - offset) * gain + 0.5 with these is the identity:
#: (raw - 0.5) + 0.5 == raw. Sending offset 0.0 instead would add 0.5 to every
#: sample and slam the output to 1.0 - "reset" would look like a hard turn.
NEUTRAL = (0.5, 0.5, 1.0, 1.0)


def _slope(pairs):
    """Least-squares slope of y against x for a list of (x, y)."""
    n = len(pairs)
    if n < 2:
        return 1.0
    sx = sum(p[0] for p in pairs)
    sy = sum(p[1] for p in pairs)
    sxx = sum(p[0] * p[0] for p in pairs)
    sxy = sum(p[0] * p[1] for p in pairs)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        # The tracker reported the same value for every dot, so there is no
        # slope to learn. Unity is the honest answer; dividing would explode.
        return 1.0
    return (n * sxy - sx * sy) / den


def _fit_once(samples, targets):
    """Plain least squares, no robustness."""
    n = len(samples)
    mu = sum(s[0] for s in samples) / n
    mv = sum(s[1] for s in samples) / n
    tu = sum(t[0] for t in targets) / n
    tv = sum(t[1] for t in targets) / n

    gx = _slope([(s[0], t[0]) for s, t in zip(samples, targets)])
    gy = _slope([(s[1], t[1]) for s, t in zip(samples, targets)])

    gx = min(GAIN_MAX, max(GAIN_MIN, gx))
    gy = min(GAIN_MAX, max(GAIN_MIN, gy))

    # Pin the line through the centroid rather than using the raw intercept:
    # when the gain is clamped, the centroid is the one property that must
    # survive, otherwise "neutral" drifts off centre and the robot pulls to
    # one side while the operator stares straight ahead.
    bx = tu - gx * mu
    by = tv - gy * mv

    return ((0.5 - bx) / gx, (0.5 - by) / gy, gx, gy)


def _residual(sample, target, calib):
    cu, cv = apply(sample[0], sample[1], calib)
    return abs(cu - target[0]) + abs(cv - target[1])


#: A dot whose residual is this many times the median is treated as "the
#: operator was not looking at it" and dropped. 3x is loose enough that honest
#: tracker jitter never triggers it.
OUTLIER_FACTOR = 3.0


def fit(samples, targets):
    """Fit (ox, oy, gx, gy).

    samples  [(raw_u, raw_v), ...] - one mean per dot, in dot order
    targets  [(u, v), ...]         - where each dot was shown, same order

    Dots that fit far worse than the others are dropped and the fit repeated.
    Least squares is not robust: one dot where the operator blinked, glanced at
    the robot, or looked at the previous dot's position drags the whole line.
    Measured on the 9-dot profile, a single bad dot moved the fitted gain from
    2.00 to 1.26 - a 37% error that would feel like the head barely moving.

    With only four dots there is no redundancy to tell which one is wrong, so
    nothing is dropped: that is the fast profile's whole trade-off, and why the
    long one exists.
    """
    if len(samples) < 2 or len(samples) != len(targets):
        raise ValueError('fit() needs one target per sample, at least two')

    keep = list(range(len(samples)))
    calib = _fit_once(samples, targets)

    # Two passes is enough: one grossly wrong dot is the realistic case, and
    # each pass needs the others to be good to know which one is bad.
    for _ in range(2):
        if len(keep) < 5:
            break
        res = [_residual(samples[i], targets[i], calib) for i in keep]
        ordered = sorted(res)
        med = ordered[len(ordered) // 2]
        if med <= 1e-6:
            break
        worst = max(range(len(res)), key=lambda k: res[k])
        if res[worst] <= OUTLIER_FACTOR * med:
            break
        keep.pop(worst)
        calib = _fit_once([samples[i] for i in keep],
                          [targets[i] for i in keep])

    return calib


def apply(u, v, calib):
    """Raw tracker point -> the point the operator actually meant.

    Clamped after correction: a bad gain must not be able to push the point
    outside the unit square, where the zone lookup falls off the end.
    """
    ox, oy, gx, gy = calib
    cu = (u - ox) * gx + 0.5
    cv = (v - oy) * gy + 0.5
    return (min(1.0, max(0.0, cu)), min(1.0, max(0.0, cv)))


def is_neutral(calib, tol=1e-6):
    return all(abs(a - b) <= tol for a, b in zip(calib, NEUTRAL))
