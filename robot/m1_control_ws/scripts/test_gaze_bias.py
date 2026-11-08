"""Checks that a BIASED tracker still produces a usable, distinct signal.

    python3 scripts/test_gaze_bias.py

This reproduces the failure seen on the hardware, where the Gaze panel read:

    raw 0.55, 0.01        <- vertical pinned at the bottom rail
    gain 1.00, 1.00       <- the fit found no relationship at all
    CENTRE 0  TOP 154  BOTTOM 454  LEFT 906  RIGHT 806

Two separate clamps caused it, one after the other:

1. The eye term was stretched AFTER blending with the head term, so anything
   more than 0.167 from centre reported exactly 0.0 or 1.0.

2. The head term was clipped to +/-1 before scaling, against a span of only
   16 degrees of pitch. solvePnP reports an ABSOLUTE angle, and that angle
   carries a constant bias from the camera sitting above or below eye level
   and from the generic face model not matching the person. A 20 degree bias
   against a 16 degree span sits permanently on the rail, so the head term
   became a constant and the vertical axis reported one number forever.

A constant bias is the normal case, not an edge case, so the pipeline must
survive it. That means:

  * a biased tracker must still produce DISTINCT values for distinct targets
  * the calibration must recover a usable gain from it
  * and it must work at any bias, not just a convenient one
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

import numpy as np                                               # noqa: E402

import gaze_hud_node as g                                        # noqa: E402
from gaze_calib import apply as apply_calib                      # noqa: E402
from gaze_calib import fit as fit_calib                          # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


class FakeNode:
    """Only the attributes _blend_head touches."""

    def __init__(self, head_weight=0.75):
        self.head_weight = head_weight
        self.head_yaw = 0.0
        self.head_pitch = 0.0


def blend(node, yaw_deg, pitch_deg, eye_u=0.5, eye_v=0.5):
    """Run the real _blend_head with a chosen pose, bypassing solvePnP.

    The pose is injected on the instance rather than faked through landmarks so
    the test can sweep bias directly; the solver itself is covered by
    test_gaze_pose.py.
    """
    node._head_rotation = lambda lm, w, h: (math.radians(yaw_deg),
                                            math.radians(pitch_deg))
    return g.GazeHUDNode._blend_head(node, eye_u, eye_v, None, 640, 480)


def main():
    node = FakeNode()

    # ---------------- 1. a bias must not pin the signal -----------------
    print('%-14s %8s %8s' % ('pitch bias', 'v(up 15)', 'v(down 15)'))
    worst_span = 1.0
    for bias in (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0):
        _u, v_up = blend(node, 0.0, 15.0 + bias)
        _u, v_down = blend(node, 0.0, -15.0 + bias)
        span = v_down - v_up
        worst_span = min(worst_span, span)
        print('%-14.1f %8.3f %8.3f   span %.3f' % (bias, v_up, v_down, span))

    check('every head-pitch bias keeps a usable vertical span',
          worst_span > 0.20,
          'smallest span %.3f of the range' % worst_span)

    # The whole point: at the bias that broke it, the two directions must
    # still be tellable apart.
    _u, v_up = blend(node, 0.0, 35.0)      # +20 bias, looking up
    _u, v_down = blend(node, 0.0, 5.0)     # +20 bias, looking down
    check('at +20 deg bias, up and down still differ',
          abs(v_down - v_up) > 0.20,
          'up %.3f  down %.3f' % (v_up, v_down))
    check('the biased signal is NOT pinned at a rail',
          v_up != 0.0 or v_down != 1.0,
          'up %.3f  down %.3f' % (v_up, v_down))

    # ---------------- 2. a biased tracker must calibrate -----------------
    targets = [(0.50, 0.50), (0.50, 0.08), (0.92, 0.50),
               (0.50, 0.92), (0.08, 0.50)]
    bias_pitch = 20.0

    samples = []
    for u, v in targets:
        # A real operator looking at the target: head pitch moves with the
        # target, eyes contribute a little.
        pitch = (0.5 - v) * 40.0 + bias_pitch
        yaw = (u - 0.5) * 44.0
        bu, bv = blend(node, yaw, pitch)
        samples.append((bu, bv))

    calib = fit_calib(samples, targets)
    ox, oy, gx, gy = calib
    print('\nbiased tracker -> offset=(%+.3f,%+.3f) gain=(%.2f,%.2f)'
          % (ox, oy, gx, gy))

    check('a biased tracker still yields a finite, non-identity gain',
          not (abs(gx - 1.0) < 1e-9 and abs(gy - 1.0) < 1e-9),
          'gain=(%.2f, %.2f)' % (gx, gy))

    # The fitted model must reproduce the targets it was given.
    worst = 0.0
    for (u, v), (bu, bv) in zip(targets, samples):
        cu, cv = apply_calib(bu, bv, calib)
        worst = max(worst, abs(cu - u), abs(cv - v))
    check('the biased tracker calibrates to its own dots',
          worst < 0.02, 'worst residual %.3f' % worst)

    # ---------------- 3. the corrected output stays in range ------------
    # Whatever the raw range, the value that steers must be 0..1.
    for u, v in ((0.5, 0.5), (0.0, 0.0), (1.0, 1.0), (0.5, 0.08)):
        pitch = (0.5 - v) * 40.0 + bias_pitch
        yaw = (u - 0.5) * 44.0
        bu, bv = blend(node, yaw, pitch)
        cu, cv = apply_calib(bu, bv, calib)
        if not (0.0 <= cu <= 1.0 and 0.0 <= cv <= 1.0):
            check('corrected output stays inside 0..1', False,
                  'got (%.2f, %.2f)' % (cu, cv))
            break
    else:
        check('corrected output stays inside 0..1', True)

    # ---------------- 4. head weight 0 and 1 remain sane -----------------
    zero = FakeNode(head_weight=0.0)
    _u, v0 = blend(zero, 0.0, 45.0, eye_v=0.5)
    check('at head_weight 0 the head is ignored',
          abs(v0 - 0.5) < 1e-9, 'v %.3f' % v0)

    one = FakeNode(head_weight=1.0)
    _u, va = blend(one, 0.0, 15.0)
    _u, vb = blend(one, 0.0, -15.0)
    # +/-15 deg against a 16 deg span is 0.9375 of the range, not the full 1.0.
    # Asserting 1.0 would be asserting the span is 15 deg, which it is not.
    expect = 2.0 * 15.0 / math.degrees(g.HEAD_PITCH_SPAN) * 0.5
    check('at head_weight 1 the eyes are ignored but range survives',
          abs((vb - va) - expect) < 1e-6,
          'span %.3f, expected %.3f' % (vb - va, expect))

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
