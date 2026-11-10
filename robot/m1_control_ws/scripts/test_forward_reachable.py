#!/usr/bin/env python3
"""Forward must be REACHABLE from gaze, and backward must not be.

The bug this pins down
----------------------
`gaze_control_node` declared `enable_vertical=False` and, with it off,
`_classify` returned only CENTER, TURN_LEFT or TURN_RIGHT. FORWARD and
BACKWARD were structurally unreachable - not hard to hit, not badly tuned,
but absent from the set of values the function could return.

The symptom the operator reported was exactly that: "the robot only turns left
and right, never forward". And the comment sitting on top of the flag claimed
forward was "handled by the jaw clench (EEG)" - except the clench maps to
TOGGLE_RUN, which is a gate on whether gaze may drive, and commands no velocity
of its own. So the robot could turn and stop, and never advance. The comment
described an intention that was never implemented, which is worse than no
comment: it tells the next reader the gap is already covered.

The fix keeps vertical STEERING off (the axis is too noisy to choose between
forward, backward and strafe) and adds one asymmetric rule: look UP for
forward, down does nothing.

Run:  python3 scripts/test_forward_reachable.py
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent /
                       'src/m1_control'))

from m1_control.gaze_control_node import (ZONE_BACKWARD, ZONE_CENTER,  # noqa: E402
                                          ZONE_FORWARD, ZONE_STRAFE_LEFT,
                                          ZONE_STRAFE_RIGHT, ZONE_TURN_LEFT,
                                          ZONE_TURN_RIGHT, GazeControlNode)
from m1_control.gaze_hud_node import GazeHUDNode                        # noqa: E402

failures = []


def check(name, cond, detail=''):
    if not cond:
        failures.append(name)
    print(f'  [{"PASS" if cond else "FAIL"}] {name}' + (f'   {detail}' if detail
                                                        else ''))


def make_ctl(enable_forward, forward_scale=1.6):
    """A GazeControlNode with just the attributes `_classify` reads.

    Built with __new__ so no rclpy context, no parameters and no timers are
    created: `_classify` is pure, and a test of a pure function should not need
    a robot.
    """
    o = object.__new__(GazeControlNode)
    o.enable_vertical = False
    o.enable_forward = enable_forward
    o.forward_scale = forward_scale
    o.enable_strafe = True
    return o


def make_hud(enable_forward, forward_scale=1.6):
    o = object.__new__(GazeHUDNode)
    o.enable_vertical = False
    o.enable_forward = enable_forward
    o.forward_scale = forward_scale
    return o


DZ = 0.125        # 0.25 ratio * 0.5, the real value at the default deadzone
GRID = [i / 40.0 for i in range(-20, 21)]      # dx, dy from -0.5 .. +0.5


def sweep(node, hud=False):
    """Every zone reachable anywhere in the unit square."""
    seen = set()
    for dx in GRID:
        for dy in GRID:
            if hud:
                # The HUD normalises by the deadzone radius itself.
                z, _ = node._classify(dx, dy, DZ, DZ)
            else:
                z = node._classify(dx, dy, DZ)
            seen.add(z)
    return seen


print('with the fix OFF, forward is unreachable (this was the bug):')
off = make_ctl(enable_forward=False)
zones_off = sweep(off)
check('FORWARD cannot be produced at all', ZONE_FORWARD not in zones_off,
      f'reachable: {sorted(zones_off)}')
check('BACKWARD cannot be produced either', ZONE_BACKWARD not in zones_off)
check('only turning and centre are possible',
      zones_off == {ZONE_CENTER, ZONE_TURN_LEFT, ZONE_TURN_RIGHT},
      f'{sorted(zones_off)}')

print('\nwith the fix ON, forward is reachable:')
on = make_ctl(enable_forward=True)
zones_on = sweep(on)
check('FORWARD is reachable', ZONE_FORWARD in zones_on)
check('looking straight up goes forward',
      on._classify(0.0, -0.40, DZ) == ZONE_FORWARD,
      on._classify(0.0, -0.40, DZ))
check('a small look up does NOT go forward',
      on._classify(0.0, -0.10, DZ) == ZONE_CENTER,
      on._classify(0.0, -0.10, DZ))
check('looking down does nothing at all',
      on._classify(0.0, 0.40, DZ) == ZONE_CENTER,
      'a glance at a keyboard must never reverse the robot')
check('BACKWARD stays unreachable', ZONE_BACKWARD not in zones_on)
check('up-and-hard-left still turns left, not forward',
      on._classify(-0.40, -0.40, DZ) == ZONE_TURN_LEFT,
      on._classify(-0.40, -0.40, DZ))
check('up-and-hard-right still turns right',
      on._classify(0.40, -0.40, DZ) == ZONE_TURN_RIGHT,
      on._classify(0.40, -0.40, DZ))
check('looking left still turns left', on._classify(-0.40, 0.0, DZ)
      == ZONE_TURN_LEFT)
check('looking right still turns right', on._classify(0.40, 0.0, DZ)
      == ZONE_TURN_RIGHT)
check('centre is still centre', on._classify(0.0, 0.0, DZ) == ZONE_CENTER)

print('\nforward is HARDER to trigger than a turn, on purpose:')
# A turn needs |dx| > DZ. Forward needs |dy| > DZ * scale, and the scale is
# > 1, so the vertical gesture is the more deliberate of the two.
turn_edge = DZ
fwd_edge = DZ * on.forward_scale
check('the forward threshold is wider than the turning threshold',
      fwd_edge > turn_edge, f'forward {fwd_edge:.3f} vs turn {turn_edge:.3f}')
check('just inside the turning threshold still turns',
      on._classify(-(turn_edge + 0.01), 0.0, DZ) == ZONE_TURN_LEFT)
check('just inside the forward threshold does not go forward',
      on._classify(0.0, -(fwd_edge - 0.01), DZ) == ZONE_CENTER)

print('\nthe overlay must agree with the controller:')
hud = make_hud(enable_forward=True)
mismatch = []
for dx in GRID:
    for dy in GRID:
        c = on._classify(dx, dy, DZ)
        h, _ = hud._classify(dx, dy, DZ, DZ)
        # The HUD is allowed to differ only where its oval deadzone (rx != ry)
        # genuinely maps differently; with rx == ry the two must agree.
        if c != h:
            mismatch.append((round(dx, 3), round(dy, 3), c, h))
check('the HUD reports the same zone as the controller', not mismatch,
      f'{len(mismatch)} disagreements, first: {mismatch[:3]}')

print('\nand the HUD never claims a backward drive the robot cannot make:')
hud_off = make_hud(enable_forward=False)
zones_hud_off = sweep(hud_off, hud=True)
check('HUD shows no FORWARD when the fix is off',
      ZONE_FORWARD not in zones_hud_off, f'{sorted(zones_hud_off)}')

print('\n' + '=' * 62)
if failures:
    print(f' {len(failures)} FAILED: ' + ', '.join(failures))
    print('=' * 62)
    sys.exit(1)
print(' FORWARD IS REACHABLE - look up to advance, look aside to steer')
print('=' * 62)
