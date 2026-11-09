#!/usr/bin/env python3
"""There must be exactly ONE node allowed to drive from gaze.

Why this test exists
--------------------
`gaze_hud_node` predates `gaze_control_node`: it used to BE the controller,
running its own deadzone, its own dwell timer and its own PTZ mapping, and
publishing `/cmd_vel/gaze`, `/servo/gaze` and `/gaze/zone` itself.

When the real control node was added on the robot, the HUD's copy was left
running. Both then published the same three topics. The arbiter - which
believes it is talking to one controller - saw an interleaving of two
independent policies with two independent dwell timers.

The symptom was not "the robot turns the wrong way". It was "the robot turns
or does not turn, at random". Measured with a steady synthetic look held for
three seconds, the zone topic alternated TURN_RIGHT / CENTER about every
250 ms, because one node had confirmed the zone and the other had not. The
chain check failed roughly one run in four and passed the rest, with no code
change in between.

That is the worst possible failure shape: it makes every other test suspect.
So the invariant is asserted here, in source, where it cannot rot quietly.
"""

import ast
import pathlib
import sys

SRC = pathlib.Path(__file__).resolve().parent.parent / 'src/m1_control/m1_control'

failures = []


def check(name, cond, detail=''):
    if not cond:
        failures.append(name)
    print(f'  [{"PASS" if cond else "FAIL"}] {name}' + (f'   {detail}' if detail
                                                        else ''))


def tree(module):
    return ast.parse((SRC / module).read_text(encoding='utf-8'))


def declared_default(module, param):
    """The literal default of `declare_parameter(param, X)`, or None."""
    for node in ast.walk(tree(module)):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == param):
            if len(node.args) > 1 and isinstance(node.args[1], ast.Constant):
                return node.args[1].value
    return None


def created_publishers(module):
    """{topic: whether the create_publisher call is conditional}."""
    out = {}
    for node in ast.walk(tree(module)):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'create_publisher'):
            continue
        if len(node.args) > 1 and isinstance(node.args[1], ast.Constant):
            out[node.args[1].value] = True
    return out


MOTION_TOPICS = ('/cmd_vel/gaze', '/servo/gaze', '/gaze/zone')

print('the HUD must be display-only by default:')
check('gaze_hud_node declares publish_motion', 'publish_motion' in
      (SRC / 'gaze_hud_node.py').read_text(encoding='utf-8'))
default = declared_default('gaze_hud_node.py', 'publish_motion')
check('and defaults it to False', default is False, f'default={default!r}')

hud = tree('gaze_hud_node.py')
cond_pubs = []
for node in ast.walk(hud):
    # Any `X if publish_motion else None` that produces a publisher.
    if isinstance(node, ast.IfExp):
        src = ast.dump(node)
        if 'publish_motion' in src and 'create_publisher' in src:
            cond_pubs.append(node)

hud_pub_topics = created_publishers('gaze_hud_node.py')
print('\nthe HUD still publishes the thing it is FOR:')
check('the HUD publishes /gaze/point', '/gaze/point' in hud_pub_topics)
check('the HUD publishes /gaze/raw for calibration',
      '/gaze/raw' in hud_pub_topics)

print('\nthe HUD must not create an unconditional motion publisher:')
for topic in MOTION_TOPICS:
    # Find the create_publisher call for this topic and require that it sits
    # inside a conditional on publish_motion.
    guarded = False
    for node in ast.walk(hud):
        if (isinstance(node, ast.IfExp) and 'publish_motion' in ast.dump(node)
                and topic in ast.dump(node)):
            guarded = True
    check(f'{topic} is created only when publish_motion is on', guarded,
          '' if guarded else 'an unconditional publisher advertises an '
                             'authority this node does not have')

print('\nand the real controller must still own them:')
ctl = created_publishers('gaze_control_node.py')
for topic in ('/cmd_vel/gaze', '/servo/gaze', '/gaze/zone'):
    check(f'gaze_control_node publishes {topic}', topic in ctl)

print('\nthe mute must exist, or the chain check cannot isolate gaze:')
check('gaze_hud_node subscribes /gaze/hud_enable',
      '/gaze/hud_enable' in (SRC / 'gaze_hud_node.py').read_text(encoding='utf-8'))
check('and acknowledges on /gaze/hud_enabled_state',
      '/gaze/hud_enabled_state' in
      (SRC / 'gaze_hud_node.py').read_text(encoding='utf-8'))

print('\n' + '=' * 60)
if failures:
    print(f' {len(failures)} FAILED: ' + ', '.join(failures))
    print('=' * 60)
    sys.exit(1)
print(' ONE AUTHORITY - the HUD tracks, the robot drives')
print('=' * 60)
