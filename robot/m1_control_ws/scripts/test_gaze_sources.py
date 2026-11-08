"""Checks that every gaze source produces valid samples.

    python3 scripts/test_gaze_sources.py          (needs ROS, runs headless)

This exists because the calibration was reported as "all profiles broken, one
dot then 'not enough samples'". The dialog was fine - the SOURCE was reporting
no gaze.

Mouse mode was the culprit: `gaze_valid` stayed False until the operator
happened to click, so /gaze/raw reported invalid, the calibration collected
zero samples on its first dot, and the dialog said "the tracker lost your eyes"
- which sent the operator off to check the camera for a fault that was not
there.

So this checks the invariant that matters: every source that is RUNNING must
report valid, continuously, without the operator having to do anything first.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

import numpy as np                                               # noqa: E402
import rclpy                                                     # noqa: E402

import gaze_hud_node as g                                        # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


def main():
    rclpy.init()
    node = g.GazeHUDNode.__new__(g.GazeHUDNode)
    rclpy.node.Node.__init__(node, 'test_sources')
    g.GazeHUDNode.__init__(node)
    node.show_window = False
    node.frame = np.zeros((480, 640, 3), np.uint8)

    published = []
    node.raw_pub.publish = lambda m: published.append((m.x, m.y, m.z))

    # ---------------- mouse mode must be valid with no interaction --------
    node.gaze_source = 'mouse'
    node.gaze_valid = False
    node.mouse_click = None
    node._hud_tick()
    check('mouse mode reports valid without a click',
          published and published[-1][2] > 0.5,
          'z=%.1f' % (published[-1][2] if published else -1))
    check('mouse mode defaults to centre',
          published and abs(published[-1][0] - 0.5) < 1e-6,
          'u=%.3f' % (published[-1][0] if published else -1))

    # Hovering must move it, without any button being pressed.
    node._on_mouse(0, 480, 120, 0, None)          # EVENT_MOUSEMOVE, no flags
    node._hud_tick()
    check('hovering moves the mouse gaze',
          abs(published[-1][0] - 0.75) < 0.02 and abs(published[-1][1] - 0.25) < 0.02,
          'u=%.2f v=%.2f' % (published[-1][0], published[-1][1]))

    # ---------------- a lost face must report invalid ---------------------
    node.gaze_source = 'external'
    node.gaze_valid = False
    node._hud_tick()
    check('external mode with no publisher reports invalid',
          published[-1][2] < 0.5, 'z=%.1f' % published[-1][2])

    # ---------------- a continuous run must stay valid --------------------
    # The calibration needs a valid sample every tick, not occasionally.
    node.gaze_source = 'mouse'
    node.gaze_valid = False
    node.mouse_click = (320, 240)
    valid = 0
    for _ in range(30):
        node._hud_tick()
        if published[-1][2] > 0.5:
            valid += 1
    check('every tick of a run produces a valid sample',
          valid == 30, '%d/30' % valid)

    # ---------------- and it must be smooth, not frozen -------------------
    node._on_mouse(0, 100, 100, 0, None)
    node._hud_tick()
    a = published[-1][:2]
    node._on_mouse(0, 500, 400, 0, None)
    for _ in range(12):
        node._hud_tick()
    b = published[-1][:2]
    check('the reported point follows the source',
          abs(a[0] - b[0]) > 0.2, '%.2f -> %.2f' % (a[0], b[0]))

    node.destroy_node()
    rclpy.shutdown()

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
