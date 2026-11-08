"""Checks that gaze filtering rejects tracker spikes without adding lag.

Runs on the Pi or the operator PC:  python3 scripts/test_gaze_smoothing.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                        / "src" / "m1_control" / "m1_control"))
import rclpy
from gaze_hud_node import GazeHUDNode
fails = []
def check(n, ok, d=""):
    print(("PASS  " if ok else "FAIL  ") + n + ("   " + d if d else ""))
    if not ok: fails.append(n)

rclpy.init()
n = GazeHUDNode.__new__(GazeHUDNode)
rclpy.node.Node.__init__(n, "t")
GazeHUDNode.__init__(n)
n.show_window = False; n.gaze_source = "external"

# 1. steady gaze stays steady
n._reset_smooth(0.5, 0.5)
for _ in range(30):
    su, sv = n._smooth(0.5, 0.5)
check("steady gaze stays put", abs(su-0.5) < 1e-9, "%.4f" % su)

# 2. single-frame spike is rejected no matter how large
n._reset_smooth(0.5, 0.5)
for _ in range(10):
    n._smooth(0.5, 0.5)
peak = 0.0
n._smooth(0.99, 0.99)
peak = max(peak, n.smooth_u)
n._smooth(0.5, 0.5)
check("a huge one-frame spike barely moves the aim", peak < 0.62,
      "peak %.3f from a 0.99 spike" % peak)

# 3. a deliberate look is followed
n._reset_smooth(0.5, 0.5)
for _ in range(40):
    su, sv = n._smooth(0.80, 0.30)
check("deliberate look is followed", abs(su-0.80) < 0.01 and abs(sv-0.30) < 0.01,
      "settled %.3f, %.3f" % (su, sv))
n._reset_smooth(0.5, 0.5)
for _ in range(3):
    su, _ = n._smooth(0.80, 0.5)
check("look is mostly there within 3 frames", su > 0.60, "%.3f after 3 frames" % su)

# 4. sustained jitter around a boundary does not flicker the zone
n._reset_smooth(0.5, 0.5)
zones = set()
import random
random.seed(7)
for i in range(120):
    # +/- 0.04 jitter around a point just inside the vertical radius
    j = random.uniform(-0.04, 0.04)
    su, sv = n._smooth(0.5, 0.5 - 0.10 + j)
    z, _ = n._classify((su-0.5)*640, (sv-0.5)*480, 640*n.deadzone_ratio, 480*n.deadzone_ratio_y)
    zones.add(z)
check("jitter near the edge does not flicker zones", len(zones) == 1, str(zones))

n.destroy_node(); rclpy.shutdown()
print()
print(("FAILED: " + ", ".join(fails)) if fails else "all checks passed")
sys.exit(1 if fails else 0)
