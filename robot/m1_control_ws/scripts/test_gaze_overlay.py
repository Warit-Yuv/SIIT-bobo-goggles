"""Checks the console's gaze overlay geometry and rendering.

Runs headless on the operator PC:  QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_overlay.py

The overlay is the only feedback the operator has about where the robot
thinks they are looking, so its geometry has to match the zones the robot
actually uses. These checks measure pixels, not impressions.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                        / "src" / "m1_control" / "m1_control"))
from PyQt5 import QtCore, QtGui, QtWidgets
from m1_console import make_gaze_overlay

app = QtWidgets.QApplication(sys.argv)
draw = make_gaze_overlay(QtCore, QtGui)
W, H = 640, 480
DZX, DZY = 0.30, 0.16
CAL = (0.62, 0.55, 2.0, 0.8)
fails = []
def check(n, ok, d=""):
    print(("PASS  " if ok else "FAIL  ") + n + ("   " + d if d else ""))
    if not ok: fails.append(n)

def render(raw, zone, debug=False):
    pix = QtGui.QPixmap(W, H); pix.fill(QtGui.QColor("#101010"))
    return draw(pix, zone, raw, CAL, DZX, DZY, debug).toImage()

def extent(img, y=None, x=None):
    if y is not None:
        return [p for p in range(W) if img.pixelColor(p, y).green() > 140 and img.pixelColor(p, y).red() < 130]
    return [p for p in range(H) if img.pixelColor(x, p).green() > 140 and img.pixelColor(x, p).red() < 130]

img = render((0.5, 0.5, True, 0.0), "CENTER")
row = extent(img, y=H//2); col = extent(img, x=W//2)
exp_w, exp_h = 2*W*DZX, 2*H*DZY
print("oval width  %d  expected %.0f" % (max(row)-min(row) if row else -1, exp_w))
print("oval height %d  expected %.0f" % (max(col)-min(col) if col else -1, exp_h))
check("deadzone oval is the size the HUD uses",
      row and col and abs((max(row)-min(row)) - exp_w) <= 4 and abs((max(col)-min(col)) - exp_h) <= 4)

# The old code halved these. Prove it is now 2x what it would have been.
check("no hidden halving (radius = w*ratio)",
      row and abs((max(row)-min(row))/2.0 - W*DZX) <= 2,
      "radius %.0f px" % ((max(row)-min(row))/2.0 if row else -1))

# dot position
raw = (0.70, 0.40, True, 0.0)
img = render(raw, "FORWARD")
cu = (raw[0]-CAL[0])*CAL[2]+0.5
cv = (raw[1]-CAL[1])*CAL[3]+0.5
xs = ys = n = 0
for y in range(60, H):
    for x in range(W):
        c = img.pixelColor(x, y)
        if c.red() > 220 and c.green() < 90 and c.blue() < 90:
            xs += x; ys += y; n += 1
check("aim dot at the calibrated position",
      n and abs(xs/n - cu*W) < 4 and abs(ys/n - cv*H) < 4,
      "(%0.1f,%0.1f) vs (%0.1f,%0.1f)" % (xs/n if n else -1, ys/n if n else -1, cu*W, cv*H))

# centre must be inside the oval, and a big look outside
check("centre gaze sits inside the oval", abs(0.5-0.5)/DZX < 1)
check("a 25%% look up is outside the vertical radius", (0.5-0.25)/DZY > 1.0,
      "%.2f of radius" % ((0.5-0.25)/DZY))

for z in ("CENTER","FORWARD","BACKWARD","TURN_LEFT","TURN_RIGHT"):
    render((0.5,0.5,True,0.0), z, debug=True)
print("all zone labels render: PASS")
render(None, ""); render((0.5,0.5,False,0.0), ""); render((0.5,0.5,True,5.0), "")
print("no-source / face-lost / stale render: PASS")
print()
print(("FAILED: " + ", ".join(fails)) if fails else "all checks passed")
sys.exit(1 if fails else 0)
