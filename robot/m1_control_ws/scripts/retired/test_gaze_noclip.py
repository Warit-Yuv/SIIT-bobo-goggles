"""Checks that distinct head poses produce distinct gaze values.

    python3 scripts/test_gaze_noclip.py

This exists because of a bug that made the outer calibration dots unusable and
was invisible in every other test.

The eye term is stretched by `iris_gain` to fill the 0..1 range, and the head
term already spans it. The code used to stretch AFTER blending, so with
iris_gain = 3.0 anything more than 0.167 from centre reported EXACTLY 0.0 or
1.0:

    UP 15 deg    0.15 -> 0.00
    LEFT 20 deg  0.16 -> 0.00     identical
    DOWN 15 deg  0.85 -> 1.00
    RIGHT 20 deg 0.84 -> 1.00     identical

So top, top-left and top-right all reported the same number. No fit can recover
a position from data where distinct targets share one value - which is exactly
why the centre worked and the edges did not, however long the calibration ran.

The checks here are about DISTINCTNESS, not accuracy: two different head poses
must never collapse to the same value, and nothing may sit pinned at a rail.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / "src" / "m1_control" / "m1_control"))

import numpy as np, cv2, rclpy
import gaze_hud_node as g

f=640.0
K=np.array([[f,0,320],[0,f,240],[0,0,1]],float)
def Rx(d):
    a=math.radians(d); c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]],float)
def Ry(d):
    a=math.radians(d); c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]],float)
class P:
    def __init__(s,x,y): s.x,s.y=x,y
class LM:
    def __init__(s,p): s.p=p
    def __getitem__(s,i): return s.p[i]
def lms(rot):
    rv,_=cv2.Rodrigues(rot)
    im,_=cv2.projectPoints(g._PNP_MODEL, rv, np.array([0.,0.,500.]), K, None)
    im=im.reshape(-1,2)
    p=[P(0.5,0.5)]*500
    for k,i in enumerate(g._PNP_IDS): p[i]=P(im[k][0]/640.0, im[k][1]/480.0)
    return LM(p)

# Fake node carrying only what _blend_head reads.
n = g.GazeHUDNode.__new__(g.GazeHUDNode)
n.head_weight = 0.75
n.head_yaw = n.head_pitch = 0.0

fails=[]
def check(name, ok, d=""):
    print(("PASS  " if ok else "FAIL  ")+name+("   "+d if d else ""))
    if not ok: fails.append(name)

# Reproduce the REAL pipeline: eye gain applied first, then blend, then clip.
GAIN = 3.0
def pipeline(rot, eye_u=0.5, eye_v=0.5):
    eh = min(1.0, max(0.0, (eye_u-0.5)*GAIN + 0.5))
    ev = min(1.0, max(0.0, (eye_v-0.5)*GAIN + 0.5))
    u, v = g.GazeHUDNode._blend_head(n, eh, ev, lms(rot), 640, 480)
    return min(1.0,max(0.0,u)), min(1.0,max(0.0,v))

poses = [("neutral",np.eye(3)),("UP 15",Rx(15)),("DOWN 15",Rx(-15)),
         ("RIGHT 20",Ry(-20)),("LEFT 20",Ry(20)),
         ("UP-LEFT 15/20", Rx(15)@Ry(20)), ("UP-RIGHT 15/-20", Rx(15)@Ry(-20))]
print("%-16s %6s %6s" % ("pose","u","v"))
vals = {}
for name, rot in poses:
    u,v = pipeline(rot)
    vals[name]=(u,v)
    print("%-16s %6.3f %6.3f" % (name,u,v))

# THE BUG: distinct poses must give distinct values.
seen = {}
for name,(u,v) in vals.items():
    key = (round(u,3), round(v,3))
    seen.setdefault(key, []).append(name)
dupes = {k:v for k,v in seen.items() if len(v)>1}
check("no two distinct poses collapse to the same value", not dupes, str(dupes))

# And the corners must differ from the cardinals.
check("UP differs from UP-LEFT", abs(vals["UP 15"][0]-vals["UP-LEFT 15/20"][0])>0.02,
      "%.3f vs %.3f" % (vals["UP 15"][0], vals["UP-LEFT 15/20"][0]))
check("UP differs from UP-RIGHT", abs(vals["UP 15"][0]-vals["UP-RIGHT 15/-20"][0])>0.02,
      "%.3f vs %.3f" % (vals["UP 15"][0], vals["UP-RIGHT 15/-20"][0]))

# Nothing may sit pinned at a rail unless it genuinely is at the extreme.
rails = [k for k,(u,v) in vals.items() if u in (0.0,1.0) or v in (0.0,1.0)]
check("no pose is clipped flat to a rail", not rails, str(rails))

print()
print(("FAILED: "+", ".join(fails)) if fails else "all checks passed")
sys.exit(1 if fails else 0)
