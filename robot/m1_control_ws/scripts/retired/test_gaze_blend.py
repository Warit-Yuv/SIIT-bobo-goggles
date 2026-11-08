"""Checks that head rotation and eye position combine into a usable range.

    python3 scripts/test_gaze_blend.py

The iris barely moves vertically inside the eye, so an iris-only vertical
signal is mostly noise - measured on the hardware, the fitted vertical gain
came back clamped at its 0.25 minimum. Adding head rotation is what makes
up-and-down work, and this checks the blend actually produces a full range
rather than a few hundredths.
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

fails=[]
def check(n,ok,d=""):
    print(("PASS  " if ok else "FAIL  ")+n+("   "+d if d else ""))
    if not ok: fails.append(n)

# Fake node carrying only what _blend_head touches.
n = g.GazeHUDNode.__new__(g.GazeHUDNode)
n.head_weight = 0.75
n.head_yaw = n.head_pitch = 0.0

print("%-16s %6s %6s" % ("head pose", "u", "v"))
res={}
for name, rot in [("neutral",np.eye(3)),("UP 15",Rx(15)),("DOWN 15",Rx(-15)),
                  ("RIGHT 20",Ry(-20)),("LEFT 20",Ry(20))]:
    u,v = g.GazeHUDNode._blend_head(n, 0.5, 0.5, lms(rot), 640, 480)
    res[name]=(u,v)
    print("%-16s %6.2f %6.2f" % (name,u,v))

check("neutral maps to centre", abs(res["neutral"][0]-0.5)<0.02 and abs(res["neutral"][1]-0.5)<0.02)
check("UP gives v < 0.5", res["UP 15"][1] < 0.45, "%.2f" % res["UP 15"][1])
check("DOWN gives v > 0.5", res["DOWN 15"][1] > 0.55, "%.2f" % res["DOWN 15"][1])
check("RIGHT gives u > 0.5", res["RIGHT 20"][0] > 0.55, "%.2f" % res["RIGHT 20"][0])
check("LEFT gives u < 0.5", res["LEFT 20"][0] < 0.45, "%.2f" % res["LEFT 20"][0])

# The point of the change: vertical must move by a USABLE amount.
span_v = res["DOWN 15"][1] - res["UP 15"][1]
span_u = res["RIGHT 20"][0] - res["LEFT 20"][0]
check("vertical span is usable (>0.6 of range)", span_v > 0.6, "%.2f" % span_v)
check("horizontal span is usable (>0.6 of range)", span_u > 0.6, "%.2f" % span_u)

# With head weight 0 the vertical axis must collapse - proving the head term
# is what carries it, which is the whole reason for this change.
n.head_weight = 0.0
u0,v0 = g.GazeHUDNode._blend_head(n, 0.5, 0.5, lms(Rx(15)), 640, 480)
check("at head_weight 0 the head is ignored", abs(v0-0.5)<1e-9, "v %.3f" % v0)
n.head_weight = 1.0
u1,v1 = g.GazeHUDNode._blend_head(n, 0.5, 0.5, lms(Rx(15)), 640, 480)
check("at head_weight 1 the eyes are ignored", abs(v1-0.5)<1e-9 or v1<0.5, "v %.3f" % v1)

print()
print(("FAILED: "+", ".join(fails)) if fails else "all checks passed")
sys.exit(1 if fails else 0)
