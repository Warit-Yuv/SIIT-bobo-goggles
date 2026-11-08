# Gaze control

How the robot is steered with the operator's eyes.

---

## The three gaze sources

All three produce the same `/gaze/raw` and `/gaze/point`, so everything
downstream is identical. Pick whichever fits the moment.

| Source | Runs on | Needs | Quality |
|---|---|---|---|
| **Mediapipe** | operator PC | `~/gaze-libs` (already installed) | Real iris tracking |
| **Mouse** | operator PC | nothing | Aim by clicking — for testing |
| **External** | anywhere | your own tracker sending UDP :9100 | Best, if you have one |

### Start it from the console

**5 · Gaze** panel → pick a source → **Start gaze**.

A window opens showing the robot camera with your gaze point drawn on it. That
window is also where the deadzone circle is, so you can see exactly where the
robot thinks you are looking.

Or from a terminal:

```bash
~/m1_gaze.sh              # mediapipe
~/m1_gaze.sh mouse        # mouse
~/m1_gaze.sh external     # wait for /gaze/point from elsewhere
```

---

## Why mediapipe lives in `~/gaze-libs`

A normal `pip install mediapipe` pulls **numpy 2.x**, and rclpy is built against
numpy 1.x. Upgrading numpy breaks *every* ROS 2 Python node on the machine — not
just the gaze one.

So mediapipe is installed to a separate directory and put on `PYTHONPATH` for
that one process:

```
~/gaze-libs                 mediapipe 0.10.14 + numpy 1.26.4   (isolated)
/usr/lib/python3/dist-packages   ROS 2 Humble + rclpy          (untouched)
```

`m1_gaze.sh` sets this up. Verify at any time:

```bash
python3 -c "import numpy; print(numpy.__version__)"        # 1.26.4  (ROS)
PYTHONPATH=~/gaze-libs python3 -c "import mediapipe; print(mediapipe.__version__)"
```

Reinstall if needed:

```bash
pip3 install --target ~/gaze-libs "mediapipe==0.10.14" "numpy<1.27"
```

> Do **not** run `pip install mediapipe` without `--target`. It will upgrade
> numpy and break ROS.

---

## What the tracker actually measures

**Iris position and head rotation together.** Head rotation is not a refinement —
it is what makes vertical work at all.

| Signal | How | Why |
|---|---|---|
| Eye horizontal | iris position **along** the line joining the eye corners | clean, but a small range |
| Eye vertical | iris distance **off** that same line | tiny range — see below |
| Head yaw / pitch | `cv2.solvePnP` on six landmarks → 3D rotation | large, clean, dominates |

### Why the head is needed

The iris moves about 3 mm inside a 30 mm eye. Horizontally that is still a
usable fraction of the eye's width; **vertically it is a few percent**, and
the eyelid is right there adding its own motion. An iris-only vertical signal
is therefore mostly noise.

Measured on the hardware: the fitted vertical gain came back clamped at its
**0.25 minimum** every single run. That is the fit reporting "there is no
usable relationship between this signal and where you are looking". No
threshold, deadzone or calibration length could have fixed it.

People do not look at the top of a screen with their eyes alone — they tilt
their head up 10–20°. That motion is large, clean, and measurable from the
same landmark set. So the signal is:

```
u = (1 - k) * eye_u + k * head_yaw_term
v = (1 - k) * eye_v + k * head_pitch_term
```

with `head_weight` (`k`, default 0.75) on a slider in the Gaze panel. Measured
span at k=0.75: **0.70 vertically, 0.68 horizontally** from comfortable
movement. At k=0 the vertical axis is flat.

> A sign error in the PnP maths made this worse before it made it better:
> neutral was reported as **180° of yaw**, so the head term was a constant
> offset plus noise. `scripts/test_gaze_pose.py` now projects synthetic
> landmarks from known rotations, so a sign flip fails the test instead of
> reaching the operator.

### The signal is stretched before blending, never after

The eye term is multiplied by `iris_gain` to fill the range, and the head term
is already full-range. Getting that order wrong cost a whole debugging cycle:

```
UP 15°     blended 0.15 → ×3 clipped → 0.00
LEFT 20°   blended 0.16 → ×3 clipped → 0.00     ← identical
DOWN 15°   blended 0.85 → ×3 clipped → 1.00
RIGHT 20°  blended 0.84 → ×3 clipped → 1.00     ← identical
```

Everything more than 0.167 from centre reported **exactly** 0.0 or 1.0, so top,
top-left and top-right all reported the same number. No fit can recover a
position from data where distinct targets share one value — which is precisely
why the centre worked and the edges did not, however long the calibration ran.

The gain is now applied to the eye component *before* blending, so the clamp
only ever touches the term that genuinely needs stretching.
`scripts/test_gaze_noclip.py` asserts that distinct head poses produce distinct
values and that nothing sits pinned at a rail.

### Nothing in the acquisition path is clamped

There was a **second** clamp, and after the first was fixed it produced the same
symptom: `raw 0.55, 0.01`, `gain 1.00, 1.00`, and 900 px of error with the
**Head slider already at maximum**.

`solvePnP` reports an **absolute** head angle. That angle carries a constant
bias from things the operator cannot change: the camera sits above or below eye
level, and the generic face model is not their face. Clipping the head term to
±1 against a 16° pitch span put a 20° bias permanently on the rail — so the
head term was a *constant*, and the vertical axis reported one number forever.

A constant bias is the **normal** case, not an edge case. So neither the head
term nor the raw output is clamped any more: a bias simply shifts the value, and
the calibration's `offset` absorbs it. The clamp lives only on the **corrected**
value, where `gaze_calib.apply()` already does it.

Measured across ±30° of pitch bias, the usable vertical span is constant at
**0.703 of the range** — it does not shrink as the bias grows, which is the
property that matters. `scripts/test_gaze_bias.py` sweeps the bias and asserts
it.

**Head position (near / far) is not used.** solvePnP recovers translation too,
but only rotation is blended in, which is why leaning closer or further does
not shift the aim. The reference implementation's multi-posture calibration
exists to learn head-*position* effects; this does not attempt that.

---

## The pipeline

```
  eyes
   │
   ├── mediapipe (operator PC) ──┐
   ├── mouse click  (operator PC)┤
   └── UDP :9100    (anywhere)  ─┤
                                 ▼
                        /gaze/raw        uncorrected, z = valid flag
                                 │
                                 ▼
                        calibration      (raw - offset) * gain + 0.5
                                 │
                                 ▼
                        /gaze/point      corrected, 0..1
                                 │
                                 ▼
                     gaze_control_node   deadzone → PTZ, dwell → drive
                                 │
                    ┌────────────┴────────────┐
                    ▼                         ▼
             /servo/gaze              /cmd_vel/gaze
                    │                         │
                    └────────► arbiter ◄──────┘   (on the robot)
```

`/gaze/raw` exists so the console can calibrate against an uncorrected stream.
It comes from `gaze_udp_bridge` on the robot **and** from `gaze_hud_node` on the
PC, so calibration works with any source.

`/gaze/face` carries a 112×112 JPEG crop of the operator's face at 5 Hz, which
the console shows beside the gaze plot. It exists because "the dot is not
moving" and "the webcam is aimed at the ceiling" look identical on a bare dot.

---

## Calibrating

**5 · Gaze → Calibrate…**

Four profiles. All fit the same two numbers per axis; they differ only in how
many dots are used to fit them.

| Profile | Dots | Time | Use it when |
|---|---|---|---|
| **Fast** | 5 — centre, top, right, bottom, left | ~10 s | you shifted in your seat |
| **Long** | 9 — a 3×3 grid | ~30 s | first time, or after a long break |
| **Super** | 20 — weighted to the centre and the vertical axis | ~1 min | up/down still not working |
| **Ultra** | 47 — a scattered 5×8 grid plus 7 centre dots | ~2 min | nothing else worked |

More dots do not add parameters — the model has two unknowns per axis either
way. They add **confidence**, and they give the outlier rejection enough spare
dots to work with.

**Super** and **Ultra** are deliberately not even grids. Read their `v`
columns: the samples are concentrated on the centre column, because vertical is
where this tracker is weakest — the iris barely moves up and down inside the
eye, so most of that signal is head pitch.

**Ultra** also does not sweep. A raster walk visits each row left-to-right,
which means the eyes traverse the same horizontal path eight times and the
samples are correlated with each other rather than independent. It uses a
nearest-neighbour walk plus a 2-opt pass instead: scattered (38 row changes
across 47 dots) but never jumping more than 0.24 of the screen.

Timing **ramps down** across the run — 1.6 s settle at the start, 0.9 s at the
end — because the operator knows the rhythm by then, and the tail of a long run
is where attention fades anyway. A floor stops the ramp squeezing a sample
window below the point where it can collect enough samples.

### Is the calibration any good?

After the fit, the five cardinal directions are **re-visited and measured**.
That is the honest answer to the question, because the fit's own error only
tells you how well it reproduces the dots it was trained on.

```
Measured error (px):  CENTRE 8   TOP 41   BOTTOM 33   LEFT 12   RIGHT 19
Accurate in all five directions.
```

The result is shown in its **own label**, not appended to the instructions. It
used to be appended, which pushed the dim wrapped paragraph past what the bar
could show and clipped it away — the calibration appeared to end with no verdict
at all.

If a gain pins at a limit the run stops before validating and says why, since
re-visiting dots cannot help when the signal itself is wrong:

| Gain | Means | What to do |
|---|---|---|
| at `GAIN_MAX` (5.0) | the raw signal barely moved between dots — a **weak** signal | raise the **Head** slider |
| at `GAIN_MIN` (0.25) | the raw signal moved far more than the dots did — **noise** dominating | check lighting, retry |

### The dot shrinks

The calibration dot is **red** and contracts from 34 px to 9 px across the
settle phase, reaching its smallest exactly when sampling starts.

That is the instruction, not decoration. A static dot gives no cue about when
to start looking at it, and people arrive late — which puts the first samples
of every dot on the **previous** dot's position. That is a small, systematic,
one-directional error repeated twenty times, and it is exactly the kind of
thing that makes a calibration feel like it did nothing.

The dots spread across the **whole screen**, at 10% and 90% in each direction.

This matters more than it sounds. An eye tracker's raw signal is tiny: the iris
moves by a few thousandths of a frame between looking at the middle of a screen
and looking at its edge. The fit recovers `gain` by dividing that small change
into a large target change, so the smaller the calibration area, the more the
gain has to multiply — and it multiplies the tracker's noise along with the
signal.

> Calibrating inside the video panel asked for eye movement across a few square
> centimetres. Every sample landed within a hair of every other, the gain came
> back clamped or meaningless, and the outer zones were unreachable. That is
> what "it needs the whole screen to be effective" was describing.

The console fits:

```
corrected = (raw - offset) * gain + 0.5
```

by least squares — a line of *target against raw* per axis:

```
t = gain * raw + b     =>     gain = slope,  offset = (0.5 - b) / gain
```

That works for any number of dots, which is why one fitter serves both
profiles. The intercept is pinned through the centroid, so clamping the gain
cannot drag the neutral point off centre.

The **offset** is the raw value at rest — if your neutral gaze reads `u=0.62`,
the offset becomes `0.62` so that neutral maps to `0.5`. Getting this wrong is
what makes "calibration did nothing" happen.

The **long** profile also drops a dot that fits three times worse than the
median and refits. Measured on a simulated tracker, one bad dot moved the
fitted gain from 2.00 to 1.26 — a 37% error that feels like the head barely
moving. The **fast** profile drops nothing: with no spare dots there is no way
to know which one is wrong, which is exactly its trade-off.

Press **Reset** to return to neutral (offset 0.5, gain 1.0, i.e. passthrough).

### What calibration cannot fix

- **Head movement.** See above — iris position is relative to the eye, so
  moving your head while looking at the same place changes the reading. Sit
  however you intend to sit while driving.
- **Glasses with strong reflections.** Try tilting the screen or adding a light
  behind the camera.
- **A webcam that is too close.** Your whole face needs to be in frame.

---

## Zone mapping

`gaze_control_node` divides the frame into a deadzone and outer zones:

| Gaze position | Result |
|---|---|
| Inside the deadzone | Chassis still, gaze drives PTZ pan/tilt |
| Outside, held for `dwell_time_sec` | Chassis drives in that direction |

The **deadzone** exists so that looking around does not drive the robot. The
**dwell** exists because eyes saccade constantly — requiring the gaze to stay in
one zone for 0.4 s rejects the natural jitter that would otherwise cause
constant false commands.

### The deadzone is an oval, and it is tunable

```
deadzone_radius_ratio    0.30    horizontal, fraction of width
deadzone_radius_ratio_y  0.16    vertical,   fraction of height
```

Both are **radii**, so 0.30 covers 60% of the width either side of centre.

> These used to be multiplied by an extra 0.5 in the HUD, which silently halved
> both. A deadzone that should have covered half the width covered a quarter.
> That is why it felt far too small to be useful.

The eye travels roughly 2.5x further sideways than up and down, so an equal
radius makes FORWARD and BACKWARD unreachable while turning stays easy. Each
axis is normalised by its own radius and the direction test compares those
normalised offsets, so "up" competes on equal terms with "sideways".

Both are on **sliders in the Gaze panel**, live while driving, together with
`head_weight`. These are the numbers that have to be tuned by feel, so they
should not need a config edit and a relaunch. The overlay redraws immediately
so you can see what you just did.

---

## PTZ aiming is positional

The head does **not** integrate your gaze. A small look left is a small angle
to the left, and it stops there; look back to centre and the head returns.

```
pan_target  = pan_centre  + fx * (pan_span  / 2)      fx in -1..+1
```

where `fx` is the gaze offset as a fraction of the deadzone radius.

> An earlier version added a small step to the servo angle on every tick. At
> 30 Hz a full deflection moved the servo 45°/s and never stopped moving while
> your eyes were off centre — it drove into the travel limit and sat there.
> That is joystick behaviour, it makes fine aiming impossible, and it was
> reported as "the servo is so jerky".

A low-pass (`servo_smoothing`, default 0.25) then removes tracker jitter.

Separately, the **gaze signal itself** is filtered before any of this: a
median of the last five frames, then a low-pass. Tracker noise is not
symmetric — most frames are close to the truth and the occasional frame is
wildly wrong, because the iris ring is estimated and a blink or a specular
highlight moves it a long way for one frame. A plain low-pass cannot remove
that without making every real eye movement lag; a median rejects up to two
bad frames out of five no matter how wrong they are. Measured: a one-frame
spike to 0.99 from a steady 0.50 moves the aim by 0.000.

**Axis order matters.** The arbiter reads `/servo/gaze` as `s1 = msg.x`
(TILT, range 5–75) and `s2 = msg.y` (PAN, range 80–100). The HUD used to send
pan in `.x`, so each axis was clamped to the *other* axis's limits — pan to
5–75 and tilt to 80–100. Both were pinned against a wall, which looked like
jerking.

---

## Driving from one window

**5 · Gaze → Overlay** draws the control space onto the robot camera image in
the main window:

- the **oval deadzone**, in the same proportions the robot uses
- the **four sectors** and their labels — FWD, BACK, LEFT, RIGHT
- the **raw** point as a hollow amber ring
- the **aim** point filled, green inside the deadzone and red while driving
- a diagnostic line: `raw 0.62,0.55  aim 0.50,0.48`

It turns itself on when a gaze mode (2 or 4) is selected, from the button or
from the keyboard, because a gaze mode you cannot see is a mode you cannot
drive.

The picture is a map of the **control space**, not a claim about where on the
robot's view you are looking. The camera image is just the largest convenient
rectangle to draw it in, and it uses the same normalised coordinates the zones
are computed from — so if the dot is inside the oval the chassis really is
still, and if it is up in the top sector the robot really is about to drive.

The face thumbnail beside the gaze plot is fed by `/gaze/face` at 5 Hz. If it
says `no face`, the camera cannot see you and no amount of calibration will
help.

The overlay uses the console's own `deadzone_radius_ratio` and
`deadzone_radius_ratio_y`. **These must stay equal to the HUD's.** A picture
that disagrees with the zones the robot actually uses is the worst kind of
wrong, because it looks right.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `no source` in the Gaze panel | nothing publishing `/gaze/raw` | Start gaze, or start the robot stack (the bridge is in it) |
| `face lost` | mediapipe cannot see you | Move into frame; improve lighting |
| `no face` thumbnail | nothing on `/gaze/face` | Start gaze; it only publishes while tracking |
| Falls back to mouse | webcam busy or mediapipe missing | Close other camera apps (`cheese`, `guvcview`); check `~/gaze-libs` |
| `webcam did not open` | another process holds `/dev/video0` | `sudo fuser -v /dev/video0` — close it |
| Cursor drifts to one edge | uncalibrated | Run **Calibrate…** |
| Nothing moves but gaze is live | wrong control mode | Select mode **2 Gaze** (or **4** for fusion) |
| Robot jitters | deadzone too small | Raise `deadzone_ratio` |
| Turning is easy, forward is not | vertical deadzone too wide | Lower `deadzone_radius_ratio_y` |
| Up and down does nothing at all | `head_weight` too low — iris-only vertical is noise | Raise the **Head** slider |
| Up and down still poor after that | not enough vertical samples | Run the **Super** profile |
| Up and down still poor after that | the signal itself is weak | Run **Ultra**, and read the fit verdict |
| Have to strain your eyes to reach a zone | not enough head contribution | Raise the **Head** slider |
| Everything moves when you shift in your seat | too much head contribution | Lower the **Head** slider, then recalibrate |
| Head sweeps to a limit and stops | servo running in rate mode | Check `servo_pan_span` / `servo_smoothing` |
| Head moves the wrong way | axis inverted | `invert_pan` / `invert_tilt` |
| A calibration dot disappears | bar overlapping the dot area | Fixed — see `scripts/test_gaze_calib_layout.py` |

### Checking the camera is free

```bash
sudo fuser -v /dev/video0     # should print nothing
ls /dev/video0                # must exist
```

Only one process can open a V4L2 camera at a time. `cheese` and `guvcview` hold
it until closed, and so does a gaze HUD left running.

---

## Fusion mode (4 · Gaze + EEG)

Gaze steers continuously but cannot say "stop". EEG is discrete but reliable.
Mode 4 uses each for what it is good at:

| Input | Effect |
|---|---|
| Gaze | steers |
| EEG jaw clench | halts immediately, outranks gaze |
| EEG turn / strafe | brief override |
| EEG rest (zero) | ignored — gaze stays in charge |
| Keyboard | wins for 0.4 s in any mode |

The STOP-versus-REST distinction is the whole safety story: both produce a zero
velocity, so `/eeg/stop` is raised only for a genuine clench.
