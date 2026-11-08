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

Iris position only — **not** head pose.

Mediapipe FaceMesh gives 478 landmarks including an iris ring per eye. From
those, two numbers are derived per eye:

| Axis | How |
|---|---|
| Horizontal | where the iris sits **along** the line joining the eye corners, as a fraction |
| Vertical | how far the iris sits **off** that same line, in units of eye width |

Both eyes are averaged. Because both axes are measured against the *same*
stable reference — the eye-corner line — they have the same noise floor.

> An earlier version measured vertical against the distance between the upper
> and lower eyelid. That denominator is a few pixels and changes on every
> blink, so the vertical signal was mostly eyelid motion. The symptom was
> exactly what an operator would report as "up and down is hard to tell apart":
> the vertical gain in calibration kept coming back clamped at its minimum.

**This means the tracker does not compensate for head movement.** Look at the
same spot while moving your head and the reading changes. Full head-pose
compensation (the `geo_iris` model in [GazeModel](https://github.com/lANXEZ/GazeModel))
adds a solved 3D head rotation to the eye-in-head angle. That is a real
improvement and it is **not implemented here** — sit still and it does not
matter; move around and it does.

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

Two profiles. Both fit the same two numbers per axis; they differ only in how
many dots are used to fit them.

| Profile | Dots | Time | Use it when |
|---|---|---|---|
| **Fast** | 5 — centre, top, right, bottom, left | ~10 s | you shifted in your seat |
| **Long** | 9 — a 3×3 grid | ~30 s | first time, or after a long break |

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

Both are on **sliders in the Gaze panel**, live while driving. This is the one
number that has to be tuned by feel, so it should not need a config edit and a
relaunch. The overlay redraws immediately so you can see what you just did.

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
| Head sweeps to a limit and stops | servo running in rate mode | Check `servo_pan_span` / `servo_smoothing` |
| Head moves the wrong way | axis inverted | `invert_pan` / `invert_tilt` |

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
