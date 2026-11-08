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

---

## Calibrating

**5 · Gaze → Calibrate…**

You are shown five dots — centre, left, right, up, down. Look at each and hold
still. The console fits:

```
corrected = (raw - offset) * gain + 0.5
```

Five points recover **gain** as well as offset, so a small comfortable eye
movement still covers the full steering range. Without gain, an operator who
barely moves their eyes could never reach the edges of the screen.

The **offset** is the raw value at rest — if your neutral gaze reads `u=0.62`,
the offset becomes `0.62` so that neutral maps to `0.5`. Getting this wrong is
what makes "calibration did nothing" happen.

Press **Reset** to return to neutral (offset 0.5, gain 1.0, i.e. passthrough).

### What calibration cannot fix

- **Head movement.** Mediapipe tracks iris position relative to the eye, so
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
| Inside the deadzone circle | Chassis still, gaze drives PTZ pan/tilt |
| Outside, held for `dwell_time_sec` | Chassis drives in that direction |

The **deadzone** exists so that looking around does not drive the robot. The
**dwell** exists because eyes saccade constantly — requiring the gaze to stay in
one zone for 0.4 s rejects the natural jitter that would otherwise cause
constant false commands.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `no source` in the Gaze panel | nothing publishing `/gaze/raw` | Start gaze, or start the robot stack (the bridge is in it) |
| `face lost` | mediapipe cannot see you | Move into frame; improve lighting |
| Falls back to mouse | webcam busy or mediapipe missing | Close other camera apps (`cheese`, `guvcview`); check `~/gaze-libs` |
| `webcam did not open` | another process holds `/dev/video0` | `sudo fuser -v /dev/video0` — close it |
| Cursor drifts to one edge | uncalibrated | Run **Calibrate…** |
| Nothing moves but gaze is live | wrong control mode | Select mode **2 Gaze** (or **4** for fusion) |
| Robot jitters | deadzone too small | Raise `deadzone_ratio` |

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
