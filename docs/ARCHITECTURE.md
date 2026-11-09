# Architecture

A concise account of how this system is put together and why. For setup
instructions see [`../robot/SETUP.md`](../robot/SETUP.md) and
[`../operator-pc/SETUP.md`](../operator-pc/SETUP.md). For the long narrative
with the full defect history, see [`../robot/docs/report.md`](../robot/docs/report.md).

**The goal:** give a person who cannot use their hands — someone with paresis,
lying in bed — real control of a mobile robot. Eyes for steering, a headset for
the commands eyes cannot express, a keyboard for when neither works.

---

## 1. The shape of it

Two machines, one ROS 2 graph.

```
   OPERATOR PC (10.0.0.102)                  ROBOT (10.0.0.54, Raspberry Pi 5)
   ┌──────────────────────────┐              ┌──────────────────────────────┐
   │ m1_console    (PyQt5)    │              │ control_arbiter_node         │
   │   ├ 6 panels, SLAM map   │   Wi-Fi      │   ├ mode mux                 │
   │   └ camera picker        │   DDS        │   ├ velocity clamp           │
   │ gaze_hud_node            │◄────────────►│   └ 350 ms watchdog          │
   │   ├ webcam -> 10 features│  unicast     │                              │
   │   └ ridge model -> point │              │ gaze_control_node            │
   │ rviz2 + slam_toolbox     │              │   └ deadzone/dwell policy    │
   └──────────────────────────┘              │ eeg_udp_bridge  (UDP :9000)  │
                                             │ sound_player_node            │
   ┌──────────────────────────┐              │ image_throttle_node          │
   │ WINDOWS LAPTOP           │  UDP :9000   │ audio_bridge                 │
   │ Muse 2 -> BlueMuse ->    │─────────────►│                              │
   │ gesture classifier       │              │ motors, PTZ, camera, LiDAR   │
   └──────────────────────────┘              └──────────────────────────────┘
```

**Why the split is where it is.** Everything safety-critical runs on the Pi.
A watchdog that runs on the PC cannot stop a robot whose Wi-Fi link just died —
the machine that would have to act is the one that lost contact.

**Why vendor code is untouched.** Everything custom lives in a Layer 4 overlay
(`m1_control_ws`), sourced after the Yahboom workspaces. The robot can be
restored to stock by deleting one directory.

---

## 2. Four ideas the whole design rests on

### 2.1 One publisher per actuator

Only `control_arbiter_node` publishes `/cmd_vel` and `/Servo`. Every input
publishes to its own private topic, and the arbiter forwards exactly one.

```
keyboard ──► /cmd_vel/keyboard ──┐
gaze     ──► /cmd_vel/gaze     ──┼──► [ARBITER] ──► /cmd_vel ──► motors
                                 ──┘        ▲
                                  /control_mode  (which one wins?)
```

Two controllers can never fight, because they are not on the same wire. The
alternative — everyone publishing `/cmd_vel` and hoping — is the classic ROS
robot that twitches.

The rule has a corollary that is easy to miss: **one publisher per *input*
topic, too.** `/cmd_vel/gaze` is a wire just like `/cmd_vel` is. Putting the
arbiter in the middle only helps if each input really has one writer — two
writers on one input topic re-create the twitch one layer down, where the
arbiter cannot see it. That is exactly what happened with the gaze HUD; see
§2.2.

### 2.2 One motion authority, and the HUD is not it

`gaze_hud_node` on the operator PC predates `gaze_control_node` on the robot: it
used to *be* the controller, with its own deadzone, its own dwell timer and its
own PTZ mapping. When the real control node was added, the HUD's copy was left
running, so both published `/cmd_vel/gaze`, `/servo/gaze` and `/gaze/zone`.

The arbiter believes it is talking to one controller. It was actually seeing an
interleaving of two independent policies with two independent dwell timers, and
the symptom was not "the robot turns the wrong way" but "the robot turns, or
does not turn, at random". A steady synthetic look held for three seconds made
the zone topic alternate `TURN_RIGHT` / `CENTER` about every 250 ms. The chain
check failed roughly one run in four and passed the rest, with no code change
in between — the worst failure shape there is, because it makes every other
test suspect.

The HUD now runs `publish_motion:=false` by default: it tracks the face,
publishes `/gaze/point`, and draws the overlay. It does not create the motion
publishers at all, so `ros2 topic info` shows one controller rather than two.
`scripts/test_one_authority.py` asserts this in source, because the invariant
cannot be observed from the topic graph without the robot powered.

### 2.3 The EEG is a gate, not a joystick

This is the single most important design decision, and it came from watching
the first version fail.

The headset classifier emits **discrete events**. Mapping those onto velocities
means every missed packet is a wrong movement, and every false positive is a
lurch. Protocol v1 replaced it with three toggles plus a state block repeated in
every packet:

| Gesture | Toggle | Effect |
|---|---|---|
| Jaw clench | `run` | go / stop |
| Double blink | `mode` | steer the **wheels** or steer the **camera** |
| Eyebrow raise | `music` | play a sound — no motion at all |

`run` is a **gate**, not a command. It says *"gaze may now drive"*. So a
misclassified gesture can start or stop the robot but **cannot make it move at
a speed nobody asked for**.

Gestures are assigned by how easily they can be **performed**, not by raw
accuracy. The clench is the least tiring, so it owns the toggle used several
times a minute. The eyebrow raise is the least reliable, so it gets the only job
where being wrong costs nothing.

### 2.4 Gaze is a regression, not a ratio

The original tracker measured two numbers per eye and fitted a straight line
through them:

```
corrected = (raw - offset) * gain + 0.5
```

It was rewritten four times, given four calibration profiles up to 47 dots, and
still could not steer. **The maths was never the problem. The input was.**

Gaze depends on where the eyes point, where the head points, how far away it is,
and where it sits laterally. Two parameters per axis draw a line through one of
those and call it the answer.

```
gaze_tracker.py   camera frame ──► 10 features
gaze_model.py     10 features  ──► screen point
```

| # | Feature | What it carries |
|---|---|---|
| 0-1 | iris offset from eye centre ÷ eye width | where the eye points |
| 2 | eyelid gap | blinks, and a proxy for pitch |
| 3-4 | MediaPipe `eyeLook*` blendshapes | the model's own eye direction |
| 5-6 | head yaw / pitch | where the head points |
| 7-9 | head x, y, z | lateral position and **distance** |

The model is a ridge regression with quadratic, interaction and **cubic** terms.

---

## 3. The interesting parts

### 3.1 The iris cannot leave the eye, and that changes everything

Past roughly 14° of look angle the measured eye signal **stops growing** — the
iris is against the corner of the socket. The relationship between gaze angle
and measured eye position is an S-curve, and an S-curve has an inflection in the
middle.

A parabola bends the same way everywhere and structurally cannot represent an
inflection. Adding a cubic term on the four eye-direction columns:

| Model | Held-out error (1920 px wide) |
|---|---|
| quadratic only | 75 px |
| **+ cubic on the eye columns** | **24 px** |

Four extra columns, no extra dots, no change in operator behaviour.

### 3.2 Calibrating at one distance teaches nothing about another

Twenty dots in one posture teach the model what the eyes look like from **one
place**. Move the chair back and every feature shifts at once.

The **Posture** profile calibrates at four postures, twenty jittered dots each:

| Posture | What it teaches |
|---|---|
| Sit normally | the baseline |
| Lean back | distance, smaller face in frame |
| Lean in | distance, the other way |
| Shift sideways | off-centre faces, and yaw |

Each posture gets its own **jittered 5×4 grid** — one random point per cell,
middle-out, a different seed per posture, so no two postures sample the same
places and the sequence cannot be anticipated.

Measured against a single-posture run **using the same model**:

| Calibration | Dots | Held-out error |
|---|---|---|
| single posture | 20 | 192 px |
| **four postures** | **80** | **24 px mean, 63 px p95** |

**8× better on points neither had seen.** That is the difference between a
tracker the operator abandons and one they use.

### 3.3 Reporting is leave-one-dot-out, because the honest number is the useful one

`fit()` returns the error on a dot the model never saw. That is the number
shown. Calling `validate()` on the training data instead would report how well
the model reproduces what it was shown — always flattering, never useful, and
precisely how a broken calibration can run to completion, report success, and
still steer wrong.

### 3.4 Three independent gates, and all three must agree

Between an eye movement and a wheel turning:

```
mode is 2/3/4?  ──►  run is ON?  ──►  link is alive?  ──►  drive
```

Both the mode and the run flag default to *off*, so a fresh session looks
completely dead — and the gaze HUD still draws a zone, which reads as "gaze
works, the robot is broken". The console now says so in words.

The link gate has a deliberate asymmetry: it only engages **once an EEG has
actually been seen**. Blocking on a link that was never there is a trap, and a
dead laptop must never leave the robot driving. So:

* never connected → gaze drives ungated (with a warning)
* connected then silent → the chassis stops

### 3.5 Positional, not integrating

Looking slightly left moves the head slightly left **and it stays there**.
Looking back to centre returns it. It does not keep drifting.

An earlier version added a step on every tick — that is rate control, and
holding your eyes off-centre drove the head to its travel limit and pinned it
there. The complaint was "it turns left too much", and the cause was that the
controller was integrating an offset instead of tracking a position.

### 3.6 Every fallback is the same code path as the real thing

The keyboard fallback for the EEG gestures does **not** publish `/eeg/run`,
`/eeg/mode` or `/eeg/music` itself. It publishes the gesture name on
`/eeg/simulate` and the **bridge** applies it, running exactly the code it would
have run for the headset.

Two independent paths would let one rot silently, and then the fallback would be
exercising something the real input never touches.

The same reasoning applies to the calibration dialog: it fits the model and
writes `~/gaze_calibration.json`; the HUD watches that file. Neither side keeps
a second copy of the truth.

---

## 4. What is where

| Module | Role |
|---|---|
| `gaze_tracker.py` | Frame → 10 features. FaceLandmarker, blendshapes, head pose |
| `gaze_model.py` | 10 features → screen point. Ridge + quadratic + cubic |
| `gaze_hud_node.py` | Webcam tracker, HUD overlay. **Display-only** (PC) |
| `gaze_control_node.py` | Deadzone/dwell policy, EEG gating |
| `eeg_udp_bridge.py` | UDP :9000 → `/eeg/run`, `/eeg/mode`, `/eeg/gesture` |
| `sound_player_node.py` | Eyebrow raise → a random file from `/home/pi/sound` |
| `control_arbiter_node.py` | Mode mux, clamping, watchdog. The safety core |
| `m1_console.py` | Operator GUI: 6 panels, SLAM map, calibration, teleop |

---

## 5. Testing philosophy

**A test that passes is not the same as a system that works.** Four tests
guarding the two-parameter gaze model all passed while the operator could not
drive the robot. They were retired with a README explaining what they protected
and where that lives now (`scripts/retired/`).

The rules that came out of that:

* **Test the widget, not the dict.** A profile existed, was tested, and was
  unreachable from the GUI because the dropdown was filled from a second,
  hand-maintained list.
* **Skip when a precondition you cannot arrange is missing.** Two copies of a
  script driving `/control_mode` produce mutual interference and mutual false
  failures. A suite that goes red for reasons outside the code is a suite people
  stop reading.
* **Verify the test can fail.** Every regression test here has been checked
  against the broken version.
* **A silent fallback looks identical to a broken component.** The gaze HUD
  falling back to mouse mode logged one line and then behaved exactly like a
  tracker that was working but inaccurate — which cost several rounds of
  debugging.
* **A test that is green sometimes is worse than a red one.** The chain check
  needed to inject its own gaze points, and it tried to silence the real HUD
  first. The attempt ran `pgrep` locally — inside a container, where it can
  never see the operator PC — found nothing, and quietly did nothing. So the
  control node saw two interleaved publishers and the result depended on which
  one won the race. It passed, then failed, then passed, with no code change in
  between. The fix was not a longer timeout: it was giving the HUD a *mute* that
  works across machines, and then **proving the mute took effect** by watching
  for silence before trusting any measurement.

---

## 6. Known limits

| Limit | Why | Consequence |
|---|---|---|
| Vertical gaze is disabled | The iris barely moves up and down; the signal is mostly head pitch and noise | Gaze steers left/right only. Forward/back is a deliberate gesture, not a look |
| One camera, one operator | The tracker is a single-face model | No multi-user support |
| Calibration is per-person and per-seating | The model is fitted to a face at a distance | Re-run Posture after moving the bed or the chair |
| The headset classifier confuses two gestures | Measured on this hardware; it reports `UNCLEAR_MUSCLE` | Every gesture has a key, and the least reliable gesture does nothing but play a sound |
| Wi-Fi multicast drops video | Measured: 87% loss | DDS is configured for **unicast** peers on both machines |

---

## 7. If something does not work

Run, in order:

```bash
# 1. Is the chain intact?  (robot off the floor)
python3 scripts/check_chain.py

# 2. Are the safety invariants intact?
python3 scripts/validate_safety.py

# 3. Is the headset reaching the robot?
python3 scripts/eeg_ping.py --host 10.0.0.54

# 4. Is the tracker seeing a face?
python3 scripts/gaze_diagnose.py --quick
```

`check_chain.py` walks `mode → run gate → gaze → arbiter → /cmd_vel` in order
and stops at the first thing that is wrong, rather than leaving you to guess
which of five layers is at fault. It mutes the gaze HUD over `/gaze/hud_enable`
first, because two publishers on one topic make the result a race — and if it
cannot prove the mute worked it says so and SKIPs the gaze checks instead of
reporting a failure it cannot justify.

`validate_safety.py` leaves the robot in **E-STOP**, which is the safe place to
leave it but means you have to select a mode before driving again. It SKIPs the
two tests that need to own the EEG link when a real headset is connected, and
says why.

Each of these names the layer that failed instead of leaving you to guess.
