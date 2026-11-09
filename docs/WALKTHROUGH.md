# Walkthrough — Understanding the Whole System

A guided tour from "what is this thing" to "I can run and debug it myself".
Read this once end to end; it will make every other document obvious.

---

## Part 1 — What we actually built

You have a small wheeled robot with a camera on a moving head and a spinning
laser on top. The goal is to drive it **three different ways**, and have it map
the room while doing so.

```
                        ┌─────────────────────────┐
   keyboard ───────────►│                         │
   eyes (webcam) ──────►│   the robot decides     │──────► 4 wheels
   brainwaves (EEG) ───►│   ONE thing to do       │──────► pan/tilt head
                        └─────────────────────────┘
                                     ▲
                                     │
                        camera + laser + battery
```

That's the whole project. Everything else is plumbing.

### The three input modes

| Mode | You control it with | Sensor | Hard part |
|---|---|---|---|
| **1 — Keyboard** | `WASD` + `QE` for wheels, `IJKL` for head | keyboard | none, it just works |
| **2 — Gaze** | where your eyes look on the screen | laptop webcam | eyes twitch constantly (saccades) |
| **3 — EEG** | jaw clench, double blink, look left/right | Muse 2 headband | brain signals are noisy |

### Why this is harder than it sounds

Three controllers all wanting to move the same motors. If keyboard says
"forward" and gaze says "stop", and both write to the motor topic, the motors
get a rapid alternation of forward/stop — violent vibration that destroys
gearboxes.

**That single problem shaped the entire architecture.** Hold onto it.

---

## Part 2 — The mental model: how ROS 2 works

If you understand four ideas, you understand ROS 2.

### Idea 1: Nodes

A **node** is one small program that does one job. Not one big program that
does everything — many small ones.

Ours:

```
Mcnamu_driver_M1        talks to the motor board over serial
ydlidar_ros2_driver     talks to the laser
usb_cam                 talks to the camera
control_arbiter_node    decides which command wins    ← the important one
```

Each can crash, restart, or be replaced without touching the others.

### Idea 2: Topics (publish / subscribe)

Nodes don't call each other. They **broadcast** on named channels called
topics, and anyone interested **listens**.

```
   usb_cam  ──publishes──►  /image_raw/compressed  ──►  anyone listening
```

The publisher has no idea who is listening, or whether anyone is. This is why
you can start a debugging tool at any moment and watch live data.

### Idea 3: Messages (typed data)

Every topic carries one specific data type. `/scan` carries `LaserScan` — an
array of 404 distances plus an angle for each. `/cmd_vel` carries `Twist` —
three numbers: forward speed, sideways speed, spin speed.

### Idea 4: TF (the coordinate tree)

The robot is a physical object with parts at known positions. **TF** is the
system that answers "where is the laser relative to the wheels?"

```
map                     ← the room (from SLAM)
 └─ odom                ← dead reckoning (from wheels + IMU)
     └─ base_footprint  ← the floor under the robot
         └─ base_link   ← centre of the robot
             ├─ laser      ← 15 cm up, 1 cm forward
             ├─ cam1_Link  ← pan joint
             │   └─ cam2_Link ← tilt joint (the actual camera)
             └─ imu_link
```

Without this tree, a laser reading of "2 metres" is meaningless — 2 metres
*from what, in which direction?*

---

## Part 3 — The core design: the Arbiter

This is the single most important idea in the project.

### The problem, restated

Three controllers, one set of motors. Naive design:

```
keyboard ──┐
gaze     ──┼──► /cmd_vel ──► motors     ← DISASTER
eeg      ──┘
```

### The solution

**Exactly one node is allowed to publish `/cmd_vel`.** Everyone else publishes
to their own private topic, and the arbiter picks one.

```
keyboard ──► /cmd_vel/keyboard ──┐
gaze     ──► /cmd_vel/gaze     ──┼──► [ARBITER] ──► /cmd_vel ──► motors
                                 ──┘        ▲
                                            │
                                /control_mode (which one wins?)
```

The EEG is NOT a velocity source. Protocol v1 sends toggles, so it has no
`/cmd_vel/eeg` at all - it publishes `/eeg/run`, `/eeg/mode` and `/eeg/link`,
and `gaze_control_node` gates the GAZE command on them. An EEG gesture
therefore decides *whether* and *what* gaze steers, never how fast.

The arbiter does four jobs:

**1. Pick one.** Mode `1` → only keyboard is forwarded. Mode `2` → only gaze.
The others are silently ignored. They can't fight because they're not in the
same lane.

**2. Clamp.** Even if a controller asks for 5 m/s, the robot gets 0.25 m/s.

| Axis | Limit | Why |
|---|---|---|
| forward `vx` | 0.25 m/s | slow enough to react |
| sideways `vy` | 0.20 m/s | mecanum strafing is less stable |
| spin `ωz` | 0.80 rad/s | ~46°/s |

**3. Watchdog — the safety net.** If no command arrives for **0.35 seconds**,
publish zero. This is the dead-man switch.

Think about why this matters: your Wi-Fi drops. The laptop is still "sending"
as far as it knows. Without a watchdog, the last command ("forward 0.2 m/s")
stays latched in the motor board and the robot drives into a wall.

With it, 350 ms later the robot stops. **This is the single most important
safety property in the system.**

**4. Latch E-STOP.** Mode `0` isn't just "stop now" — it keeps publishing zero
until you *explicitly* pick another mode. You can't accidentally leave it.

### Why the arbiter runs on the robot, not your PC

The handbook said put it on the PC. **That's wrong, and here's why:**

> A watchdog on the far side of a Wi-Fi link cannot stop a robot whose Wi-Fi
> link just died.

If the arbiter lived on the PC and Wi-Fi dropped, nothing publishes `/cmd_vel`
at all — and the robot keeps its last command forever. The safety layer must be
**downstream of the unreliable link**. So it runs on the Pi.

You still control the mode from the PC. You just can't *remove* the safety
layer.

---

## Part 4 — The three modes in detail

### Mode 1: Keyboard (mecanum + gimbal)

The M1 has **mecanum wheels** — rollers at 45° on each wheel. This gives it
superpowers a normal car doesn't have:

```
        W  forward              Q  strafe left       A  spin left
        S  backward             E  strafe right      D  spin right

        I / K   tilt head up / down
        J / L   pan head left / right
        U       re-centre
```

It can move sideways without turning. It can spin in place. All three axes
(`vx`, `vy`, `ωz`) work independently and simultaneously.

**Try this first.** It's the mode that proves the whole chain works.

### Mode 2: Gaze (the clever part)

You look at the screen; the robot reacts. But raw gaze is unusable:

- Your eyes **twitch constantly** (saccades) — several per second, involuntarily
- You blink
- You glance at the corner for 50 ms and back

If you mapped gaze straight to motion, the robot would jerk randomly.

**The fix — a two-zone scheme:**

```
┌─────────────────────────────────────────┐
│   TURN LEFT  │   FORWARD   │ TURN RIGHT │
│──────────────┼─────────────┼────────────│
│              │   ╭─────╮   │            │
│  STRAFE LEFT │   │ DEAD│   │STRAFE RIGHT│
│              │   │ ZONE│   │            │
│              │   ╰─────╯   │            │
│──────────────┼─────────────┼────────────│
│              │  BACKWARD   │            │
└─────────────────────────────────────────┘
```

**Inside the circle (deadzone, 25% of half-screen):** the wheels stay still.
Gaze offsets move the **camera head** instead. So you can look around the room
without the robot moving. This is the "free look" behaviour.

**Outside the circle:** the direction you look becomes a drive command — but
only after you **hold it for 0.40 seconds**. That dwell timer is what rejects
saccades. A glance doesn't move the robot; a deliberate look does.

And a third safety: if gaze data stops arriving for 0.6 s, velocity goes to
zero. A crashed tracker can't leave the robot driving.

### Mode 3: EEG (the noisy one)

The Muse 2 headband measures tiny voltages on your scalp. Four gestures:

| Gesture | Detected by | Command |
|---|---|---|
| Jaw clench | muscle noise (EMG, 30–110 Hz) | `STOP` |
| Double blink | two blink peaks 0.2–0.5 s apart | `GO` |
| Look left | eye-movement spike + return spike | `LEFT` |
| Look right | same, opposite direction | `RIGHT` |

**Why these four and not "think forward"?** Because real motor-imagery BCI
needs training and is unreliable. Jaw clench and blinks produce signals
*tens to hundreds of times larger* than brainwaves. It's pragmatic, not
magical.

**The noise problem:** one misclassification shouldn't move the robot. So the
bridge keeps the last 3 classifications and requires a **majority vote**.
`STOP` bypasses the filter entirely — stopping is never delayed.

**Why `LEFT`/`RIGHT` rotate in place here:** the original project was written
for an Ackermann (car-like) robot that can't turn on the spot, so it drove a
1.5-second arc. Your M1 is mecanum — it can just spin. Simpler and safer.

---

## Part 5 — Where everything runs

The work is split because the two machines are good at different things.

```
┌──────────────────────────────────────────────────────────────┐
│  OPERATOR PC (VMware Ubuntu, 10.0.0.102)                     │
│                                                              │
│   • slam_toolbox   builds the map from the laser             │
│   • RViz           draws it                                  │
│   • gaze model     watches your webcam                       │
│   • EEG pipeline   talks Bluetooth to the headband           │
│                                                              │
│   Needs: CPU, a screen, a webcam, Bluetooth                  │
└───────────────────────────┬──────────────────────────────────┘
                            │ Wi-Fi, ROS_DOMAIN_ID=30
                            │ (all topics flow both ways)
┌───────────────────────────┴──────────────────────────────────┐
│  RASPBERRY PI 5 (10.0.0.54, inside Docker)                    │
│                                                              │
│   • chassis driver  serial → motors                          │
│   • LiDAR driver    serial → 404 distances                   │
│   • camera driver   USB → JPEG frames                        │
│   • CONTROL ARBITER ← safety lives here, next to the motors  │
│   • gaze/EEG UDP bridges                                     │
│                                                              │
│   Needs: USB ports, serial ports, and to be near the motors  │
└──────────────────────────────────────────────────────────────┘
```

**Rule of thumb:** if it touches hardware, it runs on the Pi. If it needs a
human (screen, webcam, headset), it runs on the PC.

### Why a Docker container?

The Pi's own operating system stays clean. ROS 2 Humble lives in a container
with all the Yahboom drivers pre-installed. You can destroy and rebuild the
container without touching the host. Everything that matters is on the host and
mounted in:

```
/home/pi/m1_control_ws  →  /root/m1_control_ws   (our code)
/home/pi/eeg_bridge     →  /root/eeg_bridge      (EEG bridge)
/dev                    →  /dev                  (all hardware, live)
```

That last one matters more than it looks. See the bugs section.

---

## Part 6 — The four-layer stack

Never modify vendor code. Stack your code on top.

```
Layer 4  /root/m1_control_ws          ← YOUR CODE (8 nodes)
             ▲
Layer 3  yahboomcar_ws                ← chassis driver, messages
             ▲
Layer 2  library_ws                   ← LiDAR, camera, SLAM libs
             ▲
Layer 1  /opt/ros/humble              ← core ROS 2
```

Each layer is "sourced" after the one below, so your code wins conflicts.

Why it matters: if you edit vendor files, a driver update wipes your work. If
you stack on top, updating is free.

---

## Part 7 — The bugs, as lessons

These are the most instructive part of the project. Every one was **silent** —
no crash, no error, just wrong behaviour.

### Lesson 1: "No data" ≠ "broken hardware"

The LiDAR reported:

```
Error, cannot retrieve YDLidar health code: ffffffff
Failed to start scan mode: ffffffff
```

A raw serial probe then read **zero bytes at every baud rate from 9600 to
921600**. That looks exactly like dead hardware. I nearly told you to check the
wiring.

Then the vendor's own test tool worked perfectly:

```
Model: Tmini Plus     health status: good     Now lidar is scanning...
```

**The cause:** the LiDAR's microcontroller wasn't ready the instant USB
enumeration finished. A timing race.

**The fix:** `respawn=True, respawn_delay=5.0` — it now retries itself.

> **Lesson:** before touching hardware, cross-check with the vendor's own tool.
> "No bytes" is evidence, not proof.

### Lesson 2: Counting is harder than it looks

I wanted 20 frames/second out of a 30 fps camera. My first limiter gave
**15**.

Why? A naive "wait at least 50 ms between frames" rule: after emitting, the
next frame arrives 33 ms later — too soon, skip it. The one after is 66 ms —
emit. So you emit every 66 ms = 15 Hz. The maths silently quantises to
`input / 2`.

The fix is a **credit accumulator**: each frame adds its elapsed time to a
budget, and you spend one period per emission. But the cap on that budget
matters — capping at 1 period throws away the remainder and collapses straight
back to 15 Hz. It needs 2.

I verified this numerically *before* changing code:

```
cap = 1× period : 15.00 Hz     ← the bug
cap = 2× period : 19.90 Hz     ← the fix
```

Measured after: **19.9 Hz**.

> **Lesson:** when a rate is wrong by a suspicious factor, suspect quantisation,
> not performance.

### Lesson 3: Advice has a context

Widely-copied ROS 2 tuning says:

```
net.core.rmem_default=16777216
```

That was written for a **Pi 3B+ with 1 GB RAM**. On your Pi 5 it's actively
harmful, because:

| Setting | Applies to | Verdict |
|---|---|---|
| `rmem_max` | A **ceiling** — costs nothing until requested | ✅ keep |
| `rmem_default` | **Every socket** — ssh, apt, VS Code | ❌ remove |

Your Pi was at 1.4 GB free with 785 MB already swapped. Inflating every socket
would have made it worse.

I also added `vm.swappiness=10` so ROS nodes don't get paged out mid-cycle — at
the default 60, a swapped-out control loop can miss the 350 ms watchdog.

> **Lesson:** always ask *what a setting applies to*, not just *what value*.

### Lesson 4: Invisible missing pieces

The camera frames were absent from the TF tree. No error, nothing in the log —
they just weren't there. RViz simply couldn't place the image.

Why: `cam1` and `cam2` are **revolute** (rotating) joints. `robot_state_publisher`
can't compute their position without joint states. Fixed joints work fine;
revolute ones need a `joint_state_publisher`.

> **Lesson:** a missing transform is silent. Always check the TF tree.

### Lesson 5: The bug that looked like broken hardware

You saw this one:

```
[ydlidar] [error] Check Sum 0xC5A3 != 0x2191     (hundreds of lines)
[usb_cam] terminate called after throwing an instance of 'char*'
[ekf_node] Failed to meet update rate!
```

Four separate-looking failures. One cause: **five stacks running at once**.

- 4 LiDAR drivers reading `/dev/rplidar` simultaneously — each steals bytes from
  the others, so every frame fails its checksum
- Only one process can own `/dev/video0` — the extras abort
- 4× EKF nodes competing for CPU

I caused this: `./m1.sh start` had no guard, so every invocation added a stack.

Fixed with a `flock` singleton lock. A second start now refuses:

```
 REFUSING TO START: a robot stack is already running.
```

> **Lesson:** when several unrelated things fail together, look for one shared
> cause — often a resource conflict, not multiple hardware faults.

### Two more, briefly

**Throttle bug's cousin:** the EEG bridge clamped *steering* (rad/s) using the
*linear* speed limit (m/s). A configured 0.60 rad/s was silently crushed to
0.40. Different units need different limits.

**The self-killing cleanup:** `pkill -f "ros2 launch"` inside a `docker exec`
matched **its own command line** and killed the session. Patterns must never
match the process that runs them.

### Lesson 6: Some bugs can only be found by watching

The servo axes were backwards in every comment, parameter name and line of gaze
maths. No log, no test, no amount of reading could reveal it — `s1` looked
exactly as valid when it was panning instead of tilting.

It took one command and a pair of eyes:

```
{s1: 70, s2: 90}   ->  head tilts UP      so s1 is TILT
{s1: 40, s2: 120}  ->  head turns LEFT    so s2 is PAN
```

> **Lesson:** automated tests prove the code does *what you specified*. They
> cannot tell you that you specified the wrong thing. For anything physical,
> watch it move once.

### Lesson 7: A "level" topic is not an "edge" topic

The buzzer latched on forever after one mode change. The driver treats
`/Buzzer` as a level:

```python
if msg.data: set_beep(1)    # ON, and stays on
else:        set_beep(0)    # OFF
```

Publishing `true` and never `false` is a permanent beep. This is common: LEDs,
relays and enable lines all behave this way.

> **Lesson:** before publishing to an actuator, find out whether it is
> edge-triggered or level-held. If it is level-held, you own both halves of the
> signal.

### Lesson 8: Test teardown by running it twice

Three separate bugs hid behind each other in `stop`:

1. the supervisor holding the lock was never killed;
2. every child inherited the lock file descriptor, so the whole tree held it;
3. the EKF cleanup pattern matched its *node name* (`ekf_filter_node`) instead
   of its *binary* (`ekf_node`), orphaning four processes.

Each one was invisible until the previous one was fixed. And because the
singleton guard refused to start a second stack, the failure surfaced as
*"a robot stack is already running"* — which sounds like a state problem, not
three cleanup bugs.

> **Lesson:** start→stop→start. If you only ever test one direction, you are
> testing half the system. And a guard that blocks you is often hiding the bug
> rather than being the bug.

### Lesson 9: "All my topics are flowing" is not "it works"

This is the most expensive bug in the project, and the most deceptive.

The robot ran one DDS middleware (FastDDS), the operator PC another
(CycloneDDS). Every stream anyone would think to check was perfect:

| Check | Result |
|---|---|
| DDS discovery | 39 topics visible |
| `/scan` | 10.12 Hz |
| `/image_throttled/compressed` | 20.27 Hz |
| `/odom` | 9.97 Hz |
| `/tf` | 20.05 Hz |

And SLAM produced **no map at all** — no error, just silence.

The cause is subtle. ROS publishes transforms on two topics:

- **`/tf`** — the moving transforms, published continuously
- **`/tf_static`** — the fixed ones (where the laser sits on the chassis),
  published **once** and *latched* so late joiners still receive it

"Latched" means `TRANSIENT_LOCAL` durability, and that specific QoS is **not
reliably delivered between different DDS vendors**. So the dynamic transforms
arrived, the static one never did, the robot had no idea where its own laser
was, and slam_toolbox quietly did nothing.

```
$ ros2 run tf2_ros tf2_echo base_link laser
Invalid frame ID "base_link" passed to canTransform argument target_frame
```

One line of output identified it. Fixing it was changing one environment
variable so both machines used the same middleware:

```
base_link -> laser        Translation: [0.006, 0.000, 0.149]     OK
/map                      93 occupied, 741 free, 151x41 @ 5 cm   OK
```

> **Lesson:** a latched topic can be missing while every streaming topic looks
> perfect. When a consumer silently does nothing, check the **transforms** and
> the **latched** topics first. And treat a mixed-vendor DDS setup as a defect,
> not a configuration choice — the two ends must match.

### Lesson 10: One node can steal the whole robot

With a gaze node merely *running* — not even being used — keyboard driving
stopped working. No error, no warning; pressing `W` simply did nothing.

The gaze HUD published "I am the active mode" once a second, on a timer,
whether or not it was tracking anything. The arbiter obeyed, so it looked like
this:

```
Mode -> KEYBOARD   (you pressed 1)
Mode -> GAZE       (16 milliseconds later)
```

The keyboard topic was then correctly ignored — the robot genuinely was in gaze
mode. Everything behaved exactly as written; the *design* was wrong.

**Fix:** a control source must never claim the mode on its own. `assert_mode`
now defaults to off, and the HUD only offers the mode while it is actually
tracking a face.

> **Lesson:** the mode topic is an **authority** channel. Anything that writes
> to it is claiming the robot. A source should only ever *offer* commands on
> its own topic and let the operator decide who is in charge.

### Lesson 11: The number you wrote down twice

The head turned much further one way than the other. The configuration was
symmetric, the arbiter was symmetric, the teleop node was symmetric. The bug was
in the operator GUI, which had its own copy of the limits typed into the code:

```python
self.servo['s2'] = min(110.0, max(0.0, ...))   # 90 right, 20 left
```

Nobody had written a wrong number. They had written the *right* number in two
places, and only one of them got fixed.

Worse: the GUI on the operator PC had never been re-synced with the robot, so
the two machines disagreed and neither was internally inconsistent.

**Fix:** the GUI reads the same ROS parameters as the arbiter. There is now one
definition, and a drifting copy is impossible rather than merely discouraged.

> **Lesson:** if two processes must agree on a number, make one of them read it
> from the other. Never write it down twice — the second copy is the one that
> gets forgotten.

### Lesson 12: Some failures have no error message

Three of this project's hardest bugs shared a property: **when they failed, they
said nothing.**

| Bug | What you saw | What was actually wrong |
|---|---|---|
| DDS domain never set (§7.13) | GUI showing `no data` | Process on a different DDS network |
| Multicast over Wi-Fi (§7.14) | Camera at 2 Hz | 7 of every 8 frames silently dropped |
| Duplicated limit (above) | Asymmetric servo | Two copies of one number |

None produced a warning. None produced a log line. Each one looked exactly like
"the feature is broken" rather than "the environment is wrong".

The fixes that worked were all the same shape — **make the silent thing loud**:

- The GUI compares its own environment against expected values at startup and
  shows a red banner naming the mismatched variable.
- The camera rate is displayed as a number, so 2 fps is visible rather than
  invisible.
- Servo limits live in one place, so there is no second value to compare.

> **Lesson:** a failure that can only happen silently deserves an explicit
> startup assertion. "It should be configured correctly" is not a check.

### Lesson 13: Measure the biggest thing, on the receiving end

"The small messages arrive" felt like proof that the network was fine. Mode
state at 1 Hz arrived perfectly. Voltage at 1 Hz arrived perfectly. So the
camera must be broken.

It was not. A 100-byte message and a 58 000-byte message have completely
different survival rates over Wi-Fi multicast, and measuring `ros2 topic hz` on
the *publisher* proved nothing about what arrived.

The number that mattered came from running the measurement on the **receiving**
machine:

```
publisher:  20.0 Hz      receiver:  2.6 Hz
```

> **Lesson:** when a stream looks broken, measure its rate where it is
> *consumed*, not where it is produced. And measure the largest topic — small
> topics will happily survive a transport that cannot carry the big ones.

### Lesson 14: A GUI is not thread-safe, and it does not tell you

Pressing a button in the console killed the whole program:

```
QObject::connect: Cannot queue arguments of type 'QTextBlock'
QObject::connect: Cannot queue arguments of type 'QTextCursor'
[ros2run]: Segmentation fault
```

The button ran an SSH command on a worker thread so the window would stay
responsive — good instinct. But when the command finished, that thread wrote the
result into the log box. Qt widgets belong to the GUI thread, and moving a
`QTextBlock` between threads needs a type registration that was never done. The
result is not an exception you can catch; it is a crash.

The fix is one small idea: worker threads are only allowed to **emit a signal**.
Qt then delivers it on the GUI thread for you.

```python
log_signal = QtCore.pyqtSignal(str)     # worker -> log widget

def logline(self, text):                # safe from any thread
    self.log_signal.emit(f'{time.strftime("%H:%M:%S")}  {text}')
```

The same file had the opposite problem in the other direction: an SSH probe on
the GUI thread with a 10-second timeout, every 5 seconds. That does not crash —
it just freezes the window, which users report as "it hung".

> **Lesson:** if work leaves the GUI thread, the *only* permitted way back is a
> signal. And check the reverse too — blocking I/O on the GUI thread makes a
> program that is working correctly look broken.

---

## Part 8 — What's finished

### ✅ Robot side — complete and validated

| Component | Evidence |
|---|---|
| Chassis driver | edition `3.6`, battery 12.3 V |
| Mecanum + PTZ teleop | servo round-trip verified `(60,70)` |
| LiDAR Tmini Plus | **10.03 Hz**, 404 beams, 328 real returns |
| USB camera | 30.8 fps capture → **20.2 Hz** throttled |
| Arbiter + watchdog | **12/12 safety checks** |
| Gaze control | zone mapping + PTZ verified |
| EEG bridge | `GO` → 0.15 m/s, link-loss stop 0.88 s |
| Gaze + EEG fusion | **5/5 priority tests** |
| Two-way voice | 161/161 PCM packets, per-direction mute |
| Container | hot-plug works, 0 zombies |

### ✅ Operator PC — verified working

| Component | Evidence |
|---|---|
| ROS 2 Humble | present |
| DDS discovery | unicast peers, **20.2 Hz** video |
| All streams | 20.20 / 10.02 / 10.59 / 20.91 Hz |
| `slam_toolbox` | **78 occupied cells**, map 73×149 @ 5 cm |
| Operator console | 6 panels, no layout defects, live camera |
| Gaze calibration | **8/8 checks** |

### ⚠️ Not yet confirmed

| Item | Why | Effort |
|---|---|---|
| Gaze model calibration | needs you in front of the webcam | 5 min |
| EEG with real headset | needs the Muse 2 | 10 min |
| Floor trial | needs space + a spotter | 15 min |
| Operator-side microphone | VMware is not passing the Windows mic through | — |

---

## Part 9 — Where to start

Do these **in order**. Each one builds on the last, and each has a clear pass
condition.

### Step 1 — Confirm the LiDAR LED (30 seconds)

The blinking green LED is **normal**. Solid = powered but idle; blinking =
scanning.

But the LED is not the test. This is:

```bash
docker exec ros-humble-dev bash -lc \
  'source /root/m1_control_ws/scripts/env.sh >/dev/null 2>&1; ros2 topic hz /scan'
```

**Pass:** ~10 Hz, steady. You just saw it at 10.03 Hz with 328 real returns.

### Step 2 — Servo direction (already confirmed)

The head's axes are **not** what the parameter names suggest:

| | Axis | Larger value |
|---|---|---|
| **`s1`** | **TILT** (up/down) | look **up** |
| **`s2`** | **PAN** (left/right) | look **left** |

This was verified by hand: `s1` 0 = down, 30 = level, 70 = up; `s2` 60 = right,
90 = centre, 120 = left. Rest pose is `s1=40, s2=90`.

What is now **measured** is the travel range. The head carries the camera, so a
moving servo shows up as a moving image — that turns the camera into a servo
position sensor with no extra hardware.

Pan (`s2`) moves freely and linearly from 0 to about 112, then **stops dead**:
at 112, 116 and 120 the image does not change at all (~980 matched features, zero
shift). That flat tail is the firmware clamp. With a centre of 90 this gives 90
units of travel right but only 20 left — exactly the asymmetry that feels like
"it turns further one way".

Configured limits are therefore **`s1` 5..75** and **`s2` 80..100**. Pan is
narrowed well inside the mechanical stop on purpose: a camera that swings ~90°
loses the view straight ahead, and driving forward while blind is the dangerous
failure. Tilt stays wide because looking down in front of the robot is useful.

```bash
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 90}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 5, s2: 90}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 75, s2: 90}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 80}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 100}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 90}"
```

If a limit needs changing, set `servo_s1_min/max` or `servo_s2_min/max` in
`config/m1_params.yaml`. Every node that touches the servos — including the
operator console — reads them as ROS parameters, so there is no second copy to
drift out of sync.

### Step 3 — Keyboard drive (2 minutes)

**Wheels on the floor, open space, hand near the power switch.**

```bash
./m1.sh teleop
```

Press `W`. The robot should creep forward at 0.20 m/s. Press `A` — it spins
left. Press `Q` — it strafes sideways (this is the mecanum party trick).

Press `SPACE` for E-STOP. **It should stop instantly and stay stopped** even if
you keep pressing `W`. That's the latch working. Press `1` to resume.

**Pass:** all three axes work, E-STOP latches.

### Step 4 — Watch it on the PC (5 minutes)

On the VM:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
ros2 run rviz2 rviz2
```

Add a **LaserScan** display, topic `/scan`, reliability **Best Effort**.

> ⚠️ **Best Effort is not optional.** I hit this myself while testing just now
> — a Reliable subscriber connects, reports *"offering incompatible QoS. No
> messages will be received"*, and shows nothing. It's the most confusing
> failure mode in ROS 2.

You should see a fan of green points. **Pass:** the fan matches the room.

### Step 5 — Build the map (5 minutes)

Still on the VM:

```bash
ros2 launch m1_control m1_operator.launch.py slam:=true rviz:=true
```

Add a **Map** display on `/map`. Then drive the robot around with
`./m1.sh teleop` on the Pi.

**Pass:** walls appear and stay put as you move.

> The map only grows **while the robot moves**. A stationary robot gives a tiny
> map — that's correct, not a bug.

### Step 6 — Gaze, without your eyes (2 minutes)

Before involving your face, prove the pipeline works:

```bash
python3 /root/m1_control_ws/scripts/test_gaze.py --hold 0.9
```

**Pass:**

```
forward        FORWARD        0.20  0.00  0.00
strafe right   STRAFE_RIGHT   0.00 -0.20  0.00
after silence: STOPPED (pass)
```

### Step 7 — Gaze, with your eyes (10 minutes)

Only now involve the webcam. See `operator-pc/SETUP.md` §7. Calibration takes
~3 minutes and four postures.

### Step 8 — EEG (10 minutes)

Needs the Muse 2 and BlueMuse. See `operator-pc/SETUP.md` §8.

---

## Part 10 — Daily operation

Once everything is confirmed, the routine is three commands:

```bash
ssh pi@10.0.0.54
cd /home/pi
./m1.sh start          # everything
```

Then on the PC:

```bash
ros2 launch m1_control m1_operator.launch.py
```

Stop with `./m1.sh stop` or `./m1.sh stop --hard`.

### The commands you'll actually use

| Command | When |
|---|---|
| `./m1.sh start` | begin work |
| `./m1.sh stop` | end work |
| `./m1.sh diag` | something feels wrong |
| `./m1.sh status` | what's running? |
| `./m1.sh teleop` | drive manually |
| `./m1.sh logs` | watch the log |

### The three checks that solve most problems

```bash
# 1. Is only ONE stack running?   (should print 2)
docker exec ros-humble-dev bash -lc 'ps -eo cmd | grep -c "[r]os2 launch"'

# 2. Is the arbiter the only publisher?   (should print 1)
docker exec ros-humble-dev bash -lc \
  'source /root/m1_control_ws/scripts/env.sh >/dev/null 2>&1; ros2 topic info /cmd_vel'

# 3. Is data flowing?   (should be ~10 Hz)
docker exec ros-humble-dev bash -lc \
  'source /root/m1_control_ws/scripts/env.sh >/dev/null 2>&1; ros2 topic hz /scan'
```

If all three are right, the robot side is fine and the problem is on the PC.

---

## Part 11 — How to think about debugging

The pattern behind all of them:

### 1. Verify at the lowest level first

Before ROS, before config: does the hardware talk at all?

```
vendor tool → raw serial → ROS driver → whole system
```

The LiDAR "failure" was solved by going **down** a level, not up.

### 2. When several things fail together, find the shared cause

`Check Sum` errors + `usb_cam` crash + EKF rate warning looked like three
hardware faults. It was one: five stacks.

Ask: *what do these have in common?* Answer: they all wanted exclusive access
to something.

### 3. A silent failure is more dangerous than a loud one

The missing camera TF and the 15 Hz throttle produced **no errors at all**.
Both would have shipped. That's why there's a 12-check safety suite instead of
"it looked fine".

### 4. Measure, don't assume

Every claim in the report has a number behind it: 10.03 Hz, 0.38 s, 328 returns,
78 cells. "It seems to work" is how the 15 Hz bug survived.

### 5. Prefer the fix that removes the failure mode

I could have told you "don't run start twice". Instead `start_robot.sh` now
refuses. A guard is better than a warning, because guards work when you forget.

---

## Part 12 — Quick reference

### Files that matter

| Path | What |
|---|---|
| `robot/SETUP.md` | step-by-step install procedure |
| `robot/docs/report.md` | engineering report, all measurements |
| `operator-pc/SETUP.md` | PC-side setup |
| `robot/m1_control_ws/src/m1_control/` | the 8 nodes |
| `robot/m1_control_ws/scripts/` | start/stop/diagnose/validate |

### The one diagram to remember

```
  keyboard ──► /cmd_vel/keyboard ──┐
  gaze     ──► /cmd_vel/gaze     ──┼──► [ARBITER] ──► /cmd_vel ──► motors
                                 ──┘   0.35 s watchdog
  eeg      ──► /eeg/run|mode ──► gaze_control_node  0.40 m/s clamp
```

Everything else is detail.

### Numbers to remember

| | |
|---|---|
| Watchdog | **0.35 s** |
| Max speed | **0.25 m/s** |
| Scan rate | **10 Hz** |
| Video rate | **20 Hz** |
| Dwell (gaze) | **0.40 s** |
| Domain ID | **30** |
