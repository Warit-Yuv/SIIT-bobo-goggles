# ROSMASTER M1 — Robot-Side Setup Report

**Robot:** `raspberrypi` (Raspberry Pi 5, 8 GB) — **`10.0.0.54`**
**Operator PC:** `Rescue-Ubuntu` VMware VM — **`10.0.0.102`**
**ROS 2 Humble** inside Docker container `ros-humble-dev` — **`ROS_DOMAIN_ID=30`**
**Date:** 2026-11-08
**Status:** Robot side complete and validated. 12/12 safety checks pass.

> **This document is an engineering report**, not a procedure: it records what
> was built, what broke, and the measured evidence for each claim.
> For the **step-by-step setup procedure**, see [`../SETUP.md`](../SETUP.md).

---

## 1. Executive Summary

The robot is fully operational. Everything in the handbook's Layer 4 overlay is
implemented, built and validated against real hardware:

| Capability | State | Evidence |
|---|---|---|
| Chassis driver (STM32) | **Working** | `/dev/myserial`, edition `3.6`, 12.3 V |
| Mecanum + PTZ teleop | **Working** | Arbiter→`/cmd_vel` 1:1, servo round-trip verified |
| Servo axis mapping | **Working** | `s1`=tilt, `s2`=pan — confirmed by hand (§10) |
| YDLIDAR Tmini Plus | **Working** | 10.03 Hz `/scan`, 360°, 328 real returns |
| USB camera 640×480 | **Working** | 30.8 Hz capture, throttled to 20.0 Hz |
| Control arbiter + watchdog | **Working** | 12/12 safety checks, stop in 0.38 s |
| Gaze control pipeline | **Working** | Zone mapping + PTZ axes verified |
| EEG UDP bridge | **Working** | `GO`→vx 0.15, link-loss stop in 0.88 s |
| SLAM (`slam_toolbox`) on VM | **Working** | 78 occupied cells, map 73×149 @ 5 cm |
| Cross-machine DDS | **Working** | Unicast peers, 20.2 Hz video Pi→VM (§7.14) |
| start / stop / restart | **Working** | Two consecutive cycles, 0 leftovers |
| Two-way voice relay | **Working** | 161/161 PCM packets, per-direction mute |
| GAZE+EEG fusion mode | **Working** | 5/5 priority tests (gaze steers, EEG stops) |
| Operator console (PyQt5) | **Working** | 6 panels, in-window teleop, gaze calibration |
| Gaze calibration | **Working** | 8/8 checks, 5-point offset+gain fit |

**Eighteen defects were found and fixed during validation.** They are documented
in §7 because each one would have been a silent failure in operation. Seven of
them (§7.12 – §7.18) were found only by testing on the real operator station;
most produced no error message at all when they failed.

---

## 2. What Was Actually Found vs. the Handbook

The handbook is accurate in architecture but several hardware facts differ from
what the real machine reports. These differences matter:

| Handbook says | Reality on this robot |
|---|---|
| YDLIDAR **Tmini** | **Tmini Plus**, FW 1.2, HW 1, S/N `2026000800080141` |
| LiDAR at `/dev/ydlidar` | **`/dev/rplidar`** (udev rule maps `10c4:ea60`) |
| Servo centre `s1=90, s2=35` | **`s1=40, s2=90`** — and **`s1` is tilt, `s2` is pan** (verified by hand, §10) |
| CycloneDDS recommended | **Required on both machines.** A mismatch silently breaks `/tf_static` and SLAM (§7.10) |
| Arbiter runs on operator PC | **Moved to the Pi** (see §3.1) |
| `topic_tools throttle` | **Not installed** — replaced with our own node |
| `laser_link` TF frame | URDF publishes **`laser_Link`**; driver publishes **`laser`** |
| Camera frame `camera_link` | URDF has **`cam1_Link` → `cam2_Link`** (revolute joints) |

### Why this matters

An earlier version of this report claimed CycloneDDS was not installed and
therefore had to be disabled. **That was wrong**, and acting on it cost real
time. Both machines have `rmw_cyclonedds_cpp`:

```
$ ls /opt/ros/humble/lib/ | grep -E '^librmw_.*\.so'
librmw_cyclonedds_cpp.so      <- present, and required
librmw_fastrtps_cpp.so
librmw_implementation.so
```

More importantly, the two machines must use the **same** implementation.
`/tf_static` is published with `TRANSIENT_LOCAL` durability, and that QoS is
not reliably delivered across different DDS vendors. Measured with the Pi on
FastDDS and the operator PC on CycloneDDS:

| | Result |
|---|---|
| `/tf` (dynamic) | 20.05 Hz — fine |
| `/tf_static` (`base_link`→`laser`) | **never arrived** |
| `/map` | **never produced** — slam_toolbox silently did nothing |

`scripts/env.sh` therefore exports `rmw_cyclonedds_cpp` on the robot, and
`operator-pc/SETUP.md` does the same on the PC. See §7.10.

---

## 3. Architecture Decisions (and why they differ from the handbook)

### 3.1 The arbiter runs on the robot, not the PC

The handbook (§10.2) puts `control_arbiter_node` on the operator PC. That is
unsafe:

> A watchdog on the far side of a Wi-Fi link cannot stop a robot whose Wi-Fi
> link just died.

If the PC holds the arbiter and Wi-Fi drops, nothing publishes `/cmd_vel` at
all — the last setpoint stays latched in the STM32 until its own timeout. The
watchdog must be *downstream* of the unreliable link. It now runs on the Pi, so
the 350 ms dead-man switch keeps working even when the operator is gone.

Mode commands still come from anywhere (`/control_mode`), so the PC keeps full
authority — it just cannot *remove* the safety layer.

### 3.2 Only one node may publish `/cmd_vel`

Yahboom's `yahboomcar_bringup_M1_launch.py` starts `yahboom_joy_M1`, which
publishes `/cmd_vel`. Combined with an arbiter that is two publishers fighting
over one topic — the handbook's own Issue #2 ("robot drives erratically").

`m1_robot_bringup.launch.py` therefore composes the hardware nodes **explicitly**
instead of including Yahboom's launch file, and simply omits the joy node.
Verified: `/cmd_vel` has exactly 1 publisher.

### 3.3 Gaze policy on the Pi, gaze *source* on the PC

`gaze_control_node` (deadzone + dwell + velocity shaping) runs on the robot.
Only the raw gaze point comes over the network.

Reason: the deadzone/dwell logic is a **motion authority**. Running it locally
means a Wi-Fi stall cannot leave a stale driving command in flight. It also
means the policy is written once, whether the gaze comes from MediaPipe in the
VM, the GazeModel Windows app, or a future neural model.

### 3.4 LiDAR is optional and self-healing

`lidar:=auto` checks whether `/dev/rplidar` exists *at launch time*. If absent,
bringup logs a warning and starts everything else — the robot is not dead just
because one sensor is unplugged. The node is launched with `respawn=True,
respawn_delay=5.0`, which also recovers it after a hot-plug.

---

## 4. Container Configuration

The original container had a real flaw: it bind-mounted `/dev/myserial` and
`/dev/ttyUSB0` **individually**. That freezes the device list at container-start
time, so unplugging and replugging the LiDAR left a dead node forever.

`/home/pi/m1_docker.sh` now creates it correctly:

```bash
docker run -d --name ros-humble-dev \
    --init \                      # tini as PID 1 -> reaps zombies
    --net=host --privileged \
    -v /dev:/dev \                # LIVE device tree, hot-plug works
    -v /home/pi/m1_control_ws:/root/m1_control_ws \
    -v /home/pi/eeg_bridge:/root/eeg_bridge \
    -e ROS_DOMAIN_ID=30 \
    ... sleep infinity
```

Two fixes worth calling out:

* **`-v /dev:/dev`** — the udev symlinks `/dev/myserial` and `/dev/rplidar` now
  appear and disappear live. This was proven after the reboot: `ttyUSB`
  numbering swapped (`myserial`→`ttyUSB1`, `rplidar`→`ttyUSB0`) and both
  symlinks still resolved correctly.
* **`--init`** — without it, PID 1 is `sleep infinity`, which never reaps
  orphans. Every killed `ros2 launch` left a zombie. Measured before: 9
  accumulating zombies. After: `kill_all.sh` reports **0**.

Everything stateful lives on the host, so the container is disposable:

```
/home/pi/m1_control_ws   ->  /root/m1_control_ws
/home/pi/eeg_bridge      ->  /root/eeg_bridge
/home/pi/share           ->  /root/share
```

---

## 5. Workspace Layout

```
/home/pi/
├── m1.sh                     # host control panel (start/stop/diag/teleop/...)
├── m1_docker.sh              # create/attach/status the container
├── m1_backup/                # pre-change backups
└── m1_control_ws/            # Layer 4 overlay (git-able)
    ├── src/m1_control/
    │   ├── m1_control/
    │   │   ├── control_arbiter_node.py    # safety core
    │   │   ├── wasd_teleop_node.py        # WASD+QE / IJKL
    │   │   ├── gaze_control_node.py       # deadzone + dwell policy
    │   │   ├── gaze_udp_bridge.py         # UDP gaze -> /gaze/point
    │   │   ├── eeg_udp_bridge.py          # UDP EEG -> /cmd_vel/eeg
    │   │   ├── image_throttle_node.py     # 30 Hz -> 20 Hz
    │   │   └── sensor_sync_node.py        # camera+scan pairing (PC)
    │   ├── launch/
    │   │   ├── m1_robot_bringup.launch.py # hardware
    │   │   ├── m1_control.launch.py       # arbiter + EEG
    │   │   └── m1_operator.launch.py      # gaze HUD + SLAM + RViz (PC)
    │   └── config/
    │       ├── m1_params.yaml
    │       ├── ydlidar_tmini_plus.yaml
    │       └── slam_toolbox_m1.yaml
    ├── scripts/               # env, start, stop, diagnose, validate, test
    ├── integrations/gazemodel/gaze_m1_bridge.py
    └── logs/
```

### Node inventory (11 executables)

| Node | Runs on | Purpose |
|---|---|---|
| `control_arbiter_node` | **Pi** | Mode mux, clamping, 350 ms watchdog |
| `wasd_teleop_node` | Pi (TTY) | Keyboard mecanum + PTZ |
| `eeg_udp_bridge` | **Pi** | UDP :9000 → `/cmd_vel/eeg`, `/eeg/stop` |
| `gaze_udp_bridge` | **Pi** | UDP :9100 → `/gaze/point`, with calibration |
| `gaze_control_node` | **Pi** | Deadzone/dwell → `/cmd_vel/gaze` |
| `image_throttle_node` | **Pi** | 30 Hz → 20 Hz compressed |
| `audio_bridge` | **Pi + PC** | 2-way voice, raw PCM over UDP |
| `m1_console` | **PC** | Operator GUI (PyQt5) |
| `m1_panel` | PC | Small always-on-top mode/battery panel |
| `sensor_sync_node` | PC | `ApproximateTimeSynchronizer` camera+scan |
| `gaze_hud_node` | PC | Local-webcam HUD + overlay |

---

## 6. Topic Contract

### Robot → PC (Wi-Fi)

| Topic | Type | Rate (measured) | QoS |
|---|---|---|---|
| `/image_throttled/compressed` | `CompressedImage` | **20.20 Hz** | BEST_EFFORT |
| `/scan` | `LaserScan` | **10.02 Hz** | BEST_EFFORT |
| `/odom` | `Odometry` | **10.59 Hz** | RELIABLE |
| `/tf` | `TFMessage` | **20.91 Hz** | RELIABLE |
| `/voltage` | `Float32` | 10.0 Hz | BEST_EFFORT |
| `/map` | `OccupancyGrid` | on update | RELIABLE |
| `/audio/status` | `String` | 1 Hz | RELIABLE |

> The image rate is only achievable with the unicast-peer DDS config. With
default multicast over Wi-Fi it measured **2.6 Hz** — see §7.14.

### PC → Robot (actuation, RELIABLE)

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `Twist` | **Arbiter only** — 1 publisher |
| `/Servo` | `ServoControl` | `s1`=**tilt** (larger = up), `s2`=**pan** (larger = left) |
| `/control_mode` | `UInt8` | 0=E-STOP, 1=KEYBOARD, 2=GAZE, 3=EEG, 4=GAZE+EEG |
| `/Buzzer` | `Bool` | **Latches** — see §7.7 |
| `/audio/enable`, `/audio/mic`, `/audio/speaker` | `Bool` | Voice relay control |
| `/gaze/calib` | `Float32MultiArray` | `[offset_x, offset_y, gain_x, gain_y]` |

### Internal (arbiter inputs)

| Topic | Type |
|---|---|
| `/cmd_vel/keyboard`, `/cmd_vel/gaze`, `/cmd_vel/eeg` | `Twist` |
| `/servo/keyboard`, `/servo/gaze` | `Vector3` |
| `/gaze/point` | `Point` (u,v ∈ 0..1) |
| `/gaze/raw` | `Point` (uncorrected, `z`=valid) |
| `/gaze/calib_state` | `Float32MultiArray` |
| `/gaze/zone` | `String` |
| `/eeg/raw_class` | `String` |
| `/eeg/stop` | `Bool` |
| `/gaze/zone` | `String` |

---

## 7. Defects Found and Fixed

Eleven real bugs were caught by testing rather than by reading. Each is recorded
because all five were **silent** — no crash, no error, just wrong behaviour.

### 7.1 The LiDAR "hardware fault" was a race condition

The first driver start failed:

```
[YDLIDAR] Error, cannot retrieve YDLidar health code: ffffffff
[YDLIDAR] Failed to start scan mode: ffffffff
```

A raw serial probe then read **zero bytes at every baudrate from 9600 to
921600**, on both candidate ports, which looks exactly like dead hardware.

It was not. The vendor SDK (`tmini_test`) talked to it perfectly:

```
[YDLIDAR] Lidar running correctly! The health status: good
Model: Tmini Plus   Serial: 2026000800080141
Successed to start scan mode ... Now lidar is scanning
```

The MCU simply was not ready the instant USB enumeration completed. **Fix:**
`respawn=True, respawn_delay=5.0` on the lifecycle node — the retry is now
automatic and invisible.

> Lesson: "no bytes on the wire" is not proof of broken hardware. Cross-check
> with the vendor tool before touching the wiring.

### 7.2 The throttler produced 15 Hz, not 20 Hz

A naive minimum-period limiter quantises to `input / N`. From 30 Hz toward
20 Hz it emits **15 Hz**, because after emitting it must skip the next frame
(33 ms < 50 ms) and then waits again.

A time-credit accumulator fixes it — but the credit cap must be **≥ 2 periods**.
Capping at 1 period discards the remainder and collapses straight back to 15 Hz.
Verified numerically before changing code:

```
current  (cap = 1x period): 15.00 Hz
fixed    (cap = 2x period): 19.90 Hz
```

Measured after the fix: **19.9–20.08 Hz**.

### 7.3 `rmem_default=16MB` is wrong on any machine

The tuning snippets in circulation were written for a **Pi 3B+ with 1 GB**. Two
of their values are harmful here:

| Setting | Effect |
|---|---|
| `rmem_max` / `wmem_max` | A *ceiling*. Costs nothing until requested. **Safe, keep.** |
| `rmem_default` / `wmem_default` | Applies to **every** socket — ssh, apt, VS Code tunnel. Reserves memory across ordinary connections. **Removed.** |

This Pi 5 was measured at **1.4 GB free with 785 MB already swapped**, so
inflating defaults would have made things worse, not better.

`scripts/tune_network.sh` now applies only the safe subset, is idempotent
(verified by running twice — one managed block), and documents the reasoning in
the file itself. `vm.swappiness=10` was added so ROS nodes are not paged out
mid-cycle — at the default 60, a swapped-out control loop can miss the 350 ms
watchdog.

### 7.4 Camera frames were missing from the TF tree

`cam1`/`cam2` are **revolute** joints, so `robot_state_publisher` cannot publish
their transforms without joint states. The camera frames were silently absent —
RViz would have been unable to place the image.

**Fix:** added `joint_state_publisher`. TF tree now complete, `cam1_Link` and
`cam2_Link` at 10.2 Hz.

### 7.5 Angular velocity was being clamped by the linear limit

`eeg_udp_bridge` applied `max_speed` (0.40, a *linear* m/s value) to steering,
silently crushing a configured 0.60 rad/s into 0.40 — the robot turned far more
gently than commanded. **Fix:** separate `--max-turn` (0.80 rad/s), matching the
arbiter's envelope.

Also fixed: an inline `pkill -f "ros2 launch"` inside a `docker exec` matched
**its own shell** and killed the session. Cleanup now lives in
`scripts/kill_all.sh`, which matches by executable name and skips itself and the
ROS daemon.

### 7.6 The servo axes were swapped

I had `s1` = pan and `s2` = tilt, and every comment, parameter name and piece of
gaze maths inherited that assumption. A bench test with single commands and
someone watching the head showed otherwise:

| Command | Observed |
|---|---|
| `{s1: 0, s2: 90}` | head angled **down** |
| `{s1: 30, s2: 90}` | head level |
| `{s1: 70, s2: 90}` | head angled **up** |
| `{s1: 40, s2: 60}` | head turned **right** |
| `{s1: 40, s2: 120}` | head turned **left** |

So **`s1` is tilt** (larger = up) and **`s2` is pan** (larger = left). The gaze
controller would have panned when it meant to tilt — a plausible-looking but
completely wrong behaviour, and one that no log message would ever reveal.

**Fix:** the `s1`/`s2` naming is kept (it matches the message and the hardware),
but the keyboard mapping, the gaze step maths and every comment now say which
axis is which. Verified afterwards:

```
baseline        s1=40  s2=90
look UP         s1=62 (+22)   s2=90  (unchanged)
look LEFT       s1=63 (~0)    s2=110 (+20)
```

> This is the one defect that could not be found by reading code or logs. It
> needed a human watching the robot.

### 7.7 The buzzer latched on and never stopped

The chassis driver treats `/Buzzer` as a **level**, not an edge:

```python
if msg.data:  self.car.set_beep(1)   # ON, and it stays on
else:         self.car.set_beep(0)   # OFF
```

The arbiter published `Bool(true)` on every mode change and never `Bool(false)`.
One mode switch therefore started a beep that would have sounded until the
robot was power-cycled.

**Fix:** `_beep()` now pulses true then false via a one-shot 0.15 s timer, so
the OFF is always sent. `beep_on_mode_change` also defaults to **false** — the
feature is not worth the risk during normal operation — and `shutdown()` sends
false, so an interrupted beep cannot survive.

### 7.8 `q` was both "strafe left" and "quit"

Pressing `q` to strafe exited the teleop node instead. From the operator's side
this looked like the terminal detaching, with no obvious way back.

**Fix:** `q` is strafe-left only. Quit is **Ctrl+C**, and the exit path prints
`restart with: ./m1.sh teleop`.

### 7.9 `stop` reported success but the stack kept running

After a stop, `m1.sh start` was refused with *"a robot stack is already
running"*. Three independent causes:

1. **The supervisor was never killed.** `start_robot.sh` holds the singleton
   lock, and `ros2 launch` respawns its children (the LiDAR runs with
   `respawn=True`), so killing nodes was undone immediately.
2. **Every child inherited the lock fd.** `start_robot.sh` opened fd 9 and
   spawned `ros2 launch` without closing it, so the whole process tree held the
   same flock. Confirmed via `/proc/<pid>/fd` — 14 processes had it open.
3. **The EKF pattern matched the wrong string.** The pattern said
   `ekf_filter_node` (the ROS *node name*) but the binary is `ekf_node`. Four
   orphaned EKF processes had accumulated.

**Fix:** `stop_all.sh` now stops supervisor → launch parents → stray nodes,
with SIGINT escalating to SIGKILL; both launches use `9>&-` so the lock means
exactly one thing; and the EKF pattern is corrected in both cleanup scripts. It
also reports whether the lock is actually free rather than leaving you to find
out on the next start.

> Lesson: a teardown path is not tested until it is run **twice in a row**.
> Three stop/start bugs surfaced in sequence because each one was hidden behind
> the previous one.

### 7.10 A DDS mismatch silently disabled SLAM

This is the most expensive defect in the project, and the only one I got
*backwards* in this report.

The robot ran FastDDS; the operator PC had `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
in its `.bashrc`. On the surface everything worked:

| Check | Result |
|---|---|
| DDS discovery | 39 topics visible from the PC |
| `/scan` | 10.12 Hz |
| `/image_throttled/compressed` | 20.27 Hz |
| `/odom` | 9.97 Hz |
| `/tf` | 20.05 Hz |

Every stream a person would think to check was healthy. But
`slam_toolbox` produced **no map at all**, with no error beyond one startup
`queue is full` line.

**Root cause:** `/tf_static` is published with `TRANSIENT_LOCAL` durability.
That QoS is not reliably delivered between different DDS vendors. So:

- `/tf` (dynamic, `VOLATILE`) — arrived
- `/tf_static` (latched, `TRANSIENT_LOCAL`) — **never arrived**
- therefore `base_link` → `laser` did not exist on the PC
- therefore slam_toolbox could not transform a single scan, and quietly did
  nothing

Confirmed by asking for the transform directly:

```
$ ros2 run tf2_ros tf2_echo base_link laser
Invalid frame ID "base_link" passed to canTransform argument target_frame
```

**Fix:** put both machines on `rmw_cyclonedds_cpp` (both already had it — the
earlier claim in this report that it was missing was simply wrong). After the
change:

```
base_link -> laser          Translation: [0.006, 0.000, 0.149]     OK
odom -> base_footprint      Translation: [0.342, 0.000, 0.000]     OK
/map                        93 occupied, 741 free, 151x41 @ 5 cm   OK
```

> **Lesson:** "all my topics are flowing" is not the same as "the system
> works". A latched topic can be missing while every streaming topic is
> perfect. When a consumer silently does nothing, check the *transforms* and
> the *latched* topics first — and treat a mixed-vendor DDS setup as a
> defect, not a configuration choice.

### 7.11 A gaze node stole the control mode

With a gaze node merely *running*, keyboard driving silently did nothing. The
safety suite dropped from 12/12 to 9/12.

`gaze_hud_node` declared `assert_mode` with a default of `True` and published
`/control_mode = GAZE` from a 1 Hz timer, unconditionally — no gaze data
required. The arbiter log showed it plainly:

```
Mode -> KEYBOARD   (t=...439.192)
Mode -> GAZE       (t=...439.208)   <- 16 ms later
```

Every keyboard selection was undone almost immediately, and the keyboard topic
was correctly ignored because the arbiter really was in GAZE mode.

**Fix:** `assert_mode` defaults to `False` on both gaze nodes, and
`gaze_hud_node` only offers the mode while it is genuinely tracking. The
arbiter additionally logs an ERROR when it sees 6+ mode changes in 5 seconds,
naming the likely cause — but it does **not** block the switch, because that
would break `ros2 topic pub --once /control_mode`, which is the documented way
to change mode.

> **Lesson:** a mode topic is an *authority* channel. A node that asserts it on
> a timer, merely because it is running, will silently disable every other
> mode. Assert only when you are actually producing commands — and prefer a
> loud warning over a silent block.

### 7.12 The operator console kept a private copy of the servo limits

The head turned much further one way than the other. Config, arbiter and teleop
all agreed on symmetric limits, so the config was not the problem.

The operator console had the limits written as literals inside
`nudge_servo()`:

```python
self.servo['s1'] = min(180.0, max(0.0, self.servo['s1'] + d_tilt))
self.servo['s2'] = min(110.0, max(0.0, self.servo['s2'] + d_pan))
```

For pan that is 90 units to the right and only 20 to the left — exactly the
asymmetry being reported. Worse, the console on the operator VM had never been
re-synced after the Pi was fixed, so the two machines were running different
limit sets and neither was wrong on its own terms.

**Fix:** the console declares the same `servo_*` ROS parameters as the arbiter
and reads them at startup. There is now one authoritative definition, and a
copy that drifts is impossible rather than merely discouraged.

> **Lesson:** a limit duplicated in a GUI is a limit that will drift. If two
> processes must agree on a number, make one of them read it from the other —
> do not write it down twice.

### 7.13 A GUI that starts with the wrong environment shows nothing, forever

The console displayed `no data` for battery, mode and video while the robot was
demonstrably publishing all three.

The GUI had been launched through `bash -lc`, which sources `.profile` but not
`.bashrc` — and Ubuntu's default `.bashrc` returns early for non-interactive
shells anyway. `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` were therefore unset in
the GUI's process. With the default domain 0 and the default RMW, the GUI was
on a completely separate DDS network: not an error, not a warning, just silence.

**Fix:** two layers, because one is not enough.

1. A launcher (`~/m1_console.sh`) exports the domain, RMW and Cyclone config
   explicitly before starting the GUI. It also brackets the ROS `setup.bash`
   sourcing with `set +u`, since those scripts read unset variables and abort
   under `set -u`.
2. The GUI compares its own environment against the expected values at startup
   and shows a **red banner** naming both mismatched variables if they differ.

> **Lesson:** a DDS mismatch is a *silent* failure, so it needs an *active*
> check. Anything that can only fail silently deserves an assertion at startup.

### 7.14 Wi-Fi multicast crippled the camera feed

The operator GUI showed `0 fps` and "waiting for camera", while the robot was
publishing compressed frames at a clean 20 Hz.

Small topics were fine. `/control_mode_state` and `/voltage` are ~100 bytes at
1 Hz and arrived perfectly, which made DDS look healthy. But measuring the
actual rate **on the operator side** told a different story:

```
$ ros2 topic hz /image_throttled/compressed     # on the VM
average rate: 2.6
```

One frame is 58 377 bytes. At 20 Hz that is about **9.3 Mbit/s**.

The Pi reaches the operator VM over Wi-Fi. Wi-Fi transmits **multicast** frames
at the lowest basic rate, with no acknowledgement and no retransmission, so a
lost multicast frame is simply lost. A 100-byte topic survives that. A 58 kB
JPEG published 20 times a second does not — roughly 7 of every 8 frames died in
transit, and nothing anywhere reported an error.

**Fix:** switch from multicast to unicast peers in `config/cyclonedds.xml` on
both machines:

```xml
<AllowMulticast>spdp</AllowMulticast>   <!-- discovery only -->
<Discovery>
    <ParticipantIndex>none</ParticipantIndex>
    <Peers><Peer address="10.0.0.102"/></Peers>
</Discovery>
```

Multicast is retained for participant discovery — it is small, and it is what
finds the peer in the first place — while all user data goes over unicast,
which is acknowledged and retransmitted. The interface is also pinned by name,
which stops the host-network container from selecting `docker0` (172.17.0.1)
instead of `wlan0`.

Measured result: **2.6 Hz → 20.2 Hz**, matching the publisher exactly.

> **Lesson:** "the small messages arrive" does not mean the transport is
> healthy. Measure the rate of the *largest* topic on the *receiving* machine.
> Wi-Fi and multicast are a poor combination for anything bandwidth-heavy.

> **Gotcha:** XML comments may not contain `--`. The first version of the
> config used `-------` rules inside the comment block and Cyclone rejected the
> file without any useful diagnostic. Use `=====` instead.

### 7.15 Clicking a button in the GUI killed the whole process

Pressing **Start** in the operator console terminated it with a segmentation
fault. The only clue on the console:

```
QObject::connect: Cannot queue arguments of type 'QTextBlock'
QObject::connect: Cannot queue arguments of type 'QTextCursor'
[ros2run]: Segmentation fault
```

The Start button runs an SSH command, and it did so on a worker thread so the
window would stay responsive. When the command finished, the worker logged the
result — by calling `appendPlainText()` on a `QPlainTextEdit`. Qt widgets may
only be touched from the GUI thread. To deliver a log line across threads Qt has
to queue a `QTextBlock`/`QTextCursor` through the event loop, which requires
those types to be registered with `qRegisterMetaType()`. They are not, so the
call is undefined behaviour — and in practice it is a hard crash, not an
exception. Nothing in the Python code could catch it.

The same file also ran `shell.test()` — an SSH round trip with a 10 s timeout —
directly on the GUI thread every 5 seconds. That did not crash, but it froze the
window for the duration of every failed probe, which is its own kind of broken.

**Fix:** two Qt signals on the window class.

```python
log_signal     = QtCore.pyqtSignal(str)    # any thread -> log widget
link_signal    = QtCore.pyqtSignal(bool)   # probe result -> link label
recheck_signal = QtCore.pyqtSignal()       # worker done -> re-probe
```

Worker threads now only emit; Qt delivers on the GUI thread because the
connection is automatically queued when the emitter and receiver live in
different threads. The link probe moved onto a worker thread as well, guarded by
a busy flag so a slow probe cannot pile up.

An AST check now walks every function used as a thread target and asserts it
contains no widget method calls. It reports `CLEAN` for all three.

> **Lesson:** in Qt, a widget is not thread-safe and the failure mode is a
> segfault, not an exception. If work moves off the GUI thread, the *only*
> permitted way back is a signal. "It usually works" is not evidence — the
> unsafety is in the scheduler, not the code.

> **Lesson:** the same audit applies to blocking I/O in the other direction. An
> SSH call on the GUI thread does not crash, it just makes the application look
> frozen — which users report as "it hung" and developers dismiss as "it was
> busy".

### 7.16 Three small bugs in one new panel

Adding a live SLAM map to the console produced three failures that are worth
recording because each one is a shape that recurs.

**A guard that never fires the first time.** The refresh loop only pushed data
into the map widget when it had *changed*:

```python
if m is not node._map or sc is not node._scan or pose != self._last_pose:
    self.map_view.update_view(...)
```

On the first refresh `m` and `node._map` are both `None`, `pose` and
`self._last_pose` are both `None`, so every clause is false — and the widget was
never initialised. It sat blank while the data was arriving perfectly, which
looks exactly like a broken subscription. Fixed by comparing a signature tuple
against a sentinel that cannot match:

```python
sig = (id(m), id(sc), pose)
if sig != self._map_sig:      # _map_sig starts as None
```

**A timeout with the wrong type.** `tf2`'s `lookup_transform` wants a
`Duration`, not a number:

```python
buffer.lookup_transform('map', 'base_link', Time(), timeout=0.02)
# TypeError: Can't compare duration with object of type: <class 'float'>
```

The call was inside a `try/except`, so the pose silently stayed `None` and the
map drew with no robot marker. The exception was caught correctly — it was just
being caught for the wrong reason.

**A flattened index used as a coordinate.** `QImage.setPixel` takes `(x, y)`:

```python
img.setPixel(i, (h - 1 - j) * w, colour)   # wrong: y is a row index
img.setPixel(i, h - 1 - j, colour)         # right
```

Qt printed `QImage::setPixel: coordinate (206,218) out of range` hundreds of
times and drew a garbled map.

> **Lesson:** a "did it change?" guard must be tested from its initial state.
> The bug is invisible in review because the logic is correct for every call
> except the first one.

> **Lesson:** a broad `except` around a call you have not yet seen succeed hides
> programming errors, not just runtime ones. When the pose stayed `None` the
> code looked like it was waiting for TF, not like it was throwing a
> `TypeError`.

### 7.17 The EKF threw away 91% of the robot's rotation

Rotating the robot by hand did not move `/odom`, so the map never updated. It
looked like SLAM had stopped working.

Measured with `scripts/probe_rotation.py`, turning by hand through 61 degrees
with the wheels off the ground:

| Source | Before fix | After fix |
|---|---|---|
| `/imu/data` (the sensor) | 81.9° | 61.6° |
| `/odom` (what SLAM uses) | **7.7°** | **61.5°** |
| `/odom_raw` (wheel encoders) | 0.0° | 1.6° |

The gyro saw the whole turn. The EKF kept 9% of it.

The vendor `ekf_A1.yaml` fuses yaw **and yaw-rate from both sources**:

```yaml
odom0_config: [T, T, F,  F, F, T,   T, T, F,  F, F, T,  F, F, F]
#                              ^yaw                  ^yaw-rate
imu0_config:  [F, F, F,  F, F, T,   F, F, F,  F, F, T,  F, F, F]
```

With the wheels off the ground the encoders report "not rotating" and hold that
claim with a tight covariance, so the filter mostly believed them. This is the
correct behaviour for a differential-drive robot on the floor — wheel odometry
genuinely is a good rotation source there — and completely wrong the moment the
wheels are not the thing turning the robot.

`slam_toolbox` triggers a scan match on `minimum_travel_heading`, measured
against the **odometry** pose. With `/odom` barely moving it never fired, so the
map only ever changed while driving.

**Fix:** `config/ekf_m1.yaml`, a Layer-4 override that removes yaw and yaw-rate
from `odom0_config`. The IMU becomes the single authority on heading; wheels
still supply x, y, vx and vy, which mecanum encoders measure well.

**This also explains the wall-collision drift.** Driving into a wall makes the
wheels slip. Slipping wheels report rotation that did not happen, and the old
config trusted them — so a collision corrupted the heading and the map sheared
while reversing out. With heading on the gyro, a slipping wheel can no longer
lie about which way the robot is facing.

> **Trade-off:** heading now comes from gyro integration, and this IMU has no
> magnetometer (`use_mag: false`). It drifts a few degrees over a long session.
> For teleoperation mapping that is the better failure: a map that follows the
> robot and drifts slightly beats a map that ignores the robot entirely.

> **Lesson:** a fused sensor configuration encodes an assumption about which
> source is trustworthy. That assumption can be true in the lab and false the
> moment the robot is picked up. When two sensors disagree, check whether the
> filter is *averaging* them — sometimes the right answer is to pick one.

### 7.18 RViz could not draw the robot, and the URDF was innocent

The RobotModel display showed `Status: Error` with `Errors loading geometries...`
while every TF frame underneath reported `Transform OK`. The URDF was fine and
the meshes were installed.

Two separate causes, and the second is §7.13 wearing a different hat.

**Cause 1: the package was not on the operator machine.** The URDF references
`package://yahboomcar_description/meshes/M1USBMecanum/*.STL`, and that package
existed only on the Pi. Copied and built it on the VM.

**Cause 2: the view was launched with a stale environment.** The console started
RViz with `bash -lc 'ros2 launch ...'`. Ubuntu's `~/.bashrc` returns early for
non-interactive shells:

```bash
case $- in *i*) ;; *) return;; esac
```

so a login-but-not-interactive shell never reaches the `source .../setup.bash`
lines further down the file. The child inherits whatever `AMENT_PREFIX_PATH` its
parent had — which is stale if a package was built after the parent started.
RViz then cannot resolve `package://` and reports a *geometry* error, which
sends you looking at the URDF instead of the environment.

Measured: `bash -lc 'echo $AMENT_PREFIX_PATH' | grep -c yahboomcar_description`
returned **0**.

**Fix:** `~/m1_view.sh` on the operator PC, which sources ROS and both
workspaces explicitly, then checks that the meshes are resolvable and **refuses
to start** if they are not. The console now calls that wrapper instead of
composing its own shell command.

```bash
$ ~/m1_view.sh
description: /home/rescue/m1_control_ws/install/yahboomcar_description/share/yahboomcar_description
```

Verified with `resource_retriever.get()` on all eight meshes the URDF
references — the same code path RViz uses — all 2–4 MB each, all resolving.

> **Lesson:** "Errors loading geometries" is a resource-resolution error, not a
> model error. When TF is fine but geometry is not, check
> `AMENT_PREFIX_PATH` before you touch the URDF.

---

## 8. Validation Results

### 8.1 Safety suite — 12/12 pass

```
=== 1. publisher ownership ===
     publisher: control_arbiter_node
  [PASS] /cmd_vel has exactly one publisher  (1 publisher(s))

=== 2. watchdog stop drill ===
  [PASS] keyboard command reaches /cmd_vel  (|v|=0.100)
  [PASS] watchdog halts within 0.35 s  (0.380 s)

=== 3. mode isolation ===
  [PASS] gaze ignored in KEYBOARD mode  (|v|=0.000)

=== 4. velocity clamping ===
  [PASS] vx clamped to <= 0.25  (0.250)
  [PASS] vy clamped to <= 0.20  (0.200)
  [PASS] wz clamped to <= 0.80  (0.800)

=== 5. E-STOP latch ===
  [PASS] E-STOP holds zero despite input  (|v|=0.000)
  [PASS] mode 1 clears the E-STOP latch  (|v|=0.100)

=== 6. EEG UDP end-to-end ===
  [PASS] EEG bridge reachable on UDP :9000
  [PASS] EEG GO drives forward  (vx=0.150)

=== 7. EEG link loss ===
  [PASS] EEG stops when heartbeat stops  (0.88 s)

 12/12 checks passed - all good
```

Reproduce: `./m1.sh` then `scripts/validate_safety.py` (wheels off the ground).

### 8.2 Gaze zone mapping

| Gaze target | Zone | vx | vy | wz |
|---|---|---|---|---|
| centre | CENTER | 0.00 | 0.00 | 0.00 |
| top | FORWARD | **0.20** | 0.00 | 0.00 |
| right | STRAFE_RIGHT | 0.00 | **−0.20** | 0.00 |
| top-right | FORWARD_RIGHT | **0.14** | **−0.14** | 0.00 |
| *silence* | — | **0.00** | 0.00 | 0.00 |

PTZ inside the deadzone, after the axis fix (§7.6) — `s1` = tilt, `s2` = pan:

```
baseline        s1=40  s2=90
look UP         s1=62 (+22)   s2=90  (unchanged)   -> tilts up
look LEFT       s1=63 (~0)    s2=110 (+20)         -> pans left
```

Both axes respond in the correct direction and independently.

### 8.3 SLAM on the operator VM

```
cells=10877 occupied=78 free=994 unknown=9805
map 73x149 @ 0.050m
VERDICT: SLAM producing real map
```

Zero-configuration DDS discovery, Pi → VM, 1.3 ms RTT.

---

## 9. How to Operate

### Daily start

```bash
ssh pi@10.0.0.54
cd /home/pi
./m1.sh start          # full stack, foreground
./m1.sh bg             # or: background, log to logs/robot.log
```

`./m1.sh` also provides: `stop`, `stop --hard`, `diag`, `status`, `shell`,
`teleop`, `test-eeg`, `logs`, `devices`, `build`, `fix-perms`.

### Subcommands worth knowing

```bash
./m1.sh diag                      # full health check, read-only
./m1.sh teleop                    # keyboard drive (WASD+QE, IJKL gimbal)
./m1.sh test-eeg                  # simulate the EEG laptop locally
./m1.sh build                     # rebuild overlay + fix file ownership
```

### Container lifecycle

```bash
./m1_docker.sh status     # container + host/container device state
./m1_docker.sh recreate   # rebuild from image (host mounts survive)
./m1_docker.sh attach     # shell inside
```

### Keyboard teleop

```
 CHASSIS (mecanum)                 GIMBAL
        W                              I   tilt up    (s1 +)
   Q    S    E                         K   tilt down  (s1 -)
   A         D                J  pan left (s2 +)  L  pan right (s2 -)
                                      U   re-centre

   W/S  forward / back
   Q/E  strafe left / right
   A/D  rotate CCW / CW

 SPACE   E-STOP (latched - press 1 to resume)
 1 2 3   KEYBOARD / GAZE / EEG mode
 Ctrl+C  quit          (q is STRAFE, not quit)
```

### Gaze / EEG

```bash
# Gaze (Windows app or VM publisher sends to :9100)
ros2 run m1_control gaze_udp_bridge
ros2 run m1_control gaze_control_node

# EEG (laptop sends to :9000)
ros2 run m1_control eeg_udp_bridge
python3 scripts/test_eeg_udp.py --ip 127.0.0.1 --sequence   # no headset needed
```

### GAZE+EEG fusion (mode 4)

Mode 4 combines the two input channels that are each incomplete on their own:
gaze is continuous but noisy and cannot express "stop", while EEG is discrete
but reliable and can express a deliberate command. Fusion uses each for what it
is good at.

| Input | Effect in FUSION |
|---|---|
| Gaze | steers the robot continuously |
| EEG **STOP** (jaw clench) | halts immediately, outranks gaze |
| EEG turn / strafe | takes over briefly as an override |
| EEG **REST** (zero) | **ignored** — gaze stays in charge |
| Keyboard | wins for 0.4 s in any mode |

The distinction between EEG STOP and EEG REST is the whole safety story. Both
produce a zero `Twist` on `/cmd_vel/eeg`, so the bridge raises `/eeg/stop` only
for a genuine clench. Without that, the robot would stutter every time the
classifier rested.

Validation — 5/5 priority tests:

| Test | Result |
|---|---|
| Gaze alone steers | `vx = 0.20` |
| EEG STOP halts | `vx = 0.00` |
| STOP released resumes | `vx = 0.20` |
| Keyboard overrides | `vx = -0.12` |
| Keyboard released resumes | `vx = 0.20` |

### Operator console (on the PC)

```bash
~/m1_console.sh
```

One window for the whole session: start/stop the robot, pick a mode, drive,
watch the camera, calibrate gaze, and control the voice link. Details in
`operator-pc/README.md`.

> Launch it through the wrapper, never `ros2 run` directly — the wrapper sets
> the DDS environment, and a missing domain does not produce an error, only an
> empty GUI (§7.13).

### Operator map panel

The console draws `/map` itself rather than embedding RViz: librviz has no usable
Python widget bindings in Humble, and a second ROS process is one more thing to
keep alive. The panel shows the occupancy grid, live laser returns transformed
into the map frame, and the robot pose from TF `map`→`base_link`, auto-fitted to
whatever has been mapped so far.

The grid is rasterised into a `QImage` once per `/map` message and then scaled.
Painting individual cells on every repaint would be acceptable for a small map
and hopeless for a large one.

### Two-way voice

Started automatically by the launch file in the `robot` role and muted until
`/audio/enable` arrives.

```bash
# manual, on the PC
ros2 run m1_control audio_bridge --role operator \
    --peer-ip 10.0.0.54 --device-in plughw:2,0 --device-out plughw:2,0

ros2 topic pub --once /audio/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /audio/mic    std_msgs/msg/Bool "{data: false}"   # mute me
ros2 topic echo /audio/status
```

Raw PCM 16 kHz mono, 640-byte chunks every 20 ms, 60 ms jitter buffer. Ports are
derived from `--role`: the robot listens on 9201 and sends to 9200.

`--no-ros` runs the relay with no ROS at all and starts unmuted. That exists
because the operator's physical microphone is on the Windows host, and VMware's
virtual sound card captures digital silence unless the host forwards its mic —
measured as `peak=1` out of 32768 over a 2 s recording, which is an empty stream
rather than a quiet room. Being able to run the operator end where the hardware
actually is removes the passthrough problem entirely.

`/audio/status` reports `enabled`, `mic`, `spk`, `sent`, `recv`, `drop` and
`live`. Note that **`live` means audio arrived** — i.e. the *other* side is
transmitting. On the robot, `live=false` with `sent` climbing is normal until
the operator-side bridge is running.

---

## 10. Servo Convention — Confirmed by Hand

Both the rest pose and the **axis mapping** were verified on the physical robot
by sending single commands and watching the head.

| | Axis | Larger value means | Rest pose |
|---|---|---|---|
| **`s1`** | **TILT** (up/down) | look **up** | **40** |
| **`s2`** | **PAN** (left/right) | look **left** | **90** |

Observed: `s1` 0 = angled down, 30 = level, 70 = angled up. `s2` 60 = right,
90 = centre, 120 = left.

The driver maps `s1`→PWM channel 2 and `s2`→PWM channel 3, and hard-clamps `s2`
to 0..110.

### Travel limits — measured with the camera as the sensor

The configured ranges below were **measured**, not copied from the vendor
joystick defaults. The method matters because it needs no extra hardware: the
camera is mounted on the head, so any head movement shows up as image motion.

The first attempt used phase correlation between the frame at the start and the
frame at each position. That failed — the head moves too far per step and phase
correlation wraps around, producing nonsense. The working method stepped the
servo by 4–6 units and used **ORB feature matching with a median consensus**
between *consecutive* frames, then integrated the per-step shifts. Each step is
small enough for feature matching to stay locked, and the consensus filter
discards mismatched features.

Results:

| `s2` (pan) | Behaviour |
|---|---|
| 0 → ~112 | moves freely, linear (~48 px per 6 units) |
| 112 → 120 | **no image change at all** with ~980 matched features |

The flat tail is the proof: the frame does not move, so the servo is not
moving. That is the firmware clamp, and it sits at 110.

This is the origin of the asymmetry an operator notices as "it turns further
right than left": with the rest pose at 90, the head has **90 units of travel
to the right but only 20 to the left**. The asymmetry was never a configuration
bug — it is the geometry of a clamp at 110 combined with a centre of 90.

| | Measured usable | Configured range | Centre |
|---|---|---|---|
| `s1` (tilt) | 0 .. 90 | **5 .. 75** (±35) | 40 |
| `s2` (pan) | 0 .. 110 | **80 .. 100** (±10) | 90 |

Pan is deliberately narrowed well inside the mechanical limit. A camera that
swings ~90° loses the view straight ahead, and driving forward while blind is
the dangerous failure. ±10° keeps the path in frame. Tilt keeps a wide range
because looking down at the floor in front of the robot is genuinely useful
when navigating.

If a limit needs changing, set it in `config/m1_params.yaml`
(`servo_s1_min/max`, `servo_s2_min/max`). The same values are ROS parameters on
every node that touches the servos, including the operator console, so there is
no second copy to drift out of sync — see §7.12 for what happens when there is.

Move the servos slowly the first time and stop before they buzz or stall.

### If a direction feels inverted

```bash
ros2 run m1_control gaze_control_node --ros-args \
    -p invert_pan:=true -p invert_tilt:=true
```

Then make it permanent in `config/m1_params.yaml`.

---

## 11. Known Limitations

1. **Both machines must use CycloneDDS.** This is a requirement, not a
   preference — see §7.10. A mismatch does not error; it silently breaks
   `/tf_static` and therefore SLAM.

2. **`sensor_sync_node` produces a schematic overlay**, not a calibrated
   projection — this camera publishes no `CameraInfo`, so a true 3D projection
   is impossible without calibration. The overlay exists to make a bad
   time-pairing *visible*; it is not a measurement tool.

3. **Memory pressure comes from dev tooling, not ROS.** VS Code Server
   (~3.4 GB) and Chromium (~1.4 GB) are the top consumers. ROS nodes are small.
   If control latency ever spikes, close the VS Code session before blaming the
   robot.

4. **The `gaze_hud_node` on the PC is untested** — it needs the VM's webcam and
   display, which I cannot verify from here. `gaze_control_node` (the part that
   actually moves the robot) is fully tested.

5. **`/map` only updates while the robot moves.** A stationary robot produces a
   small map (78 occupied cells). This is correct behaviour, not a fault.

---

## 12. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Node not found` for ydlidar | Lifecycle node not activated / crashed | Check log; `respawn` retries in 5 s |
| `/scan` missing | LiDAR unplugged at launch | Reconnect, restart with `lidar:=true` |
| Robot drives erratically | Rogue `/cmd_vel` publisher | `ros2 topic info /cmd_vel`; `./m1.sh stop --hard` |
| Video freezes after seconds | UDP buffers too small | `sudo scripts/tune_network.sh apply` |
| Camera `Select timeout` | 120 Hz MJPG forced to 30 fps | Use `pixel_format:=yuyv2rgb` (already set) |
| Nodes exit immediately | `RMW_IMPLEMENTATION` unset while the PC uses CycloneDDS | Export `rmw_cyclonedds_cpp` on **both** machines |
| `/tf` arrives but SLAM makes no map | DDS mismatch breaks `/tf_static` | Same — see §7.10 |
| VM sees no topics | Domain ID mismatch / not bridged | `echo $ROS_DOMAIN_ID` = 30 both sides |
| Servo not moving | `yahboomcar_msgs` not sourced | Source Layer 3 before Layer 4 |
| `queue is full` spam in SLAM | TF timeout too short for Wi-Fi | `transform_timeout: 0.5` (already set) |

---

## 13. Files Changed on the Host

| Path | Change |
|---|---|
| `/etc/sysctl.conf` | Managed block via `tune_network.sh` (backups kept) |
| `/etc/sysctl.conf.bak.*` | Automatic backups before each apply |
| `/home/pi/.bashrc` | Unchanged — already had `ROS_DOMAIN_ID=30` |
| Container `/root/.bashrc` | `ROS_DOMAIN_ID` 61 → **30** |
| Container `/root/run_handle.sh` | `ROS_DOMAIN_ID` 61 → **30** |
| Container itself | **Recreated** with `--init` and `-v /dev:/dev` |

> `run_handle.sh` previously auto-started `yahboomcar_joy_launch.py`, which
> publishes `/cmd_vel` — a second publisher competing with the arbiter. The
> recreated container starts **idle**; the stack is started explicitly with
> `./m1.sh start`. This is deliberate: a container restart must never silently
> re-arm a controller.

---

## 14. Next Steps

1. ~~Confirm the servo axis convention~~ — **done**, see §10. Travel limits
   still unconfirmed; check them slowly on the bench.
2. **Run `operator-pc/SETUP.md` on the VM** to finish the operator side.
3. **Floor trial**: elevate → verify → lower, per handbook §11.
4. **Calibrate the gaze model** (GazeModel, ~3 min, 4 postures).
5. Optional: install CycloneDDS if throughput ever becomes a bottleneck.
