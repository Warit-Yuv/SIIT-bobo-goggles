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
| Cross-machine DDS | **Working** | Zero-config discovery, Pi↔VM, 1.3 ms |
| start / stop / restart | **Working** | Two consecutive cycles, 0 leftovers |

**Nine defects were found and fixed during validation.** They are documented in
§7 because each one would have been a silent failure in operation.

---

## 2. What Was Actually Found vs. the Handbook

The handbook is accurate in architecture but several hardware facts differ from
what the real machine reports. These differences matter:

| Handbook says | Reality on this robot |
|---|---|
| YDLIDAR **Tmini** | **Tmini Plus**, FW 1.2, HW 1, S/N `2026000800080141` |
| LiDAR at `/dev/ydlidar` | **`/dev/rplidar`** (udev rule maps `10c4:ea60`) |
| Servo centre `s1=90, s2=35` | **`s1=40, s2=90`** — and **`s1` is tilt, `s2` is pan** (verified by hand, §10) |
| CycloneDDS recommended | **Not installed.** FastDDS only — exporting `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` breaks every node |
| Arbiter runs on operator PC | **Moved to the Pi** (see §3.1) |
| `topic_tools throttle` | **Not installed** — replaced with our own node |
| `laser_link` TF frame | URDF publishes **`laser_Link`**; driver publishes **`laser`** |
| Camera frame `camera_link` | URDF has **`cam1_Link` → `cam2_Link`** (revolute joints) |

### Why this matters

The handbook's `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` line, copied verbatim,
would make **every node fail to start**. Verified:

```
$ ls /opt/ros/humble/lib/ | grep ^librmw_
librmw_fastrtps_cpp.so        <- the only DDS implementation present
librmw_implementation.so
```

`scripts/env.sh` therefore explicitly *unsets* it, with a comment explaining why.

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

### Node inventory (8 executables)

| Node | Runs on | Purpose |
|---|---|---|
| `control_arbiter_node` | **Pi** | Mode mux, clamping, 350 ms watchdog |
| `wasd_teleop_node` | Pi (TTY) | Keyboard mecanum + PTZ |
| `eeg_udp_bridge` | **Pi** | UDP :9000 → `/cmd_vel/eeg` |
| `gaze_udp_bridge` | **Pi** | UDP :9100 → `/gaze/point` |
| `gaze_control_node` | **Pi** | Deadzone/dwell → `/cmd_vel/gaze` |
| `image_throttle_node` | **Pi** | 30 Hz → 20 Hz compressed |
| `sensor_sync_node` | PC | `ApproximateTimeSynchronizer` camera+scan |
| `gaze_hud_node` | PC | Local-webcam HUD + overlay |

---

## 6. Topic Contract

### Robot → PC (Wi-Fi)

| Topic | Type | Rate (measured) | QoS |
|---|---|---|---|
| `/image_throttled/compressed` | `CompressedImage` | **20.08 Hz** | BEST_EFFORT |
| `/scan` | `LaserScan` | **10.02 Hz** | BEST_EFFORT |
| `/odom` | `Odometry` | **10.59 Hz** | RELIABLE |
| `/tf` | `TFMessage` | **20.91 Hz** | RELIABLE |
| `/voltage` | `Float32` | 10.0 Hz | BEST_EFFORT |
| `/map` | `OccupancyGrid` | on update | RELIABLE |

### PC → Robot (actuation, RELIABLE)

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `Twist` | **Arbiter only** — 1 publisher |
| `/Servo` | `ServoControl` | `s1`=**tilt** (larger = up), `s2`=**pan** (larger = left) |
| `/control_mode` | `UInt8` | 0=E-STOP, 1=KEYBOARD, 2=GAZE, 3=EEG |
| `/Buzzer` | `Bool` | **Latches** — see §7.7 |

### Internal (arbiter inputs)

| Topic | Type |
|---|---|
| `/cmd_vel/keyboard`, `/cmd_vel/gaze`, `/cmd_vel/eeg` | `Twist` |
| `/servo/keyboard`, `/servo/gaze` | `Vector3` |
| `/gaze/point` | `Point` (u,v ∈ 0..1) |
| `/eeg/raw_class` | `String` |
| `/gaze/zone` | `String` |

---

## 7. Defects Found and Fixed

Nine real bugs were caught by testing rather than by reading. Each is recorded
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

### Travel limits are NOT yet confirmed

Only the *directions* above were observed. The configured ranges are the vendor
joystick controller's values and have not been checked against the mechanical
stops:

| | Configured range | Source |
|---|---|---|
| `s1` | 0 .. 180 | vendor joystick |
| `s2` | 0 .. 110 | chassis driver hard clamp |

Move the servos slowly the first time and stop before they buzz or stall. If a
limit is wrong, set it in `config/m1_params.yaml` (`servo_s1_min/max`,
`servo_s2_min/max`) — no code change needed.

### If a direction feels inverted

```bash
ros2 run m1_control gaze_control_node --ros-args \
    -p invert_pan:=true -p invert_tilt:=true
```

Then make it permanent in `config/m1_params.yaml`.

---

## 11. Known Limitations

1. **CycloneDDS is not available.** FastDDS is used. Measured throughput is
   already sufficient (20 Hz video, 10 Hz scan, no buffer overruns), so
   installing CycloneDDS is optional, not required.

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
| Nodes exit immediately | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | `unset RMW_IMPLEMENTATION` |
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
