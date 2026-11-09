# SIIT-bobo-goggles

Multi-modal teleoperation for the **Yahboom ROSMASTER M1** (Raspberry Pi 5,
ROS 2 Humble): keyboard mecanum + PTZ, eye-gaze control, and EEG headset control,
with live 2D LiDAR SLAM.

```
                 ┌──────────────────────────────┐
   Muse 2  ─BLE─►│ Operator PC (VMware Ubuntu)  │
   (EEG)         │  • gaze model / HUD          │
                 │  • slam_toolbox              │
   Webcam ──────►│  • RViz                      │
                 └──────────┬───────────────────┘
                            │ Wi-Fi, ROS_DOMAIN_ID=30
                            │
                 ┌──────────▼───────────────────┐
                 │ Raspberry Pi 5 (Docker)      │
                 │  • chassis + LiDAR + camera  │
                 │  • control arbiter (SAFETY)  │
                 │  • gaze/EEG UDP bridges      │
                 └──────────────────────────────┘
```

## Repository layout

| Folder | Contents |
|---|---|
| [`robot/`](robot/) | **Robot side** — ROS 2 overlay, host scripts, engineering report |
| [`operator-pc/`](operator-pc/) | **Operator side** — VM setup, operator console, voice, gaze calibration |
| [`tools/`](tools/) | Development helpers (vision bridge for AI-assisted debugging) |

## Start here

**New to the project? Read [`docs/WALKTHROUGH.md`](docs/WALKTHROUGH.md) first.**
It explains the whole system from first principles, what is finished, and where
to begin — no prior ROS 2 knowledge assumed.

| I want to… | Read |
|---|---|
| **Understand the big picture** | [`docs/WALKTHROUGH.md`](docs/WALKTHROUGH.md) |
| **Architecture, in one page** | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| Set the robot up from scratch | [`robot/SETUP.md`](robot/SETUP.md) |
| Set up the operator PC | [`operator-pc/SETUP.md`](operator-pc/SETUP.md) |
| Understand what was built and why | [`robot/docs/report.md`](robot/docs/report.md) |
| **Map a space, then localise** | [`docs/MAPPING.md`](docs/MAPPING.md) |
| **Set up gaze / eye control** | [`docs/GAZE.md`](docs/GAZE.md) |
| **Start / stop / close properly** | [`docs/START_STOP.md`](docs/START_STOP.md) |
| **Connect a webcam or mic to the VM** | [`docs/DEVICE_PASSTHROUGH.md`](docs/DEVICE_PASSTHROUGH.md) |

## Design principles

**One publisher per actuator.** Only `control_arbiter_node` may publish
`/cmd_vel` and `/Servo`. Keyboard and gaze each publish to their own topic
(`/cmd_vel/keyboard`, `/cmd_vel/gaze`) and the arbiter forwards exactly one,
based on the active mode. Two controllers can never fight.

The EEG is **not** a velocity source. It publishes *state* (`/eeg/run`,
`/eeg/mode`, `/eeg/link`) and `gaze_control_node` gates the gaze command on it.
A headset gesture therefore decides *whether* and *what* gaze steers, never how
fast - a misclassified gesture can start or stop the robot but cannot make it
lurch at an unexpected speed.

**Safety lives on the robot.** The arbiter and its 350 ms dead-man watchdog run
on the Pi, not the PC — a watchdog across a Wi-Fi link cannot stop a robot whose
Wi-Fi link just died.

**Vendor code is never modified.** Everything custom lives in a Layer 4 overlay
(`m1_control_ws`), sourced after the Yahboom workspaces.

## Control modes

| Mode | Command source | Topic |
|---|---|---|
| `0` E-STOP | - | latched, holds zero |
| `1` Keyboard | WASD + QE, IJKL gimbal | `/cmd_vel/keyboard` |
| `2` Gaze | deadzone to PTZ, dwell to drive | `/cmd_vel/gaze` |
| `3` EEG | headset gates gaze | `/cmd_vel/gaze` |
| `4` Gaze + EEG | clench arms, gaze steers | `/cmd_vel/gaze` |

**Gaze only reaches the wheels in modes 2, 3 and 4.** In mode 1 the arbiter
discards it, so the gaze HUD will happily draw a zone while the robot ignores
it - the console now warns when that is the case.

```bash
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 4}"
```

Safety envelope: `vx <= 0.40 m/s`, `vy <= 0.20 m/s`, `wz <= 0.80 rad/s`,
watchdog `0.35 s`. The console asks for 0.35 m/s forward; the cap sits above
it so float rounding can never clip the request.

**Keyboard fallback for every EEG gesture.** The classifier on this headset
confuses a jaw clench with an eyebrow raise often enough that it reports
`UNCLEAR_MUSCLE` and refuses to guess. Each action therefore has a key, and
the gesture pad in the console is clickable:

| Key | Gesture | Does |
|---|---|---|
| `B` | double blink | switch what gaze steers |
| `N` | jaw clench | go / stop |
| `M` | eyebrow raise | play a sound |

## Status

| Area | State |
|---|---|
| Chassis, PTZ, camera, LiDAR | Working (validated on hardware) |
| Servo axis mapping | Working — `s1`=tilt, `s2`=pan, confirmed by hand |
| Arbiter + watchdog | Working — 12/12 safety checks pass |
| Gaze control | Working (zone mapping + PTZ verified) |
| Gaze calibration | Working - 10-feature ridge model, 4-posture profile |
| EEG control | Working (protocol v1: toggles + state, verified end-to-end) |
| EEG keyboard fallback | Working - B / N / M, plus clickable gesture pad |
| SLAM on operator PC | Working |
| start / stop / restart | Working — verified over two consecutive cycles |
| Servo travel limits | Working — measured with the camera as a sensor |
| Gaze + EEG fusion (mode 4) | Working — 5/5 priority tests |
| Two-way voice | Working — per-direction mute, raw PCM over UDP |
| Operator console (PyQt5) | Working - 6 panels, live SLAM map, in-window teleop |
| Sound effects (eyebrow raise) | Working - random file from `/home/pi/sound` |
| Gaze calibration | Working — 8/8 checks, 5-point offset + gain |

Twenty defects were found and fixed during hardware validation. Most of them
were **silent** — no error, no warning, just a feature that did not work:
a duplicated limit in the GUI, a DDS environment that never got set, and
Wi-Fi multicast quietly dropping 7 of every 8 video frames. All are written up
in [`robot/docs/report.md`](robot/docs/report.md) §7.
