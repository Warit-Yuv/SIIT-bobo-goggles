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
| [`operator-pc/`](operator-pc/) | **Operator side** — VM setup guide, VM teleop (in progress) |

## Start here

**New to the project? Read [`docs/WALKTHROUGH.md`](docs/WALKTHROUGH.md) first.**
It explains the whole system from first principles, what is finished, and where
to begin — no prior ROS 2 knowledge assumed.

| I want to… | Read |
|---|---|
| **Understand the big picture** | [`docs/WALKTHROUGH.md`](docs/WALKTHROUGH.md) |
| Set the robot up from scratch | [`robot/SETUP.md`](robot/SETUP.md) |
| Set up the operator PC | [`operator-pc/SETUP.md`](operator-pc/SETUP.md) |
| Understand what was built and why | [`robot/docs/report.md`](robot/docs/report.md) |

## Design principles

**One publisher per actuator.** Only `control_arbiter_node` may publish
`/cmd_vel` and `/Servo`. Keyboard, gaze and EEG each publish to their own
topic (`/cmd_vel/keyboard`, `/cmd_vel/gaze`, `/cmd_vel/eeg`) and the arbiter
forwards exactly one, based on the active mode. Two controllers can never fight.

**Safety lives on the robot.** The arbiter and its 350 ms dead-man watchdog run
on the Pi, not the PC — a watchdog across a Wi-Fi link cannot stop a robot whose
Wi-Fi link just died.

**Vendor code is never modified.** Everything custom lives in a Layer 4 overlay
(`m1_control_ws`), sourced after the Yahboom workspaces.

## Control modes

| Mode | Command source | Topic |
|---|---|---|
| `0` E-STOP | — | latched, holds zero |
| `1` Keyboard | WASD + QE, IJKL gimbal | `/cmd_vel/keyboard` |
| `2` Gaze | deadzone → PTZ, dwell → drive | `/cmd_vel/gaze` |
| `3` EEG | Muse 2 gestures | `/cmd_vel/eeg` |

```bash
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 1}"
```

Safety envelope: `vx ≤ 0.25 m/s`, `vy ≤ 0.20 m/s`, `ωz ≤ 0.80 rad/s`,
watchdog `0.35 s`.

## Status

| Area | State |
|---|---|
| Chassis, PTZ, camera, LiDAR | Working (validated on hardware) |
| Servo axis mapping | Working — `s1`=tilt, `s2`=pan, confirmed by hand |
| Arbiter + watchdog | Working — 12/12 safety checks pass |
| Gaze control | Working (zone mapping + PTZ verified) |
| EEG control | Working (UDP end-to-end verified) |
| SLAM on operator PC | Working |
| start / stop / restart | Working — verified over two consecutive cycles |
| Servo travel limits | **Not confirmed** against mechanical stops |
| VM-side teleop | **In progress** |
