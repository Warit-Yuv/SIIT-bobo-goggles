# Robot side — Raspberry Pi 5

Everything that runs on the ROSMASTER M1. Validated against real hardware.

**→ Full procedure: [`SETUP.md`](SETUP.md)**
**→ Why things are built this way: [`docs/report.md`](docs/report.md)**

---

## Contents

```
robot/
├── SETUP.md                     # step-by-step from bare Pi to driving
├── docs/report.md               # engineering report (findings, defects, evidence)
├── host/
│   ├── m1.sh                    # control panel: start/stop/diag/teleop/build
│   └── m1_docker.sh             # create/attach/inspect the container
└── m1_control_ws/
    ├── src/m1_control/
    │   ├── m1_control/          # the 8 nodes
    │   ├── launch/              # bringup, control, operator launches
    │   └── config/              # params, LiDAR, slam_toolbox
    ├── scripts/                 # env, start, stop, diagnose, validate, test
    └── integrations/gazemodel/  # gaze-model → robot bridge module
```

---

## Nodes

| Node | Purpose |
|---|---|
| `control_arbiter_node` | **Safety core.** Mode mux, velocity clamping, 350 ms watchdog. The only publisher of `/cmd_vel` and `/Servo`. |
| `wasd_teleop_node` | Keyboard mecanum (`WASD`+`QE`) and gimbal (`IJKL`) |
| `eeg_udp_bridge` | UDP `:9000` → `/cmd_vel/eeg`, majority filter + link watchdog |
| `gaze_udp_bridge` | UDP `:9100` → `/gaze/point` |
| `gaze_control_node` | Deadzone → PTZ, dwell → drive; the motion *policy* |
| `image_throttle_node` | 30 Hz → 20 Hz compressed frames |
| `sensor_sync_node` | `ApproximateTimeSynchronizer` for camera + scan *(PC)* |
| `gaze_hud_node` | Local-webcam HUD overlay *(PC)* |

## Launch files

| File | Runs on | Purpose |
|---|---|---|
| `m1_robot_bringup.launch.py` | Pi | Driver, odometry, EKF, camera, LiDAR |
| `m1_control.launch.py` | Pi | Arbiter + EEG bridge |
| `m1_operator.launch.py` | PC | Gaze HUD + SLAM + RViz |

```bash
# On the robot
./m1.sh start

# Individual launches, inside the container
ros2 launch m1_control m1_robot_bringup.launch.py lidar:=auto camera:=true
ros2 launch m1_control m1_control.launch.py eeg:=true
```

---

## Key topics

| Topic | Type | Rate | QoS |
|---|---|---|---|
| `/image_throttled/compressed` | `CompressedImage` | 20.0 Hz | BEST_EFFORT |
| `/scan` | `LaserScan` | 10.0 Hz | BEST_EFFORT |
| `/odom` | `Odometry` | 10.6 Hz | RELIABLE |
| `/tf` | `TFMessage` | 20.9 Hz | RELIABLE |
| `/voltage` | `Float32` | 10 Hz | BEST_EFFORT |
| `/cmd_vel` | `Twist` | — | RELIABLE |
| `/Servo` | `ServoControl` | — | RELIABLE |
| `/control_mode` | `UInt8` | — | RELIABLE, latched |

*(Rates measured on hardware, received over Wi-Fi.)*

---

## Scripts

| Script | Purpose |
|---|---|
| `env.sh` | Canonical environment (source this, never hand-type the chain) |
| `start_robot.sh` | Bringup + control in one process group |
| `stop_all.sh` | Stop nodes, zero velocity first |
| `kill_all.sh` | Hard reset of the ROS graph |
| `diagnose.sh` | Read-only health check (exit code = failures) |
| `validate_safety.py` | 12 automated safety assertions |
| `test_eeg_udp.py` | Simulate the EEG laptop, no headset needed |
| `test_gaze.py` | Drive the gaze pipeline with synthetic points |
| `lidar_probe.py` | Raw YDLidar protocol probe |
| `tune_network.sh` | Idempotent sysctl tuning (`apply` / `--check` / `--revert`) |

---

## Hardware notes

These differ from common documentation — verified on this unit:

| Item | Value |
|---|---|
| LiDAR | **Tmini Plus**, FW 1.2, S/N `2026000800080141`, 230400 baud |
| LiDAR device | `/dev/rplidar` (not `/dev/ydlidar`) |
| Servo rest pose | **`s1=40, s2=90`** (not `90/35`) |
| Camera format | `yuyv2rgb` @ 30 fps (MJPG is locked to 120 Hz at 640×480) |
| DDS | **CycloneDDS on both machines** — a mismatch silently breaks `/tf_static` and SLAM |

> Do **not** export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. It is not
> installed, and setting it makes every node fail to start.

---

## Quick checks

```bash
./m1.sh diag                    # 0 failed?
./m1.sh status                  # nodes + topics
./m1.sh devices                 # device + container state

# Safety suite (wheels off the ground)
docker exec -it ros-humble-dev bash -lc \
  'source /root/m1_control_ws/scripts/env.sh >/dev/null 2>&1; \
   python3 /root/m1_control_ws/scripts/validate_safety.py'
```
