# Operator PC side — VMware Ubuntu 22.04

The operator workstation: visualisation, SLAM, and the gaze/EEG interfaces.

**→ Full setup procedure: [`SETUP.md`](SETUP.md)**

---

## Contents

| Path | Status | Purpose |
|---|---|---|
| `SETUP.md` | ✅ | Complete VM setup guide (verified against the live VM) |
| `teleop/` | 🚧 **in progress** | VM-side teleoperation — to be added |

---

## What runs here

| Component | Purpose |
|---|---|
| `slam_toolbox` | 2D mapping from `/scan` + `/tf` |
| RViz | Laser scan, camera, map, TF tree |
| Gaze model / HUD | Webcam gaze estimation |
| `sensor_sync_node` | Camera + LiDAR time pairing |
| EEG pipeline (`muse_robot`) | Runs on the Windows host — needs Bluetooth to the Muse 2 |

## What must **not** run here

| Component | Why |
|---|---|
| `control_arbiter_node` | The safety watchdog must be downstream of the Wi-Fi link |
| Any `/cmd_vel` publisher | Only the arbiter on the robot may publish it |

---

## Measured environment

| Item | Value |
|---|---|
| OS | Ubuntu 22.04.5 LTS, kernel 6.8 |
| ROS 2 | Humble (`/opt/ros/humble`) |
| CPU / RAM | 16 cores / 19 GB |
| Network | `ens33` = `10.0.0.102/24`, **bridged** |
| Latency to robot | **1.3 ms** |
| Domain ID | `30` (must match the robot) |

Installed: `slam_toolbox`, `rviz2`, `nav2_bringup`, `tf2_ros`,
`message_filters`, `image_transport`, `cv_bridge`, `cv2` 4.11, `numpy` 1.26.

Not installed (and not needed here): `robot_localization`, `usb_cam`,
`rmw_cyclonedds_cpp`.

---

## Verified stream rates (robot → VM, over Wi-Fi)

```
/image_throttled/compressed   20.08 Hz
/scan                         10.02 Hz
/odom                         10.59 Hz
/tf                           20.91 Hz
```

---

## Quick start

```bash
# 1. Environment (once)
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'unset RMW_IMPLEMENTATION' >> ~/.bashrc
source ~/.bashrc

# 2. Copy the workspace from the robot
scp -r pi@10.0.0.54:/home/pi/m1_control_ws ~/m1_control_ws
cd ~/m1_control_ws && colcon build --symlink-install --packages-select m1_control
source install/setup.bash

# 3. Check the robot is visible
ros2 topic list | grep -E 'scan|odom|image_throttled'

# 4. Operator stack
ros2 launch m1_control m1_operator.launch.py
```

---

## Gotchas

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` empty | Domain mismatch / NAT mode | `ROS_DOMAIN_ID=30`, adapter → **Bridged** |
| Nodes exit instantly | `RMW_IMPLEMENTATION` set | `unset RMW_IMPLEMENTATION` |
| Image display black | QoS mismatch | Set display reliability to **Best Effort** |
| SLAM `queue is full` spam | TF timeout too short | Use `config/slam_toolbox_m1.yaml` |
| Gaze pointer far off | Stale calibration | Press **F** (quick fix) or **C** (recalibrate) |

> `/scan` and the image are published **Best Effort**. A *Reliable* subscriber
> connects but receives nothing — the most confusing ROS 2 failure mode there is.
