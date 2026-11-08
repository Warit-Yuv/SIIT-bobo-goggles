# Operator PC side — VMware Ubuntu 22.04

The operator workstation: visualisation, SLAM, and the gaze/EEG interfaces.

**→ Full setup procedure: [`SETUP.md`](SETUP.md)**

---

## Contents

| Path | Status | Purpose |
|---|---|---|
| `SETUP.md` | ✅ | Complete VM setup guide (verified against the live VM) |
| `teleop/` | ✅ | VM-side teleop (`m1_console`) + safety contract |

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
`message_filters`, `image_transport`, `cv_bridge`, `cv2` 4.11, `numpy` 1.26,
**`rmw_cyclonedds_cpp`**, `python3-pyqt5` 5.15.

Not installed (and not needed here): `robot_localization`, `usb_cam`, `ffmpeg`.
The voice relay sends raw PCM over UDP precisely so that `ffmpeg` is not a
dependency.

---

## Verified stream rates (robot → VM, over Wi-Fi)

```
/image_throttled/compressed   20.20 Hz
/scan                         10.02 Hz
/odom                         10.59 Hz
/tf                           20.91 Hz
```

> These numbers require the unicast-peer DDS configuration. With default
> multicast over Wi-Fi the image arrived at **2.6 Hz** while every other topic
> looked healthy. See `robot/docs/report.md` §7.14.

---

## Operator console

The console puts everything on one screen so an operator does not have to
juggle four terminals while something is going wrong.

```bash
~/m1_console.sh
```

**Always launch it through the wrapper, not `ros2 run` directly.** The wrapper
exports `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` before
starting. Launching straight from a non-interactive shell silently leaves those
unset, and the GUI then shows `no data` forever without any error — see
`robot/docs/report.md` §7.13. The console also checks its own environment at
startup and shows a red banner naming the mismatch if it is wrong.

| Panel | What it does |
|---|---|
| **1 · Robot** | Start / stop / diagnose the stack on the Pi over SSH |
| **2 · Control mode** | Keyboard, Gaze, EEG, Gaze+EEG, and a large E-STOP |
| **3 · Camera head** | Pan and tilt with the allowed range drawn on the bar |
| **4 · Gaze** | Live gaze plot (raw vs corrected) and 5-point calibration |
| **5 · Voice** | Enable audio, mute either direction, packet meters |
| **Camera** | Live feed at the throttled rate |
| **Drive** | Keyboard teleop — the window captures keys, no terminal needed |
| **Log** | Timestamped actions and remote command output |

### Keyboard

| Key | Action |
|---|---|
| `W` / `S` | forward / back |
| `Q` / `E` | strafe left / right |
| `A` / `D` | rotate left / right |
| `I` / `K` | tilt up / down |
| `J` / `L` | pan left / right |
| `U` | centre the head |
| `1`–`4` | select control mode |
| `0` or `Space` | E-STOP |

Keys work in **any** mode. The arbiter lets a recent keyboard command win for
0.4 s, so you can nudge the robot without switching modes first, and control
returns to gaze/EEG automatically when you stop pressing.

---

## Two-way voice

Speaker and microphone are one C-Media USB audio device (card 2,
`plughw:2,0`). The robot runs `audio_bridge` in the `robot` role; the operator
runs it in the `operator` role.

| Direction | Port |
|---|---|
| robot → operator | UDP 9200 |
| operator → robot | UDP 9201 |

Audio is raw PCM, 16 kHz mono, 640-byte chunks every 20 ms, with a 60 ms jitter
buffer. Raw PCM means no encoder dependency on either side.

**It is muted until `/audio/enable` says otherwise**, so the microphone in
someone's home is never live by accident. The console's Voice panel drives it,
and either direction can be muted independently.

The vendor `largemodel` package also ships audio (ASR + TTS + a hardware
wake-word microphone on `/dev/ttyUSB2`). That is a voice-*assistant* pipeline,
not an intercom, so it is not used here. Note that its `mic_serial_port`
defaults to `/dev/ttyUSB1`, which on this robot is the **LiDAR** — change it to
`/dev/ttyUSB2` before running it.

---

## Gaze calibration

Every eye tracker has a resting bias: the operator looks straight ahead and the
model reports `u=0.62` instead of `0.5`. Uncorrected, neutral steering pulls the
robot to one side for the whole session.

The console runs a guided **five-point** calibration (centre, left, right, up,
down) and fits

```
corrected = (raw - offset) * gain + 0.5
```

Five points recover the gain as well as the offset, so a small comfortable eye
movement still covers the full steering range. The corrected point is clamped to
`[0, 1]`, and a bad fit cannot produce an unbounded gain.

Press **Reset** to return to neutral (offset 0.5, gain 1.0, i.e. passthrough).

---

## Quick start

```bash
# 1. Environment (once). These MUST match the robot.
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 2. Cyclone config with a unicast peer pointing at the robot.
#    Multicast over Wi-Fi loses most of the video - see report.md §7.14.
#    Copy the robot's config and change the interface and peer:
#      <NetworkInterface name="ens33"/>   <Peer address="10.0.0.54"/>
ls ~/cyclonedds.xml

# 3. Copy the workspace from the robot
scp -r pi@10.0.0.54:/home/pi/m1_control_ws ~/m1_control_ws
cd ~/m1_control_ws && colcon build --symlink-install --packages-select m1_control
source install/setup.bash

# 4. Check the robot is visible
ros2 topic list | grep -E 'scan|odom|image_throttled'
ros2 topic hz /image_throttled/compressed      # expect ~20 Hz, not ~2 Hz

# 5. Launch the console (wrapper sets the DDS env for you)
~/m1_console.sh
```

---

## Gotchas

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` empty | Domain mismatch / NAT mode | `ROS_DOMAIN_ID=30`, adapter → **Bridged** |
| Nodes exit instantly | `RMW_IMPLEMENTATION` not set | Export `rmw_cyclonedds_cpp` (must match the robot) |
| `/tf` ok but no map | DDS mismatch breaks `/tf_static` | `tf2_echo base_link laser` to confirm |
| Image display black | QoS mismatch | Set display reliability to **Best Effort** |
| SLAM `queue is full` spam | TF timeout too short | Use `config/slam_toolbox_m1.yaml` |
| Gaze pointer far off | Stale calibration | Press **Calibrate** in the console, or **Reset** |
| GUI shows `no data` | DDS env unset in that process | Launch via `~/m1_console.sh`, never `ros2 run` |
| Camera ~2 Hz, other topics fine | Multicast over Wi-Fi | Unicast peer in `cyclonedds.xml` (§7.14) |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` around ROS setup | Bracket the `source` with `set +u` |

> `/scan` and the image are published **Best Effort**. A *Reliable* subscriber
> connects but receives nothing — the most confusing ROS 2 failure mode there is.
