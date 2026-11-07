# ROSMASTER M1 — Operator PC (VMware Ubuntu) Setup Guide

**Operator PC:** `Rescue-Ubuntu` — Ubuntu 22.04.5 LTS — **`10.0.0.102`**
**Robot:** Raspberry Pi 5 — **`10.0.0.54`**
**ROS 2 Humble** — **`ROS_DOMAIN_ID=30`** (must match the robot exactly)

This document sets up the VM side: visualisation, SLAM, gaze control and EEG.
It has been verified against your actual VM — the sections marked **✅ verified**
were tested live, not assumed.

---

## 0. Current State of Your VM (measured)

| Item | Value | Status |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS, kernel 6.8 | ✅ |
| ROS 2 | Humble at `/opt/ros/humble` | ✅ |
| CPU / RAM | 16 cores / 19 GB | ✅ |
| `slam_toolbox` | `/opt/ros/humble` | ✅ |
| `rviz2`, `nav2_bringup` | present | ✅ |
| `tf2_ros`, `message_filters` | present | ✅ |
| `image_transport`, `cv_bridge` | present | ✅ |
| Python | 3.10.12, `cv2` 4.11.0, `numpy` 1.26.4 | ✅ |
| Network | `ens33` = `10.0.0.102/24`, bridged | ✅ |
| Latency to robot | **1.3 ms** | ✅ |
| `robot_localization` | **missing** | ⚠️ not needed on PC |
| `usb_cam` | **missing** | ⚠️ not needed on PC |
| `ROS_DISTRO` env | **unset** | ⚠️ add to `.bashrc` |

Nothing essential is missing. `robot_localization` and `usb_cam` are robot-side
only — do **not** install them here.

---

## 1. Shell Environment (do this first)

`ROS_DISTRO` is unset in your shell, so ROS is not sourced by default. Add it:

```bash
cat >> ~/.bashrc <<'EOF'

# ===== ROS 2 Humble (ROSMASTER M1 operator) =====
export ROS_DOMAIN_ID=30          # MUST match the robot
source /opt/ros/humble/setup.bash
source ~/m1_control_ws/install/setup.bash   # after step 3

# FastDDS is what the robot runs. Do NOT export RMW_IMPLEMENTATION:
# CycloneDDS is not installed on either machine and setting it breaks every node.
unset RMW_IMPLEMENTATION

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID | ROS=$ROS_DISTRO"
EOF
source ~/.bashrc
```

Verify:

```bash
echo $ROS_DOMAIN_ID      # must print 30
echo $ROS_DISTRO         # must print humble
```

> **Why `unset RMW_IMPLEMENTATION` matters:** the handbook recommends
> `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, but CycloneDDS is not installed on
> either machine. Setting it makes every node fail to start with
> `failed to load rmw implementation`. FastDDS handles the measured load
> (20 Hz video + 10 Hz scan) without difficulty.

---

## 2. Kernel Network Buffers

Large JPEG frames arrive as fragmented UDP. If the kernel buffer is too small,
fragments are dropped and the video appears to freeze.

```bash
sudo tee -a /etc/sysctl.conf >/dev/null <<'EOF'

# ===== ROS 2 DDS tuning (operator PC) =====
# Only ceilings are raised. *_default is left at the kernel value because it
# applies to EVERY socket (ssh, apt, browser) and reserves memory needlessly.
net.core.rmem_max=16777216
net.core.wmem_max=8388608
net.core.netdev_max_backlog=5000
EOF

sudo sysctl -p
```

Verify:

```bash
sysctl net.core.rmem_max net.core.rmem_default
# rmem_max     = 16777216   <- raised
# rmem_default = 212992     <- must stay at default
```

---

## 3. Copy and Build the Workspace

The overlay lives on the robot. Copy it across and build it locally:

```bash
# From the Pi (or run this on the Pi):
#   scp -r pi@10.0.0.54:/home/pi/m1_control_ws ~/m1_control_ws

cd ~/m1_control_ws
colcon build --symlink-install --packages-select m1_control
source install/setup.bash
```

> The package is designed to build on the PC: the `yahboomcar_msgs` import in
> the arbiter is **lazy**, so a missing robot-only message package does not
> break the build. Only the arbiter needs it, and the arbiter runs on the robot.

Verify:

```bash
ros2 pkg executables m1_control
```

Expected:

```
m1_control control_arbiter_node
m1_control eeg_udp_bridge
m1_control gaze_control_node
m1_control gaze_hud_node
m1_control gaze_udp_bridge
m1_control image_throttle_node
m1_control sensor_sync_node
m1_control wasd_teleop_node
```

---

## 4. Verify Connectivity — ✅ verified working

Start the robot first:

```bash
# On the robot
ssh pi@10.0.0.54
cd /home/pi && ./m1.sh start
```

Then on the VM:

```bash
ping -c 3 10.0.0.54          # expect ~1-3 ms
ros2 topic list              # must show /scan, /odom, /tf, /image_throttled/compressed
```

### Expected rates (measured over your Wi-Fi)

```bash
ros2 topic hz /image_throttled/compressed   # ~20.0 Hz
ros2 topic hz /scan                         # ~10.0 Hz
ros2 topic hz /odom                         # ~10.6 Hz
ros2 topic hz /tf                           # ~20.9 Hz
```

If `ros2 topic list` is empty, it is almost always one of:

| Check | Command |
|---|---|
| Domain ID matches | `echo $ROS_DOMAIN_ID` (must be 30) |
| VM is bridged, not NAT | `ip -c a` → `ens33` on `10.0.0.x` |
| Same subnet as robot | `ping 10.0.0.54` |
| Stale daemon | `ros2 daemon stop && ros2 daemon start` |

---

## 5. Visualise with RViz

```bash
ros2 run rviz2 rviz2
```

Set these in the left panel:

| Display | Topic | Notes |
|---|---|---|
| **LaserScan** | `/scan` | Reliability → **Best Effort** |
| **Image** | `/image_throttled/compressed` | Reliability → **Best Effort** |
| **Map** | `/map` | appears once SLAM runs |
| **TF** | — | shows the frame tree |
| **Odometry** | `/odom` | |

**Global Options → Fixed Frame:** `map` (or `odom` before SLAM starts).

> **Reliability must be Best Effort** for `/scan` and the image. They are
> published Best Effort on the robot; a Reliable subscriber will connect but
> receive **nothing** — a classic and confusing ROS 2 mismatch.

---

## 6. SLAM Mapping

### 6.1 Use the tuned config (recommended)

The stock `online_async_launch.py` profile has two problems on this setup:

1. It warns that the laser range is wrong (defaults 0–20 m; the Tmini Plus is
   0.03–12 m).
2. Its transform timeout is **0**, so over Wi-Fi it drops scans with
   `Message Filter dropping message ... queue is full`.

Our `config/slam_toolbox_m1.yaml` fixes both. Launch the whole operator stack:

```bash
ros2 launch m1_control m1_operator.launch.py slam:=true rviz:=true
```

### 6.2 Or run SLAM alone

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file ~/m1_control_ws/src/m1_control/config/slam_toolbox_m1.yaml
```

**✅ Verified on your VM** — the range warnings disappear and the
`queue is full` spam stops after the first scan:

```
[INFO] [slam_toolbox]: Registering sensor: [Custom Described Lidar]
   (one startup drop while the TF buffer fills, then clean)
```

Map produced:

```
cells=10877 occupied=78 free=994 unknown=9805
map 73x149 @ 0.050m
```

### 6.3 Save the map

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/room1
# -> room1.pgm + room1.yaml
```

> **The map only grows while the robot moves.** A stationary robot produces a
> small map. This is correct behaviour.

---

## 7. Gaze Control

Two supported setups. **Both end at the same topic** (`/gaze/point`), so the
robot side is identical.

### 7.1 Option A — GazeModel natively on Windows (recommended)

**Why native Windows:** GazeModel is a PySide6 desktop app whose output is a
coordinate on the *physical* display. Inside a VM, "the screen" is only the VM
window, so calibration would map to that window instead of your monitor.

```
GazeModel (Windows) --UDP:9100--> robot gaze_udp_bridge --/gaze/point--> gaze_control_node
```

**Step 1 — copy the bridge module into the GazeModel repo:**

```bash
# From the robot / your copy of the workspace:
#   integrations/gazemodel/gaze_m1_bridge.py
# Copy it to  GazeModel/gaze/m1_bridge.py
```

**Step 2 — patch `gaze/app.py`** (four small edits):

```python
# (1) near the top, with the other imports
from .m1_bridge import make_sender

# (2) just below the M1_TRANSPORT / M1_HOST constants at module level
M1_TRANSPORT = 'udp'      # 'udp' on Windows, 'ros' inside the VM
M1_HOST = '10.0.0.54'     # robot IP

# (3) in GazeApp.__init__, after self.screen = QGuiApplication.primaryScreen()
_geo = self.screen.geometry()
self.m1 = make_sender(
    kind=M1_TRANSPORT,
    host=M1_HOST,
    screen_size=(_geo.width(), _geo.height()),
)

# (4) in on_face, right after  x, y = self.filter(...)
if self.m1 is not None:
    self.m1.send_pixels(x, y, model=self.active)
```

**Step 3 — run it and verify the robot receives gaze:**

```bash
# On the robot
ros2 run m1_control gaze_udp_bridge        # listens on UDP :9100
ros2 run m1_control gaze_control_node
```

```powershell
# On Windows
.\.venv\Scripts\python.exe main.py
```

You should see `[gaze] source connected from <windows-ip>` on the robot.

### 7.2 Option B — GazeModel inside the VM

Works, but **run VMware fullscreen during calibration**, or the calibrated
screen size will be the VM window rather than your monitor.

```bash
# In the VM
pip3 install PySide6 safetensors          # mediapipe/numpy already present
# set M1_TRANSPORT = 'ros' in the patch above, then:
python3 main.py
```

With `M1_TRANSPORT='ros'` the bridge publishes `/gaze/point` directly over
Wi-Fi — no UDP hop, and the robot's `gaze_udp_bridge` is not needed.

### 7.3 Test without a headset or webcam

```bash
# On the robot
python3 scripts/test_gaze.py --hold 0.9
```

**✅ Verified output:**

```
target                         zone                   vx     vy     wz
centre (deadzone: PTZ only)    CENTER               0.00   0.00   0.00
forward                        FORWARD              0.20   0.00   0.00
centre                         CENTER               0.00   0.00   0.00
strafe right                   STRAFE_RIGHT         0.00  -0.20   0.00
centre                         CENTER               0.00   0.00   0.00
forward-right                  FORWARD_RIGHT        0.14  -0.14   0.00
after silence: STOPPED (pass)
```

### 7.4 How the control policy works

| Region | Behaviour |
|---|---|
| Inside deadzone (25 % of half-screen) | Chassis **still**; gaze offsets drive PTZ pan/tilt proportionally |
| Outside, held ≥ **0.40 s** | Drive / strafe / turn in that direction |
| Gaze stops arriving > **0.6 s** | Velocity zeroed (link watchdog) |

The dwell gate exists to reject **saccades** — without it, every natural glance
away from centre would trigger a spurious drive command.

### 7.5 If pan/tilt move the wrong way

```bash
ros2 run m1_control gaze_control_node --ros-args \
    -p invert_pan:=true -p invert_tilt:=true
```

Set it permanently in `config/m1_params.yaml` once confirmed.

---

## 8. EEG Control

The EEG pipeline (`muse_robot/`) runs on the **Windows laptop** — it needs
Bluetooth to the Muse 2 headset. Only a small UDP bridge runs on the robot.

```
Muse 2 --BLE--> BlueMuse --LSL--> muse_robot (Windows) --UDP:9000--> robot
```

### 8.1 On Windows

```powershell
# 1. Pair the Muse 2, open BlueMuse, press Start Streaming
# 2. Run the menu
MuseRobot.bat
#    1. Check signal      (every channel should be OK)
#    2. Calibrate         (~3 min)
#    5. Run - drive the YAHBOOM ROBOT over Wi-Fi
#       enter robot IP: 10.0.0.54
```

Allow Python through **Windows Defender Firewall** on *Private* networks when
prompted, or the robot's replies will be blocked.

### 8.2 On the robot

```bash
ros2 run m1_control eeg_udp_bridge      # UDP :9000 -> /cmd_vel/eeg
```

### 8.3 Test without a headset — ✅ verified

```bash
python3 scripts/test_eeg_udp.py --ip 127.0.0.1 --sequence
```

Verified: `GO` drives forward at 0.15 m/s; stopping the heartbeat halts the
robot in **0.88 s**.

### 8.4 Gesture → motion

| Gesture | Command | Robot action (M1 mecanum) |
|---|---|---|
| Double blink | `GO` | Drive forward |
| Jaw clench | `STOP` | Stop immediately (always wins) |
| Look left | `LEFT` | **Rotate in place** CCW |
| Look right | `RIGHT` | **Rotate in place** CW |

> The upstream project assumes an Ackermann car that cannot turn on the spot.
> The M1 is a **mecanum** base, so `LEFT`/`RIGHT` rotate in place instead of
> driving a 1.5 s arc — no arc, no forward creep.

### 8.5 Tuning

```bash
ros2 run m1_control eeg_udp_bridge --ros-args -p speed:=0.10
# or on the command line for the standalone bridge:
python3 eeg_udp_bridge.py --speed 0.10 --steer 0.5 --link-timeout 1.5
```

---

## 9. Sensor Synchronisation (optional)

The camera (20 Hz) and LiDAR (10 Hz) run on independent clocks, so pairing
"newest image" with "newest scan" silently mixes data up to 100 ms apart — at
0.2 m/s that is 2 cm of travel, plus any gimbal motion.

`sensor_sync_node` uses `ApproximateTimeSynchronizer` to pair the closest pair:

```bash
ros2 run m1_control sensor_sync_node
ros2 run m1_control sensor_sync_node --ros-args -p show_window:=true
```

Outputs `/m1/synced/overlay` (image with the matching scan drawn on it) and
`/m1/synced/stats`. The overlay is **schematic, not calibrated** — this camera
publishes no `CameraInfo`, so a true projection needs calibration. It exists to
make a bad time-pairing visible, not to measure.

Tuning:

```bash
# If it never pairs, widen the slop:
ros2 run m1_control sensor_sync_node --ros-args -p slop:=0.25
```

---

## 10. Run Everything

```bash
# Terminal 1 - operator stack (gaze HUD + SLAM + RViz)
ros2 launch m1_control m1_operator.launch.py

# Terminal 2 - RViz only, if you prefer separate windows
ros2 run rviz2 rviz2

# Terminal 3 - sensor sync (optional)
ros2 run m1_control sensor_sync_node
```

Useful launch arguments:

```bash
ros2 launch m1_control m1_operator.launch.py gaze:=false           # SLAM only
ros2 launch m1_control m1_operator.launch.py slam:=false rviz:=false
ros2 launch m1_control m1_operator.launch.py gaze_source:=mouse    # click to aim
```

---

## 11. Control Modes

Switch from any machine on domain 30:

```bash
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 1}"   # KEYBOARD
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 2}"   # GAZE
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 3}"   # EEG
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 0}"   # E-STOP
```

**E-STOP is latched** — it keeps holding zero until you explicitly select
another mode. Only the active mode's topic is forwarded, so keyboard, gaze and
EEG can never fight each other.

Safety envelope enforced by the arbiter on the robot:

| Axis | Limit |
|---|---|
| `vx` | ±0.25 m/s |
| `vy` | ±0.20 m/s |
| `ωz` | ±0.80 rad/s |
| Watchdog | 0.35 s |

---

## 12. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` empty | Domain mismatch / NAT mode | `echo $ROS_DOMAIN_ID`=30; set adapter to **Bridged** |
| Nodes exit instantly | `RMW_IMPLEMENTATION` set | `unset RMW_IMPLEMENTATION` |
| Image display black | QoS mismatch | Set display reliability to **Best Effort** |
| Video freezes after seconds | Kernel buffers | Apply §2, then reboot |
| SLAM `queue is full` spam | TF timeout too short | Use `config/slam_toolbox_m1.yaml` |
| SLAM never starts mapping | No `/scan` or no TF | `ros2 topic hz /scan`; `ros2 run tf2_tools view_frames` |
| Gaze pointer far off | Calibration stale | Press **F** (quick fix) or **C** (recalibrate) |
| Gaze ignores head movement | Calibrated in one posture only | Recalibrate across all 4 postures |
| Robot NO REPLY (EEG) | Windows Firewall | Allow Python on **Private** networks |
| Robot turns wrong way | Steering sign | `--steer -0.6` |
| Pan/tilt inverted | Servo sign | `-p invert_pan:=true -p invert_tilt:=true` |
| Map is tiny | Robot not moving | Expected — drive around to grow it |

### Quick health check

```bash
echo "domain: $ROS_DOMAIN_ID"
ping -c 2 10.0.0.54
ros2 topic list | grep -E "scan|odom|image_throttled"
ros2 topic hz /scan
```

---

## 13. Checklist

- [ ] `ROS_DOMAIN_ID=30` and `ROS_DISTRO=humble` in `~/.bashrc`
- [ ] `unset RMW_IMPLEMENTATION` in `~/.bashrc`
- [ ] `/etc/sysctl.conf` tuned, `rmem_default` **still 212992**
- [ ] `~/m1_control_ws` copied and `colcon build` succeeded
- [ ] `ros2 topic list` shows `/scan`, `/odom`, `/image_throttled/compressed`
- [ ] `/scan` at ~10 Hz, image at ~20 Hz
- [ ] RViz shows laser scan (Best Effort) and camera image
- [ ] `slam_toolbox_m1.yaml` produces `/map`
- [ ] Gaze reaches `/gaze/point` (either transport)
- [ ] Pan/tilt direction confirmed (set `invert_*` if needed)
- [ ] EEG `GO`/`STOP` verified with `test_eeg_udp.py`

---

## 14. Security Note

An SSH key from the robot to this VM is installed at `~/.ssh/authorized_keys`
(user `rescue`), so `ssh rescue@10.0.0.102` works without a password.

The account password was sent in plain text during setup. **Rotate it:**

```bash
passwd
```

Also consider disabling password auth once keys work:

```bash
sudo sed -i 's/^#*PasswordAuthentication.*/PasswordAuthentication no/' /etc/ssh/sshd_config
sudo systemctl restart ssh
```

> Only do this after confirming key login works, or you can lock yourself out.
