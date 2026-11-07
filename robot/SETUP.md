# Robot Setup — Step by Step

From a bare Raspberry Pi 5 to a working ROSMASTER M1 with teleoperation, gaze,
EEG and SLAM. Every step has a **verify** command with the expected output, so
you know it worked before moving on.

> **Difference from `docs/report.md`:** that document explains *what was built,
> what broke and the measured evidence*. This one is the **procedure** — follow
> it top to bottom.

**Target:** Raspberry Pi 5 (8 GB), Debian 12 Bookworm, ROS 2 Humble in Docker,
`ROS_DOMAIN_ID=30`. Robot IP in this guide: `10.0.0.54` (change to yours).

---

## Step 0 — What you need

| Item | Notes |
|---|---|
| ROSMASTER M1 with Pi 5 | 8 GB recommended |
| MicroSD / SSD with Raspberry Pi OS | Bookworm 64-bit, already on the robot |
| Docker image | `192.168.2.51:5000/ros-humble-m1:1.1.1-cam` (Yahboom) |
| STM32 driver board | CH340 → `/dev/myserial` |
| YDLIDAR Tmini Plus | CP210x → `/dev/rplidar` |
| USB camera | UVC → `/dev/video0` |
| Network | Robot and operator PC on the **same subnet** |

Everything below runs **on the Pi** over SSH unless stated otherwise.

---

## Step 1 — Confirm the OS and hardware

```bash
uname -m            # aarch64
cat /etc/os-release | head -2
free -h             # total ~7.9Gi
lsusb
```

**Expected** in `lsusb`:

```
1a86:7523  QinHeng CH340 serial converter      <- STM32 chassis
10c4:ea60  Silicon Labs CP210x UART Bridge     <- LiDAR
05a3:9230  ARC International Camera            <- USB camera
```

If the LiDAR or camera is missing, it is a **cable/power** problem — fix it
before continuing.

> **Verify the udev symlinks exist:**
> ```bash
> ls -l /dev/myserial /dev/rplidar /dev/video0
> ```
> Expected: `/dev/myserial -> ttyUSB*`, `/dev/rplidar -> ttyUSB*`,
> `/dev/video0`.
>
> If they are missing, check the rules:
> ```bash
> cat /etc/udev/rules.d/myserial.rules   # 1a86:7523 -> myserial
> cat /etc/udev/rules.d/ydlidar.rules    # 10c4:ea60 -> rplidar
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```

---

## Step 2 — Make sure Docker is running

```bash
docker --version
sudo systemctl is-active docker
docker images | grep ros-humble-m1
```

**Expected:** at least one `ros-humble-m1` image. If there is none, load it
first (from the Yahboom image tarball or registry) — this guide assumes it is
present.

---

## Step 3 — Install the two host scripts

Both scripts go in the home directory, next to the workspace:

```bash
# On the Pi
cd ~
cp robot/host/m1_docker.sh .
cp robot/host/m1.sh        .
chmod +x m1.sh m1_docker.sh
```

**Verify:**

```bash
ls -l ~/m1.sh ~/m1_docker.sh
```

Expected:

```
-rwxr-xr-x 1 pi pi ... /home/pi/m1.sh
-rwxr-xr-x 1 pi pi ... /home/pi/m1_docker.sh
```

Every command in this guide is written as `./m1.sh …`, which assumes you are in
`/home/pi`. If you are elsewhere, use the full path:

```bash
/home/pi/m1.sh status
```

> An earlier version of this guide put them in `~/bin`, which is not on the
> default `PATH` on Raspberry Pi OS. `./m1.sh: No such file or directory`
> means the scripts are in `~` and you used the wrong prefix.

---

## Step 4 — Put the workspace on the Pi

```bash
# On the Pi
mkdir -p ~/m1_control_ws
cp -r robot/m1_control_ws/* ~/m1_control_ws/
mkdir -p ~/m1_control_ws/logs
```

**Verify** the layout:

```bash
find ~/m1_control_ws -type f | grep -v __pycache__ | sort
```

You should see `src/`, `scripts/` and `integrations/`.

---

## Step 5 — Create the container

This is the step that matters most. Two details are deliberate:

* **`-v /dev:/dev`** — mounts the *whole* device tree, so `/dev/myserial` and
  `/dev/rplidar` appear and disappear **live**. Mounting individual devices
  instead freezes the list at container start, and a replugged LiDAR never
  works again without recreating the container.
* **`--init`** — puts `tini` in as PID 1 so killed ROS processes are reaped.
  Without it, every `ros2 launch` you stop leaves a zombie behind.

```bash
./m1_docker.sh recreate
```

**Verify:**

```bash
./m1_docker.sh status
```

Expected:

```
=== container ===
status=Up ...  image=192.168.2.51:5000/ros-humble-m1:1.1.1-cam

=== host devices ===
  /dev/myserial    -> /dev/ttyUSB0
  /dev/rplidar     -> /dev/ttyUSB1
  /dev/video0      -> /dev/video0

=== inside container ===
  /dev/myserial    -> /dev/ttyUSB0
  /dev/rplidar     -> /dev/ttyUSB1
  /dev/video0      -> /dev/video0
```

**All three must appear in both sections.** If a device is visible on the host
but missing inside, the container was created without `-v /dev:/dev` — rerun
`m1_docker.sh recreate`.

---

## Step 6 — Apply the network tuning (once)

```bash
sudo ~/m1_control_ws/scripts/tune_network.sh apply
```

**Verify:**

```bash
sudo ~/m1_control_ws/scripts/tune_network.sh --check
```

Expected:

```
=== managed tuning values ===
  net.core.rmem_max                16777216
  net.core.wmem_max                8388608
  net.core.netdev_max_backlog      5000
  vm.swappiness                    10
=== socket defaults (must stay at kernel default) ===
  net.core.rmem_default            212992
  net.core.wmem_default            212992
```

> `rmem_default` **must** stay at `212992`. Some tutorials tell you to set it to
> 16 MB — that applies to *every* socket including ssh and apt, and wastes
> memory. `rmem_max` is the ceiling and costs nothing, so that one is raised.

This is idempotent: running it twice leaves one managed block, not two.

---

## Step 7 — Build the overlay

```bash
./m1.sh build
```

**Verify:**

```bash
docker exec ros-humble-dev bash -lc \
  'source /root/m1_control_ws/scripts/env.sh >/dev/null 2>&1; ros2 pkg executables m1_control'
```

Expected — exactly 8:

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

> If files become unreadable from the host after a build (the container runs as
> root), run `./m1.sh fix-perms`.

---

## Step 8 — Verify the environment

```bash
./m1.sh diag
```

Expected — **0 failed**:

```
1. Environment
  [ OK ] ROS_DOMAIN_ID=30
  [ OK ] ROS distro humble
  [ OK ] overlay installed
2. Devices
  [ OK ] /dev/myserial -> /dev/ttyUSB0
  [ OK ] /dev/rplidar -> /dev/ttyUSB1
  [ OK ] /dev/video0 -> /dev/video0
...
 result: 0 failed, N warning(s)
```

Any `[FAIL]` must be fixed before continuing.

---

## Step 9 — Staged bringup (test one subsystem at a time)

Do these in order. Each stage isolates one failure mode, so when something
breaks you know exactly what it was.

### 9.1 Chassis driver alone

**Elevate the robot so the wheels spin freely. Put a hand on the power switch.**

```bash
./m1.sh shell
# inside the container:
source /root/m1_control_ws/scripts/env.sh
ros2 run yahboomcar_bringup Mcnamu_driver_M1
```

**Verify** in a second terminal (`./m1.sh shell`):

```bash
ros2 topic echo /voltage --once      # expect >= 10.3 (12.3 measured)
ros2 topic echo /edition --once      # expect '3.6'
```

> Below **10.3 V**: charge before continuing. A brown-out mid-motion is worse
> than a delay.

Stop it with `Ctrl+C`.

### 9.2 Servo / gimbal

With the driver running, send one command at a time and **watch the head**:

```bash
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 90}"
```

**Verify:** the camera head sits at its rest pose, looking roughly straight
ahead and slightly down.

Now confirm the axis mapping, because the names are not what you would guess:

```bash
# s1 = TILT: larger looks UP
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 70, s2: 90}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 30, s2: 90}"

# s2 = PAN: larger looks LEFT
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 120}"
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 60}"
```

| | Axis | Larger value |
|---|---|---|
| **`s1`** | **TILT** (up/down) | look **up** |
| **`s2`** | **PAN** (left/right) | look **left** |

Finish by re-centring:

```bash
ros2 topic pub --once /Servo yahboomcar_msgs/msg/ServoControl "{s1: 40, s2: 90}"
```

> **`s1` is tilt, not pan** — this is the opposite of what the parameter names
> suggest, and getting it wrong makes the gaze controller pan when it means to
> tilt. Confirmed on this robot; see `docs/report.md` §10.
>
> **Ranges are not confirmed.** `s1` is configured 0..180 and `s2` 0..110 (the
> driver hard-clamps `s2`). Those are the vendor joystick limits and have not
> been checked against the mechanical stops — move slowly and stop before the
> servo buzzes or stalls.

### 9.3 Camera

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p image_width:=640 -p image_height:=480 \
  -p pixel_format:="yuyv2rgb" -p framerate:=30.0
```

**Verify:**

```bash
ros2 topic hz /image_raw/compressed        # expect ~30 Hz
```

> **Use `yuyv2rgb`, never MJPG.** This camera's MJPG mode is locked to 120 Hz at
> 640×480; asking it for 30 fps makes the V4L2 driver fail with
> `Select timeout, exiting...`. YUYV gives a true 30 Hz and 4× more light.

### 9.4 LiDAR

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

**Verify:**

```bash
ros2 topic hz /scan        # expect ~10 Hz
```

Expected log:

```
[YDLIDAR] Lidar running correctly! The health status: good
Model: Tmini Plus
[YDLIDAR] Successed to start scan mode
Now lidar is scanning...
```

> **If you see `cannot retrieve YDLidar health code: ffffffff`** the MCU was not
> ready yet. Just restart the launch — it is a startup race, not a fault. The
> production launch adds `respawn=True` so this retries by itself.
>
> If it still fails, identify the device with the vendor tool:
> ```bash
> cd /root/yahboomcar_ros2_ws/software/YDLidar-SDK-master/build
> echo 2 | ./tmini_test
> ```
> If that works, the hardware is fine and it is a driver/config issue.
> `python3 scripts/lidar_probe.py --port /dev/rplidar` probes the raw protocol.

### 9.5 Everything together

```bash
./m1.sh start
```

**Verify** in another terminal:

```bash
./m1.sh status
```

Expected nodes:

```
/Mcnamu_driver_M1   /base_node_M1   /ekf_filter_node
/imu_filter_madgwick   /joint_state_publisher   /robot_state_publisher
/usb_cam   /image_throttle_node   /ydlidar_ros2_driver_node
/control_arbiter_node   /eeg_udp_bridge
```

Expected rates:

```bash
ros2 topic hz /scan                        # ~10.0 Hz
ros2 topic hz /image_throttled/compressed  # ~20.0 Hz
ros2 topic hz /odom                        # ~10.6 Hz
ros2 topic hz /tf                          # ~20.9 Hz
```

Check the TF tree is complete:

```bash
ros2 run tf2_tools view_frames
```

Expected chain:

```
odom -> base_footprint -> base_link -> laser
                                   -> cam1_Link -> cam2_Link
                                   -> imu_link
```

---

## Step 10 — Safety validation (mandatory before floor tests)

**Wheels off the ground.**

```bash
docker exec -it ros-humble-dev bash -lc \
  'source /root/m1_control_ws/scripts/env.sh >/dev/null 2>&1; python3 /root/m1_control_ws/scripts/validate_safety.py'
```

Expected:

```
 12/12 checks passed - all good
```

This proves:

| Check | Meaning |
|---|---|
| 1 publisher on `/cmd_vel` | No rogue controller fighting the arbiter |
| Watchdog ≤ 0.35 s | Robot halts if commands stop |
| Mode isolation | Gaze ignored while in keyboard mode |
| Clamping | Oversized commands capped at the envelope |
| E-STOP latch | Mode 0 holds zero until you leave it |
| EEG end-to-end | UDP → motion → stop on link loss |

**If the watchdog check fails, do not put the robot on the floor.**

---

## Step 11 — Drive it

```bash
./m1.sh teleop
```

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

> **`q` strafes left — it does not quit.** Quitting is Ctrl+C. When the node
> exits it prints `restart with: ./m1.sh teleop`.
>
> Keys are not "held" — a command stays alive for 0.25 s after the last press
> and then decays to zero, so a single tap gives a short nudge.

---

## Step 12 — Stop and clean up

```bash
./m1.sh stop          # stop our nodes (sends zero velocity first)
./m1.sh stop --hard   # also stop vendor/app nodes
```

---

## Step 13 — Daily operation

```bash
ssh pi@10.0.0.54
cd ~
./m1.sh start          # foreground
./m1.sh bg             # or background, logs to m1_control_ws/logs/robot.log
./m1.sh logs           # follow the log
```

| Command | Purpose |
|---|---|
| `m1.sh start` | Full stack, foreground |
| `m1.sh bg` | Full stack, background |
| `m1.sh stop [--hard]` | Stop nodes |
| `m1.sh diag` | Health check (read-only) |
| `m1.sh status` | Node + topic snapshot |
| `m1.sh shell` | Shell inside the container |
| `m1.sh teleop` | Keyboard drive |
| `m1.sh test-eeg` | Simulate the EEG laptop |
| `m1.sh build` | Rebuild overlay |
| `m1.sh devices` | Device + container state |
| `m1.sh fix-perms` | Return workspace ownership to `pi` |

---

## Step 14 — After a reboot

The container has `--restart unless-stopped`, so it comes back **idle** — it
does not auto-start any node. This is deliberate: a reboot must never silently
re-arm a controller.

```bash
./m1_docker.sh status     # container up? devices resolved?
./m1.sh diag              # 0 failed?
./m1.sh start
```

> The udev symlinks survive `ttyUSB` renumbering. Measured after a real reboot:
> `/dev/myserial -> ttyUSB1` and `/dev/rplidar -> ttyUSB0` (the numbers had
> swapped) and both still resolved correctly. This is why the scripts use the
> **symlinks**, never `ttyUSB*` directly.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `/dev/rplidar` missing | LiDAR unplugged / power | Check `lsusb` for `10c4:ea60` |
| LiDAR `health code ffffffff` | MCU startup race | Restart launch (auto-retries in production) |
| `/scan` never appears | Device absent at launch | `m1.sh start` again, or `lidar:=true` |
| Camera `Select timeout` | MJPG at 30 fps | Use `pixel_format:=yuyv2rgb` |
| Camera dark | 120 Hz shutter | Same fix — YUYV 30 Hz is 4× brighter |
| Robot drives erratically | Two `/cmd_vel` publishers | `ros2 topic info /cmd_vel`; `m1.sh stop --hard` |
| Nodes exit instantly | `RMW_IMPLEMENTATION` unset while the PC uses CycloneDDS | Export `rmw_cyclonedds_cpp` on **both** machines |
| `/tf` arrives but SLAM makes no map | DDS mismatch breaks `/tf_static` | Same — `tf2_echo base_link laser` proves it |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` active while sourcing ROS setup files | Fixed in `env.sh`; if you see it, update your copy |
| Servo does not move | `yahboomcar_msgs` not sourced | Use `scripts/env.sh` |
| Video freezes after seconds | UDP buffers | `tune_network.sh apply`, reboot |
| Zombie processes pile up | Container lacks `--init` | `m1_docker.sh recreate` |
| LiDAR `Check Sum 0xC5A3 != 0x2191` spam | **Two or more stacks running** — drivers interleave bytes on `/dev/rplidar` | `m1.sh stop --hard`, then start once |
| `Fail to get baseplate device information` | Same — serial port contention | Same |
| `usb_cam` dies: `terminate called after throwing an instance of 'char*'` | **Two or more stacks running** — only one process can own `/dev/video0` | Same |
| `Failed to meet update rate!` from EKF | Duplicate EKF/IMU nodes competing | Same |
| `REFUSING TO START: a robot stack is already running` | Singleton guard working as intended | `./m1.sh stop --hard`, or add `--force` |
| `stop` says done but `start` still refuses | Older `stop_all.sh` did not kill the supervisor that holds the lock | Update `stop_all.sh`; `./m1.sh stop` now falls back to `kill_all.sh` |
| Orphaned `ekf_node` processes accumulate | Pattern matched the node name (`ekf_filter_node`) not the binary (`ekf_node`) | Fixed; `./m1.sh stop --hard` now removes them |
| Build leaves root-owned files | Container runs as root | `m1.sh fix-perms` |

### Full reset

```bash
./m1.sh stop --hard
docker exec ros-humble-dev bash /root/m1_control_ws/scripts/kill_all.sh
./m1.sh start
```

---

## What runs where

| Runs on the Pi | Runs on the operator PC |
|---|---|
| Chassis driver, odometry, EKF | `slam_toolbox` |
| LiDAR + camera acquisition | RViz |
| **`control_arbiter_node`** (safety) | Gaze model / HUD |
| EEG + gaze UDP bridges | `sensor_sync_node` |
| Gaze deadzone/dwell policy | |

> **The arbiter deliberately lives on the robot.** A watchdog on the far side of
> a Wi-Fi link cannot stop a robot whose Wi-Fi link just died.

See `operator-pc/SETUP.md` for the PC side.
