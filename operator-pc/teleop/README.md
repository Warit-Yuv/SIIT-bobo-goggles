# VM-side teleoperation

The operator-PC teleoperation is **`m1_console`** — a PyQt5 console that lives in
the robot's ROS package so both machines build from the same source.

| | |
|---|---|
| Source | `robot/m1_control_ws/src/m1_control/m1_control/m1_console.py` |
| Launch | `~/m1_console.sh` (wrapper on the operator PC) |
| Docs | [`../README.md`](../README.md#operator-console) |

```bash
~/m1_console.sh
```

It provides start/stop/diagnose over SSH, mode selection with E-STOP, keyboard
teleop with in-window key capture, live camera, gaze calibration, and voice
control — all in one window.

---

## Safety contract

Anything added here must obey the robot's safety rules. They are not
suggestions; the arbiter enforces them and a violation shows up as a robot that
ignores you.

### 1. Never publish `/cmd_vel` or `/Servo` directly

Only `control_arbiter_node` may publish those. Publish to a mode-specific topic:

| Source | Velocity topic | Servo topic | Mode |
|---|---|---|---|
| Keyboard | `/cmd_vel/keyboard` | `/servo/keyboard` | `1` |
| Gaze | `/cmd_vel/gaze` | `/servo/gaze` | `2` |
| EEG | *(none - see below)* | — | `3` |

The EEG has **no velocity topic**. Protocol v1 sends toggles, not speeds, so
the bridge publishes state on `/eeg/run`, `/eeg/mode` and `/eeg/link`, and
`gaze_control_node` gates the gaze command on them. Mode `3` alone does not
make an EEG move the robot; it needs `run: true` from a jaw clench as well.

### 2. Select your mode before commanding

```bash
ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 1}"
```

Keyboard is the exception: a non-zero keyboard command wins for 0.4 s in **any**
mode, so a nudge never requires switching first. That is deliberate — a human
pressing a key cannot arrive by accident the way a misclassified EEG gesture can.

### 3. Keep publishing

The arbiter's watchdog zeroes velocity after 0.35 s without a fresh command. A
UI that publishes only on change will stall. The console publishes at 20 Hz
whether or not a key is held, and relies on zero-valued commands being ignored
by the override check rather than being treated as input.

### 4. Respect the envelope

The arbiter clamps to `vx ≤ 0.40 m/s`, `vy ≤ 0.20 m/s`, `ωz ≤ 0.80 rad/s`, so
larger values are silently reduced rather than rejected. Servo limits are ROS
parameters (`servo_s1_min/max`, `servo_s2_min/max`) — read them, do not
hard-code them. A private copy is exactly the bug in `robot/docs/report.md`
§7.12.

### 5. Use Best Effort QoS

`/scan` and `/image_throttled/compressed` are published **Best Effort**. A
Reliable subscriber connects successfully and then receives nothing, which is
the most confusing failure mode ROS 2 has.

### 6. Set the DDS environment before starting

`ROS_DOMAIN_ID=30` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` must match the
robot, and `CYCLONEDDS_URI` should point at a config with a **unicast peer** at
the robot's address. Multicast over Wi-Fi drops most of the video (§7.14).

A mismatch produces no error at all — just an empty GUI (§7.13). Use the
wrapper.
