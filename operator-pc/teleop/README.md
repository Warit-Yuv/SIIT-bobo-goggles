# VM-side teleoperation (in progress)

Reserved for the operator-PC teleoperation code: UI, control client, and any
VM-side nodes.

## Constraints to respect

Whatever is added here must obey the robot's safety contract:

1. **Never publish `/cmd_vel` or `/Servo` directly.**
   Publish to a mode-specific topic instead:

   | Source | Velocity topic | Servo topic | Mode |
   |---|---|---|---|
   | Keyboard | `/cmd_vel/keyboard` | `/servo/keyboard` | `1` |
   | Gaze | `/cmd_vel/gaze` | `/servo/gaze` | `2` |
   | EEG | `/cmd_vel/eeg` | — | `3` |

2. **Select your mode** before commanding:

   ```bash
   ros2 topic pub --once /control_mode std_msgs/msg/UInt8 "{data: 1}"
   ```

3. **Keep publishing.** The arbiter's watchdog zeroes velocity after 0.35 s
   without a fresh command, so a UI that sends only on change will stall.

4. **Respect the envelope.** The arbiter clamps to
   `vx ≤ 0.25 m/s`, `vy ≤ 0.20 m/s`, `ωz ≤ 0.80 rad/s`, so larger values are
   silently reduced rather than rejected.

5. **Use Best Effort QoS** for `/scan` and `/image_throttled/compressed`.
