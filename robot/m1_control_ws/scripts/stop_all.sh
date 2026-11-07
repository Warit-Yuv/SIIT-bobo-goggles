#!/usr/bin/env bash
# =============================================================================
# stop_all.sh - emergency stop and teardown, run inside the container.
#
#   /root/m1_control_ws/scripts/stop_all.sh          # kill our nodes only
#   /root/m1_control_ws/scripts/stop_all.sh --hard   # also kill vendor joy/apps
#
# Publishes an explicit zero Twist first, so the motors stop even if a node
# ignores SIGINT.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Argument validation.
#
# This script stops the robot. Silently ignoring an unrecognised flag means a
# typo (or a `--dry` that does not exist) performs the destructive action
# anyway. Only the documented flags are accepted; anything else exits without
# touching a single node.
HARD=0
for arg in "$@"; do
    case "$arg" in
        --hard)   HARD=1 ;;
        -h|--help) sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)
            echo "[stop] unknown argument: ${arg}" >&2
            echo "[stop] usage: stop_all.sh [--hard]   (nothing was stopped)" >&2
            exit 2
            ;;
    esac
done

M1_QUIET=1 source "${SCRIPT_DIR}/env.sh"

echo "[stop] publishing zero velocity..."
timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 && echo "[stop] zero Twist sent" || echo "[stop] /cmd_vel not available"

PATTERNS="control_arbiter_node wasd_teleop_node gaze_hud_node eeg_udp_bridge image_throttle_node"
if [ "$HARD" = "1" ]; then
    PATTERNS="${PATTERNS} yahboom_joy_M1 Mcnamu_driver_M1 base_node_M1 ydlidar_ros2_driver_node usb_cam_node_exe ekf_filter_node"
    echo "[stop] hard mode: also killing vendor/app nodes"
fi

for pat in $PATTERNS; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
        pkill -INT -f "$pat" 2>/dev/null || true
        sleep 0.3
        pkill -TERM -f "$pat" 2>/dev/null || true
        echo "[stop] killed ${pat}"
    fi
done

echo "[stop] remaining ROS nodes:"
ros2 node list 2>/dev/null | sed 's/^/    /' || true
