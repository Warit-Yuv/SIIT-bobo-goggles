#!/usr/bin/env bash
# =============================================================================
# kill_vm.sh - stop the operator-side stack on the VM.
#
#   ./kill_vm.sh            stop SLAM, RViz and any m1_control nodes
#   ./kill_vm.sh --status   only report what is running
#
# Why this exists: `ros2 launch` leaves its children behind if the terminal is
# closed or the SSH session drops. Two async_slam_toolbox instances then both
# publish /map with different maps, and RViz flickers between them - which
# looks like "the map keeps resetting" rather than "two mappers are running".
#
# Safe to run at any time. Touches nothing on the robot.
# =============================================================================
set -uo pipefail

PATTERNS=(
    "async_slam_toolbox"
    "slam_toolbox"
    "rviz2"
    "gaze_hud_node"
    "gaze_control_node"
    "gaze_udp_bridge"
    "sensor_sync_node"
    "m1_panel"
    "control_arbiter_node"
    "wasd_teleop_node"
    "eeg_udp_bridge"
    "image_throttle_node"
    "ros2 launch m1_control"
)

status() {
    echo "=== operator-side processes ==="
    local found=0
    for pat in "${PATTERNS[@]}"; do
        local pids
        pids=$(pgrep -f -- "$pat" 2>/dev/null | grep -vx -e "$$" -e "$PPID" || true)
        if [ -n "$pids" ]; then
            found=1
            for p in $pids; do
                printf '  %-8s %-24s %s\n' "$p" "$pat" \
                    "$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-60)"
            done
        fi
    done
    [ "$found" -eq 0 ] && echo "  (nothing running)"
    echo
    echo "=== /map publishers (must be 0 or 1) ==="
    ros2 topic info /map 2>/dev/null | grep -E "Publisher count" || echo "  (no /map)"
}

if [ "${1:-}" = "--status" ]; then
    status
    exit 0
fi

SELF=$$
for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PPID" ] && continue
        kill -INT "$pid" 2>/dev/null || true
    done
done

# Give them a moment to shut down cleanly, then insist.
sleep 3
for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PPID" ] && continue
        kill -9 "$pid" 2>/dev/null || true
    done
done

sleep 1

# The ROS daemon caches node lists and will keep reporting dead nodes.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "[kill_vm] done"
status
