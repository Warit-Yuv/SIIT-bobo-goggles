#!/usr/bin/env bash
# =============================================================================
# kill_vm.sh - stop the operator-side stack on the VM.
#
#   ./kill_vm.sh              stop the VIEW stack (SLAM, RViz, gaze nodes)
#   ./kill_vm.sh --all        stop everything, including the console
#   ./kill_vm.sh --status     only report what is running, change nothing
#
# ---------------------------------------------------------------------------
# WHY THIS EXISTS
# ---------------------------------------------------------------------------
# `ros2 launch` leaves its children behind when the terminal closes or the SSH
# session drops. Two async_slam_toolbox instances then both publish /map with
# different maps, and the map appears to jump around - which reads as "the map
# keeps resetting" rather than "two mappers are running".
#
# ---------------------------------------------------------------------------
# WHY THE CONSOLE IS NOT KILLED BY DEFAULT
# ---------------------------------------------------------------------------
# The console's "Stop view" button runs this script. If the console were in the
# default kill list, pressing that button would destroy the window that is
# reporting the result - and the app would vanish for no visible reason. So the
# console is only stopped with --all, which is the "close everything" path a
# person runs from a terminal.
#
# Safe to run at any time. Touches nothing on the robot.
# =============================================================================
set -uo pipefail

# The view stack: what "Launch SLAM + RViz" starts, plus the gaze nodes.
VIEW_PATTERNS=(
    "async_slam_toolbox"
    "slam_toolbox"
    "rviz2"
    "gaze_hud_node"
    "gaze_control_node"
    "gaze_udp_bridge"
    "sensor_sync_node"
    "ros2 launch m1_control"
)

# Operator-side processes that are not part of the view. Only removed by --all.
EXTRA_PATTERNS=(
    "m1_panel"
    "m1_console"
    "ros2 run m1_control"
)

# control_arbiter_node, wasd_teleop_node, eeg_udp_bridge and
# image_throttle_node deliberately do NOT appear above: they run on the Pi.
# Listing them here was harmless but misleading, and it hid the fact that
# m1_console - the process most likely to be left behind - was missing.

PATTERNS=("${VIEW_PATTERNS[@]}")

status() {
    echo "=== operator-side processes ==="
    local found=0
    for pat in "${VIEW_PATTERNS[@]}" "${EXTRA_PATTERNS[@]}"; do
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
    local n
    n=$(ros2 topic info /map 2>/dev/null | sed -n 's/Publisher count: //p')
    echo "  ${n:-0} publisher(s)"
    if [ -n "${n:-}" ] && [ "$n" -gt 1 ] 2>/dev/null; then
        echo "  WARNING: more than one mapper is publishing /map."
        echo "           Run ./kill_vm.sh, then start the view again ONCE."
    fi
}

case "${1:-}" in
    --status)
        status
        exit 0
        ;;
    --all|--hard)
        PATTERNS=("${VIEW_PATTERNS[@]}" "${EXTRA_PATTERNS[@]}")
        ;;
    "")
        ;;
    *)
        echo "unknown argument: $1" >&2
        echo "usage: $0 [--all|--status]" >&2
        exit 2
        ;;
esac

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
