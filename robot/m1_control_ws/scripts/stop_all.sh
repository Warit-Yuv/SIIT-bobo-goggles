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

PATTERNS="control_arbiter_node wasd_teleop_node gaze_hud_node eeg_udp_bridge image_throttle_node gaze_control_node gaze_udp_bridge sensor_sync_node"
if [ "$HARD" = "1" ]; then
    # NOTE: the EKF binary is `ekf_node`, not `ekf_filter_node` - the latter is
    # only its ROS node name. Matching the wrong string left orphaned EKF
    # processes behind on every stop (4 had accumulated before this was fixed).
    PATTERNS="${PATTERNS} yahboom_joy_M1 Mcnamu_driver_M1 base_node_M1 ydlidar_ros2_driver_node usb_cam_node_exe ekf_node imu_filter_madgwick_node robot_state_publisher joint_state_publisher static_transform_publisher"
    echo "[stop] hard mode: also killing vendor/app nodes"
fi

# ---------------------------------------------------------------------------
# 1. Kill the `ros2 launch` PARENTS first.
#
# This order matters and is the reason a previous version of this script could
# not stop the robot. `ros2 launch` respawns its children, and the LiDAR node
# is launched with respawn=True, so killing children first is undone
# immediately. Worse, the launch parent is the process that holds the
# singleton lock (fd 9 -> /tmp/m1_robot_stack.lock) in start_robot.sh. Leaving
# it alive kept the lock held, so the next `m1.sh start` was refused with
# "a robot stack is already running" even though the nodes looked dead.
#
# SIGINT first: `ros2 launch` shuts its children down cleanly on SIGINT, which
# also lets the arbiter publish its final zero Twist.
# ---------------------------------------------------------------------------
launch_pids() {
    # Match the ros2 launch executable, never this script ($$) or the shell
    # that invoked it (PPID).
    pgrep -f "bin/ros2 launch" 2>/dev/null | grep -vx -e "$$" -e "$PPID" || true
}

# The supervisor holds the singleton lock, so it must go too. Older versions of
# start_robot.sh let children inherit the lock fd, which meant a stuck lock
# even after every node was dead; killing the supervisor covers both the old
# and the new behaviour.
supervisor_pids() {
    pgrep -f "scripts/start_robot.sh" 2>/dev/null \
        | grep -vx -e "$$" -e "$PPID" || true
}

SUP=$(supervisor_pids)
if [ -n "$SUP" ]; then
    echo "[stop] stopping stack supervisor: $(echo "$SUP" | tr '\n' ' ')"
    # SIGINT lets its EXIT trap bring the bringup launch down cleanly.
    # shellcheck disable=SC2086
    kill -INT $SUP 2>/dev/null || true
    for _ in $(seq 1 12); do
        [ -z "$(supervisor_pids)" ] && break
        sleep 0.5
    done
    SUP=$(supervisor_pids)
    if [ -n "$SUP" ]; then
        # shellcheck disable=SC2086
        kill -9 $SUP 2>/dev/null || true
        sleep 1
    fi
fi

PIDS=$(launch_pids)
if [ -n "$PIDS" ]; then
    echo "[stop] stopping launch parents: $(echo "$PIDS" | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill -INT $PIDS 2>/dev/null || true

    # Give ros2 launch up to 6 s to bring its children down gracefully.
    for _ in $(seq 1 12); do
        [ -z "$(launch_pids)" ] && break
        sleep 0.5
    done

    PIDS=$(launch_pids)
    if [ -n "$PIDS" ]; then
        echo "[stop] forcing launch parents: $(echo "$PIDS" | tr '\n' ' ')"
        # shellcheck disable=SC2086
        kill -TERM $PIDS 2>/dev/null || true
        sleep 1
        PIDS=$(launch_pids)
        # shellcheck disable=SC2086
        [ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null || true
    fi
else
    echo "[stop] no launch parents running"
fi

# ---------------------------------------------------------------------------
# 2. Kill any node processes that outlived their launch parent.
# ---------------------------------------------------------------------------
for pat in $PATTERNS; do
    # -f would match this script's own argument list; match the executable
    # path instead by requiring a "/" before the pattern in the cmdline.
    if pgrep -f "/${pat}\$|/${pat} " >/dev/null 2>&1; then
        pkill -INT -f "/${pat}\$|/${pat} " 2>/dev/null || true
        sleep 0.2
        pkill -9 -f "/${pat}\$|/${pat} " 2>/dev/null || true
        echo "[stop] killed ${pat}"
    fi
done

# ---------------------------------------------------------------------------
# 3. Report honestly, including whether the lock was released.
# ---------------------------------------------------------------------------
sleep 1
LEFT=$(launch_pids)
echo
if [ -n "$LEFT" ]; then
    echo "[stop] STILL RUNNING (could not stop):"
    echo "$LEFT" | sed 's/^/    /'
    echo "[stop] run:  ${SCRIPT_DIR}/kill_all.sh"
    exit 1
fi

echo "[stop] launch parents: none"

# `ros2 node list` reads a cache held by the ROS daemon, which keeps reporting
# nodes that died minutes ago. Restarting it makes the list truthful - without
# this, a clean stop still looks like it left a dozen nodes behind.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 1

echo "[stop] remaining ROS nodes:"
NODES=$(ros2 node list 2>/dev/null | grep -v "^/rqt_gui" || true)
if [ -z "$NODES" ]; then
    echo "    (none)"
else
    echo "$NODES" | sed 's/^/    /'
    echo "[stop] note: rqt windows are yours, not the robot stack"
fi

# The lock is an flock held on fd 9 by start_robot.sh; it is released the
# moment that process dies. Report it so the next start is not a surprise.
if flock -n /tmp/m1_robot_stack.lock true 2>/dev/null; then
    echo "[stop] singleton lock: free - 'm1.sh start' will work"
else
    echo "[stop] singleton lock: STILL HELD - run ${SCRIPT_DIR}/kill_all.sh"
    exit 1
fi
