#!/usr/bin/env bash
# Hard reset of the ROS graph inside the container.
#
#   /root/m1_control_ws/scripts/kill_all.sh
#
# Why a script instead of an inline pkill: an inline command line like
#   docker exec ... bash -lc 'pkill -f "ros2 launch"'
# contains the very string it is searching for, so pkill matches (and kills)
# its own shell. Keeping the patterns in a file avoids that self-match.
#
# The match is by executable path/name, never by a generic word, so this script
# cannot kill itself or an unrelated process.

set -uo pipefail

PATTERNS=(
    "/ros2 launch"
    "ydlidar_ros2_driver_node"
    "Mcnamu_driver_M1"
    "base_node_M1"
    "usb_cam_node_exe"
    "image_throttle_node"
    "control_arbiter_node"
    "wasd_teleop_node"
    "gaze_hud_node"
    "eeg_udp_bridge"
    "ekf_filter_node"
    "imu_filter_madgwick_node"
    "robot_state_publisher"
    "joint_state_publisher"
    "static_transform_publisher"
    "slam_toolbox"
    "rviz2"
)

SELF=$$
for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
        [ "$pid" = "$SELF" ] && continue
        kill -9 "$pid" 2>/dev/null || true
    done
done

sleep 2

LEFT=$(pgrep -f "ros2|ydlidar|Mcnamu|usb_cam|throttle_node" 2>/dev/null \
       | grep -v "$$" || true)

REAL=0
ZOMB=0
for pid in $LEFT; do
    cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)

    # The ROS 2 CLI daemon is a background discovery cache, not a node. It is
    # meant to outlive everything and killing it only forces a slow rebuild, so
    # skip it rather than reporting it as a survivor.
    case "$cmdline" in
        *ros2cli.daemon.daemonize*|*ros2-daemon*) continue ;;
    esac

    # Zombies (state Z) are already dead and hold no resources; they only wait
    # for their parent to reap them. With --init on the container (tini as
    # PID 1) they are reaped automatically. Counting them separately keeps a
    # real leftover from being confused with harmless residue.
    state=$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)
    if [ "$state" = "Z" ]; then
        ZOMB=$((ZOMB + 1))
    else
        REAL=$((REAL + 1))
        printf '[kill_all] still running: %s %s\n' "$pid" "${cmdline:0:70}"
    fi
done

if [ "$REAL" -eq 0 ]; then
    echo "[kill_all] ROS graph is clean (${ZOMB} unreaped zombie(s))"
else
    echo "[kill_all] ${REAL} process(es) survived SIGKILL - investigate"
fi
