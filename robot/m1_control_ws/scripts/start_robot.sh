#!/usr/bin/env bash
# =============================================================================
# start_robot.sh - full robot stack (hardware + control), run INSIDE the container
#
#   /root/m1_control_ws/scripts/start_robot.sh              # normal start
#   .../start_robot.sh --lidar false                        # no LiDAR
#   .../start_robot.sh --camera false --no-control          # driver only
#   .../start_robot.sh --dry                               # print, don't launch
#   .../start_robot.sh --force                             # start despite a
#                                                          #   running stack
#
# Runs two launch files in one process group:
#   m1_robot_bringup.launch.py   hardware: driver, odom, EKF, camera, LiDAR
#   m1_control.launch.py         arbiter + EEG bridge
#
# Ctrl+C stops both. The arbiter publishes a final zero Twist on shutdown.
#
# SINGLETON: only one stack may run at a time. Starting a second one does not
# merely waste CPU - it corrupts the hardware links, because:
#   * several drivers reading /dev/rplidar interleave each other's bytes,
#     producing "Check Sum 0xC5A3 != 0x2191" and a LiDAR that reports no data;
#   * only one process can own /dev/video0, so the extras abort in usb_cam
#     with "terminate called after throwing an instance of 'char*'";
#   * the duplicated EKF/IMU nodes miss their update rate.
# A flock-based lock makes a second start fail fast instead of doing that.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

LIDAR="auto"
CAMERA="true"
CONTROL="true"
THROTTLE="20.0"
DRY=0
FORCE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar)      LIDAR="$2"; shift 2 ;;
        --camera)     CAMERA="$2"; shift 2 ;;
        --throttle)   THROTTLE="$2"; shift 2 ;;
        --no-control) CONTROL="false"; shift ;;
        --dry)        DRY=1; shift ;;
        --force)      FORCE=1; shift ;;
        -h|--help)    sed -n '2,26p' "$0"; exit 0 ;;
        *)            echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

# ---------------------------------------------------------------------------
# Singleton guard
# ---------------------------------------------------------------------------
LOCK_FILE=/tmp/m1_robot_stack.lock

# Show what is already running, so the message is actionable.
existing_launches() {
    pgrep -af "ros2 launch m1_control" 2>/dev/null | grep -v "pgrep" || true
}

if [ "$DRY" != "1" ] && [ "$FORCE" != "1" ]; then
    exec 9>"$LOCK_FILE"
    if ! flock -n 9; then
        echo "========================================================" >&2
        echo " REFUSING TO START: a robot stack is already running." >&2
        echo "========================================================" >&2
        existing_launches | sed 's/^/  /' >&2
        echo >&2
        echo " Starting a second stack corrupts /dev/rplidar and /dev/video0:" >&2
        echo " several drivers reading one serial port interleave bytes and" >&2
        echo " produce checksum errors, and only one process can own the camera." >&2
        echo >&2
        echo "   stop it (in here):  ${SCRIPT_DIR}/stop_all.sh --hard" >&2
        echo "   stop it (on host):  /home/pi/m1.sh stop --hard" >&2
        echo "   or force:           $0 --force" >&2
        exit 1
    fi
    # Lock fd 9 is held for the lifetime of this script and released on exit.
fi

echo "========================================================"
echo " ROSMASTER M1 - robot stack"
echo "   lidar   : ${LIDAR}"
echo "   camera  : ${CAMERA} (throttle ${THROTTLE} Hz)"
echo "   control : ${CONTROL}"
echo "========================================================"

if [ "$DRY" = "1" ]; then
    echo "[dry] would run:"
    echo "  ros2 launch m1_control m1_robot_bringup.launch.py lidar:=${LIDAR} camera:=${CAMERA} throttle_rate:=${THROTTLE}"
    [ "$CONTROL" = "true" ] && echo "  ros2 launch m1_control m1_control.launch.py"
    exit 0
fi

# Bringup in the background, control in the foreground so Ctrl+C lands on the
# arbiter (it publishes a zero Twist on shutdown).
ros2 launch m1_control m1_robot_bringup.launch.py \
    lidar:="${LIDAR}" camera:="${CAMERA}" throttle_rate:="${THROTTLE}" &
BRINGUP_PID=$!

cleanup() {
    echo
    echo "[start_robot] shutting down..."
    kill -INT "${BRINGUP_PID}" 2>/dev/null || true
    wait "${BRINGUP_PID}" 2>/dev/null || true
    echo "[start_robot] done."
}
trap cleanup EXIT INT TERM

sleep 3

if [ "$CONTROL" = "true" ]; then
    ros2 launch m1_control m1_control.launch.py
else
    echo "[start_robot] control disabled - bringup only. Ctrl+C to stop."
    wait "${BRINGUP_PID}"
fi
