#!/usr/bin/env bash
# =============================================================================
# start_robot.sh - full robot stack (hardware + control), run INSIDE the container
#
#   /root/m1_control_ws/scripts/start_robot.sh              # normal start
#   .../start_robot.sh --lidar false                        # no LiDAR
#   .../start_robot.sh --camera false --no-control          # driver only
#   .../start_robot.sh --dry                               # print, don't launch
#
# Runs two launch files in one process group:
#   m1_robot_bringup.launch.py   hardware: driver, odom, EKF, camera, LiDAR
#   m1_control.launch.py         arbiter + EEG bridge
#
# Ctrl+C stops both. The arbiter publishes a final zero Twist on shutdown.
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

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar)      LIDAR="$2"; shift 2 ;;
        --camera)     CAMERA="$2"; shift 2 ;;
        --throttle)   THROTTLE="$2"; shift 2 ;;
        --no-control) CONTROL="false"; shift ;;
        --dry)        DRY=1; shift ;;
        -h|--help)    sed -n '2,20p' "$0"; exit 0 ;;
        *)            echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

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
