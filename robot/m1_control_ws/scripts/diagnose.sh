#!/usr/bin/env bash
# =============================================================================
# diagnose.sh - health check for the ROSMASTER M1. Run inside the container.
#
#   /root/m1_control_ws/scripts/diagnose.sh
#
# Read-only: it never publishes motion. Safe to run at any time.
# Exit code = number of failed checks.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M1_QUIET=1 source "${SCRIPT_DIR}/env.sh"

FAIL=0
WARN=0

ok()   { printf '  \033[32m[ OK ]\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m[FAIL]\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
warn() { printf '  \033[33m[WARN]\033[0m %s\n' "$*"; WARN=$((WARN+1)); }
head_() { printf '\n\033[1m%s\033[0m\n' "$*"; }

head_ "1. Environment"
[ "${ROS_DOMAIN_ID}" = "30" ] && ok "ROS_DOMAIN_ID=30" || bad "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} (expected 30)"
[ -n "${ROS_DISTRO:-}" ] && ok "ROS distro ${ROS_DISTRO}" || bad "ROS not sourced"
[ -f /root/m1_control_ws/install/setup.bash ] && ok "overlay installed" || bad "overlay not built (run colcon build)"

head_ "2. Devices"
for dev in /dev/myserial /dev/rplidar /dev/video0; do
    if [ -e "$dev" ]; then
        ok "${dev} -> $(readlink -f "$dev" 2>/dev/null || echo "$dev")"
    else
        case "$dev" in
            /dev/myserial) bad "${dev} MISSING - chassis driver cannot start" ;;
            /dev/rplidar)  warn "${dev} MISSING - LiDAR/SLAM unavailable (rest of robot OK)" ;;
            /dev/video0)   warn "${dev} MISSING - no camera stream" ;;
        esac
    fi
done

head_ "3. Packages"
for pkg in yahboomcar_msgs yahboomcar_bringup yahboomcar_base_node \
           robot_localization usb_cam ydlidar_ros2_driver slam_toolbox \
           m1_control; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        ok "$pkg"
    else
        bad "$pkg not found"
    fi
done

head_ "4. Python dependencies"
python3 - <<'PY'
import importlib, sys
mods = [('cv2', 'opencv'), ('numpy', 'numpy'), ('rclpy', 'rclpy'),
        ('mediapipe', 'mediapipe (optional, gaze on PC)')]
fail = 0
for mod, label in mods:
    try:
        m = importlib.import_module(mod)
        print(f'  \033[32m[ OK ]\033[0m {label} {getattr(m, "__version__", "")}'.rstrip())
    except Exception as exc:
        if 'optional' in label:
            print(f'  \033[33m[WARN]\033[0m {label} missing ({exc})')
        else:
            print(f'  \033[31m[FAIL]\033[0m {label} missing ({exc})')
            fail += 1
sys.exit(fail)
PY
[ $? -ne 0 ] && FAIL=$((FAIL+1))

head_ "5. Live graph"
NODES=$(ros2 node list 2>/dev/null | wc -l)
if [ "$NODES" -gt 0 ]; then
    ok "${NODES} node(s) visible"
    ros2 node list 2>/dev/null | sed 's/^/        /'
else
    warn "no nodes running (start with scripts/start_robot.sh)"
fi

head_ "6. Key topics"
for topic in /cmd_vel /Servo /voltage /scan /image_throttled/compressed; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
        pubs=$(ros2 topic info "$topic" 2>/dev/null | grep -oP 'Publisher count: \K\d+' || echo '?')
        subs=$(ros2 topic info "$topic" 2>/dev/null | grep -oP 'Subscription count: \K\d+' || echo '?')
        ok "${topic} (pub=${pubs} sub=${subs})"
    else
        case "$topic" in
            /cmd_vel) ok "${topic} absent (driver not running yet)" ;;
            *)        warn "${topic} not present" ;;
        esac
    fi
done

head_ "7. Safety invariants"
PUB=$(ros2 topic info /cmd_vel 2>/dev/null | grep -oP 'Publisher count: \K\d+' || echo 0)
if [ "${PUB:-0}" -le 1 ]; then
    ok "/cmd_vel has ${PUB:-0} publisher(s) - arbiter owns it"
else
    bad "/cmd_vel has ${PUB} publishers - a rogue controller is running (pkill -f yahboom_joy_M1)"
fi

head_ "8. Network"
IP=$(hostname -I | awk '{print $1}')
[ -n "$IP" ] && ok "robot IP ${IP}" || bad "no IP address"
if [ -e /dev/rplidar ]; then
    ok "LiDAR device present"
else
    warn "LiDAR absent - reconnects live, restart bringup with lidar:=true"
fi

echo
echo "========================================================"
printf ' result: \033[31m%d failed\033[0m, \033[33m%d warning(s)\033[0m\n' "$FAIL" "$WARN"
echo "========================================================"
exit "$FAIL"
