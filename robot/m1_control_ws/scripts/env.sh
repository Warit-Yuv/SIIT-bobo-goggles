#!/usr/bin/env bash
# =============================================================================
# env.sh - canonical ROS 2 environment for the ROSMASTER M1.
#
# Source this, never hand-type the source chain:
#     source /root/m1_control_ws/scripts/env.sh
#
# Sourcing order matters: base ROS -> driver underlay -> chassis underlay ->
# our overlay. Later entries win, which is why the overlay comes last.
# =============================================================================

# Unique domain: keeps this robot isolated from other ROS 2 traffic on the LAN.
export ROS_DOMAIN_ID=30

# Hardware identifiers consumed by Yahboom's launch files.
export ROBOT_TYPE=M1
export RPLIDAR_TYPE=tmini          # c1 | tmini
export CAMERA_TYPE=usb             # usb | nuwa

# PTZ rest pose verified on this robot (s1=pan, s2=tilt).
export INIT_SERVO_S1=40
export INIT_SERVO_S2=90

# ---------------------------------------------------------------------------
# ROS 2 setup files are NOT nounset-safe.
#
# /opt/ros/humble/setup.bash line 8 is:
#     if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then
# with no default, so sourcing it under `set -u` aborts with
#     AMENT_TRACE_SETUP_FILES: unbound variable
# and every script that sets `set -u` (start_robot.sh, diagnose.sh,
# stop_all.sh, kill_all.sh) dies before doing anything.
#
# Rather than weaken those scripts - `set -u` catches real typos in our own
# code - nounset is disabled only around the sourcing chain and then restored
# to whatever the caller had.
# ---------------------------------------------------------------------------
_m1_restore_nounset=0
case "$-" in
    *u*) _m1_restore_nounset=1; set +u ;;
esac

# Layer 1 - core ROS 2 Humble
source /opt/ros/humble/setup.bash

# Layer 2 - third-party drivers (usb_cam, ydlidar, robot_localization, slam)
if [ -f /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then
    source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
fi

# Layer 3 - Yahboom chassis apps and messages
if [ -f /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then
    source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
fi

# Layer 4 - our overlay
if [ -f /root/m1_control_ws/install/setup.bash ]; then
    source /root/m1_control_ws/install/setup.bash
fi

# Restore the caller's nounset setting.
if [ "$_m1_restore_nounset" = "1" ]; then
    set -u
fi
unset _m1_restore_nounset

export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

# ---------------------------------------------------------------------------
# DDS middleware: CycloneDDS on BOTH machines.
#
# This is not a preference - the two machines must use the SAME RMW
# implementation. /tf_static is published with TRANSIENT_LOCAL durability and
# that QoS is not reliably delivered across different DDS vendors. Measured
# with the Pi on FastDDS and the operator PC on CycloneDDS:
#
#     /tf        20.05 Hz   (dynamic transforms arrive)
#     /tf_static never      (base_link -> laser missing)
#     /map       never      (slam_toolbox cannot transform, silently does nothing)
#
# Both machines have rmw_cyclonedds_cpp installed, so CycloneDDS is used on
# both. It is also the better choice over Wi-Fi: lower latency and fewer
# dropped frames on fragmented image traffic.
# ---------------------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Socket buffer tuning for Cyclone. Only set when the file actually exists so a
# missing config can never stop the stack from starting.
if [ -f /root/m1_control_ws/config/cyclonedds.xml ]; then
    export CYCLONEDDS_URI=file:///root/m1_control_ws/config/cyclonedds.xml
fi

if [ "${M1_QUIET:-0}" != "1" ]; then
    echo "--------------------------------------------------------"
    echo -e "ROS_DOMAIN_ID: \033[32m${ROS_DOMAIN_ID}\033[0m | ROS: \033[34m${ROS_DISTRO:-none}\033[0m"
    echo -e "robot: \033[32m${ROBOT_TYPE}\033[0m | lidar: \033[32m${RPLIDAR_TYPE}\033[0m | camera: \033[32m${CAMERA_TYPE}\033[0m"
    echo -e "overlay: \033[32m$( [ -f /root/m1_control_ws/install/setup.bash ] && echo loaded || echo MISSING )\033[0m"
    echo "--------------------------------------------------------"
fi
