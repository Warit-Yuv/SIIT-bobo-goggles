#!/usr/bin/env bash
# Launch the M1 operator console with a known-good DDS environment.
#
# Do NOT rely on ~/.bashrc for this: Ubuntu's default .bashrc returns early
# for non-interactive shells, so `bash -lc ...` silently misses
# ROS_DOMAIN_ID and RMW_IMPLEMENTATION. A DDS mismatch does not error - it
# just leaves the GUI showing "no data" forever.
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
fi

# ROS's setup scripts read unset variables, so `set -u` must be off while
# sourcing them or the shell aborts with AMENT_TRACE_SETUP_FILES: unbound.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$HOME/m1_control_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$HOME/m1_control_ws/install/setup.bash"
else
    echo "ERROR: $HOME/m1_control_ws/install/setup.bash missing - build first:" >&2
    echo "  cd ~/m1_control_ws && colcon build --symlink-install --packages-select m1_control" >&2
    exit 1
fi
set -u

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
exec ros2 run m1_control m1_console "$@"
