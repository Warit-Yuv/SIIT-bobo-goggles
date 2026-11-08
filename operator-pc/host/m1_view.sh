#!/usr/bin/env bash
# Launch the operator VIEW stack (SLAM + RViz + gaze HUD) with a known-good env.
#
# Do NOT rely on `bash -lc` for this. Ubuntu's ~/.bashrc returns early for
# non-interactive shells:
#
#     case $- in *i*) ;; *) return;; esac
#
# so a login-but-not-interactive shell never reaches the `source .../setup.bash`
# lines further down. `bash -lc` therefore inherits whatever AMENT_PREFIX_PATH
# its parent happened to have, which is stale if a package was built after the
# parent started.
#
# Symptom: RViz shows the RobotModel in ERROR with "Errors loading
# geometries..." and every TF frame is OK. The URDF is fine and the meshes are
# installed; RViz simply cannot resolve package://yahboomcar_description because
# that package is not on its AMENT_PREFIX_PATH.
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
fi

# ROS setup scripts read unset variables, so `set -u` must be off while
# sourcing them or the shell aborts with AMENT_TRACE_SETUP_FILES: unbound.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for ws in "$HOME/m1_control_ws" "$HOME/yahboomcar_ws"; do
    if [ -f "$ws/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$ws/install/setup.bash"
    fi
done
set -u

# Fail loudly rather than starting an RViz that cannot draw the robot.
if ! python3 -c "
from ament_index_python.packages import get_package_share_directory as g
import os, sys
d = g('yahboomcar_description')
p = os.path.join(d, 'meshes/M1USBMecanum/base_link.STL')
sys.exit(0 if os.path.exists(p) else 1)
" 2>/dev/null; then
    echo "ERROR: yahboomcar_description meshes are not resolvable." >&2
    echo "  RViz would start with the RobotModel in ERROR and no robot shown." >&2
    echo "  Build it first:" >&2
    echo "    cd ~/m1_control_ws && colcon build --packages-select yahboomcar_description" >&2
    exit 1
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "description: $(python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g('yahboomcar_description'))" 2>/dev/null)"
exec flock -n /tmp/m1_operator.lock \
    ros2 launch m1_control m1_operator.launch.py "$@"
