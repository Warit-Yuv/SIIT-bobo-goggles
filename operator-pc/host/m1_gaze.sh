#!/usr/bin/env bash
# Start the gaze HUD on the operator PC, with mediapipe available.
#
#   ./m1_gaze.sh                # mediapipe (real eye tracking)
#   ./m1_gaze.sh mouse          # click the HUD to aim, no dependencies
#   ./m1_gaze.sh external       # wait for /gaze/point from elsewhere
#
# WHY THIS WRAPPER EXISTS
# -----------------------
# mediapipe is installed into ~/gaze-libs rather than system-wide, and it MUST
# stay that way: a normal `pip install mediapipe` pulls numpy 2.x, which breaks
# every ROS 2 Python node on the machine (rclpy is built against numpy 1.x).
# Installing to a separate directory and putting it on PYTHONPATH for this one
# process keeps the two apart.
#
#   ~/gaze-libs          mediapipe 0.10.14 + numpy 1.26.4 (isolated)
#   /usr/lib/python3/... ROS 2 Humble + rclpy (system, untouched)
#
# The alternative - a venv - needs python3-venv, which is now installed, but a
# --target directory is simpler to reason about and cannot shadow ROS packages.
set -u

SOURCE_KIND="${1:-mediapipe}"
shift 2>/dev/null || true

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
if [ -f "$HOME/m1_control_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$HOME/m1_control_ws/install/setup.bash"
fi
set -u

# gaze-libs goes FIRST so mediapipe wins over any system copy, but after the
# ROS overlay so ROS's own packages are not shadowed. Order matters: putting it
# before ROS would let numpy 1.26.4 from gaze-libs override a newer ROS numpy.
export PYTHONPATH="$HOME/gaze-libs${PYTHONPATH:+:$PYTHONPATH}"

# The HUD needs a real display - it opens an OpenCV window with imshow.
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

if [ "$SOURCE_KIND" = "mediapipe" ]; then
    if ! python3 -c "import mediapipe" 2>/dev/null; then
        echo "ERROR: mediapipe not importable from ~/gaze-libs" >&2
        echo "  install it with:" >&2
        echo "    pip3 install --target ~/gaze-libs 'mediapipe==0.10.14' 'numpy<1.27'" >&2
        exit 1
    fi
    python3 -c "import mediapipe, numpy; print('mediapipe', mediapipe.__version__, '| numpy', numpy.__version__)"
fi

echo "gaze source : $SOURCE_KIND"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "Press Ctrl+C to stop. The HUD window shows the robot camera with the gaze point."
exec ros2 run m1_control gaze_hud_node --ros-args \
    -p gaze_source:="$SOURCE_KIND" \
    -p assert_mode:=false \
    "$@"
