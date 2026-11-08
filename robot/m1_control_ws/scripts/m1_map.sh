#!/usr/bin/env bash
# =============================================================================
# m1_map.sh - map once, then localise instead of mapping.
#
#   ./m1_map.sh start                  start mapping (slam_toolbox)
#   ./m1_map.sh save NAME              save the current map to ~/maps/NAME
#   ./m1_map.sh list                   list saved maps
#   ./m1_map.sh localize NAME          localise against a saved map (AMCL)
#   ./m1_map.sh relocalize X Y YAW     tell AMCL where the robot is
#   ./m1_map.sh relocalize             same, but uses (0,0,0)
#   ./m1_map.sh reset                  restart the mapper with an empty graph
#   ./m1_map.sh status                 what is running, and where is the robot
#   ./m1_map.sh stop                   stop whichever mode is running
#
# ---------------------------------------------------------------------------
# WHY THERE ARE TWO MODES
# ---------------------------------------------------------------------------
# slam_toolbox in mapping mode accumulates a pose graph. It has no concept of
# having been moved, so picking the robot up and putting it down elsewhere makes
# it match the next scan against a pose that no longer means anything: the map
# smears, then tears.
#
# AMCL localises against a map that is already known. Given the map and a rough
# starting pose it converges on the true pose from the laser scan alone.
#
# So:
#   * first visit to a space   -> start, drive around, save
#   * every visit after that   -> localize, and relocalize whenever the robot
#                                 is picked up, bumped, or shifted
#
# The one thing AMCL cannot do is guess where a kidnapped robot is. That is what
# `relocalize` is for: you supply a rough position, AMCL does the rest.
# =============================================================================
set -uo pipefail

MAPS_DIR="$HOME/maps"
PKG_SHARE=$(ros2 pkg prefix m1_control 2>/dev/null)/share/m1_control
SLAM_PARAMS="$PKG_SHARE/config/slam_toolbox_m1.yaml"

die() { echo "error: $*" >&2; exit 1; }

need_ros() {
    # Every command here talks to the graph, so make the failure explicit
    # rather than letting ros2 print a confusing timeout.
    if ! ros2 pkg prefix m1_control >/dev/null 2>&1; then
        die "ROS environment not sourced. Run: source ~/m1_control_ws/install/setup.bash"
    fi
}

map_running()  { pgrep -f async_slam_toolbox >/dev/null 2>&1; }
amcl_running() { pgrep -f 'amcl' >/dev/null 2>&1; }

cmd_status() {
    echo "=== modes ==="
    map_running  && echo "  mapping  : RUNNING (slam_toolbox)" || echo "  mapping  : off"
    amcl_running && echo "  localise : RUNNING (amcl)"         || echo "  localise : off"

    if map_running && amcl_running; then
        echo "  WARNING  : both are running. They fight over map->odom."
        echo "             Run './m1_map.sh stop' and start one mode only."
    fi

    echo
    echo "=== /map ==="
    local n
    n=$(ros2 topic info /map 2>/dev/null | sed -n 's/Publisher count: //p')
    echo "  publishers: ${n:-0}"

    echo
    echo "=== robot pose ==="
    local pose
    pose=$(timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null \
           | grep -A1 'Translation' | head -2 | tr '\n' ' ')
    if [ -n "$pose" ]; then
        echo "  $pose"
    else
        echo "  unknown (no map -> base_footprint yet)"
        echo "  If localising, supply a starting pose: ./m1_map.sh relocalize X Y YAW"
    fi

    echo
    echo "=== saved maps ==="
    if [ -d "$MAPS_DIR" ] && [ -n "$(ls -A "$MAPS_DIR"/*.yaml 2>/dev/null)" ]; then
        ls -1 "$MAPS_DIR"/*.yaml 2>/dev/null | sed "s|$MAPS_DIR/||;s|\.yaml$||" \
            | sed 's/^/  /'
    else
        echo "  (none yet - ./m1_map.sh save NAME)"
    fi
}

cmd_start() {
    need_ros
    [ -f "$SLAM_PARAMS" ] || die "missing $SLAM_PARAMS"
    if map_running; then
        echo "mapping is already running"; return 0
    fi
    if amcl_running; then
        die "AMCL is running. './m1_map.sh stop' first - both publish map->odom."
    fi
    echo "[m1_map] starting mapper"
    nohup ros2 run slam_toolbox async_slam_toolbox_node \
        --ros-args --params-file "$SLAM_PARAMS" \
        >/tmp/m1_slam.log 2>&1 &
    sleep 4
    map_running && echo "[m1_map] mapping. Drive around, then: ./m1_map.sh save NAME" \
                || { tail -5 /tmp/m1_slam.log >&2; die "mapper failed to start"; }
}

cmd_save() {
    need_ros
    local name="${1:-}"
    [ -n "$name" ] || die "usage: $0 save NAME"
    map_running || die "no mapper running - start mapping first"
    mkdir -p "$MAPS_DIR"
    local target="$MAPS_DIR/$name"

    echo "[m1_map] saving to $target.{pgm,yaml}"
    if ! ros2 service call /slam_toolbox/save_map nav_msgs/srv/SaveMap \
            "{name: {data: '$target'}}" >/tmp/m1_save.log 2>&1; then
        tail -5 /tmp/m1_save.log >&2
        die "save service call failed"
    fi
    if [ ! -f "$target.yaml" ]; then
        tail -5 /tmp/m1_save.log >&2
        die "service returned but $target.yaml was not written"
    fi
    echo "[m1_map] saved:"
    ls -lh "$target".* | awk '{printf "    %-40s %s\n", $9, $5}'
    echo
    echo "  Next time, localise instead of mapping:"
    echo "    ./m1_map.sh stop"
    echo "    ./m1_map.sh localize $name"
}

cmd_list() {
    if [ -d "$MAPS_DIR" ] && [ -n "$(ls -A "$MAPS_DIR"/*.yaml 2>/dev/null)" ]; then
        for f in "$MAPS_DIR"/*.yaml; do
            local base; base=$(basename "$f" .yaml)
            local res; res=$(grep -m1 '^resolution' "$f" 2>/dev/null | awk '{print $2}')
            printf '  %-24s res=%s  %s\n' "$base" "${res:-?}" \
                "$(stat -c '%y' "$f" 2>/dev/null | cut -d. -f1)"
        done
    else
        echo "  (no saved maps in $MAPS_DIR)"
    fi
}

cmd_localize() {
    need_ros
    local name="${1:-}"
    [ -n "$name" ] || die "usage: $0 localize NAME   (see: $0 list)"
    local yaml="$MAPS_DIR/$name.yaml"
    [ -f "$yaml" ] || die "no such map: $yaml"

    if map_running; then
        echo "[m1_map] stopping the mapper first (both publish map->odom)"
        pkill -INT -f async_slam_toolbox 2>/dev/null || true
        sleep 2
        pkill -9 -f async_slam_toolbox 2>/dev/null || true
    fi
    if amcl_running; then
        echo "[m1_map] restarting localisation with $name"
        pkill -INT -f 'amcl' 2>/dev/null || true
        sleep 2
        pkill -9 -f 'amcl' 2>/dev/null || true
        pkill -f 'lifecycle_manager_localization' 2>/dev/null || true
        pkill -f 'nav2_map_server' 2>/dev/null || true
        sleep 1
    fi

    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true

    echo "[m1_map] starting localisation against $name"
    nohup ros2 launch m1_control m1_localize.launch.py \
        map:="$yaml" rviz:=false >/tmp/m1_amcl.log 2>&1 &
    sleep 8

    if amcl_running; then
        echo "[m1_map] AMCL up."
        echo "  Give it a starting pose:  ./m1_map.sh relocalize X Y YAW"
        echo "  or use RViz '2D Pose Estimate'."
    else
        tail -15 /tmp/m1_amcl.log >&2
        die "AMCL failed to start"
    fi
}

cmd_relocalize() {
    need_ros
    amcl_running || die "AMCL is not running - './m1_map.sh localize NAME' first"
    local x="${1:-0}" y="${2:-0}" yaw="${3:-0}"
    # Convert yaw (radians) to a quaternion. AMCL wants the full quaternion.
    local qz qw
    read -r qz qw < <(python3 -c "
import math,sys
y=float(sys.argv[1])
print(math.sin(y/2.0), math.cos(y/2.0))" "$yaw")
    echo "[m1_map] telling AMCL: x=$x y=$y yaw=$yaw rad (qz=$qz qw=$qw)"
    # A covariance of 0.25 means "within about half a metre, but not exact" -
    # which is honest for a human pointing at a map, and lets the particle
    # filter spread out enough to find the true pose.
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map},
          pose: {pose: {position: {x: $x, y: $y, z: 0.0},
                        orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}},
                 covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                              0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" \
        >/dev/null 2>&1
    sleep 3
    echo "[m1_map] pose sent. Check with: ./m1_map.sh status"
}

cmd_reset() {
    need_ros
    map_running || die "no mapper running"
    [ -f "$SLAM_PARAMS" ] || die "missing $SLAM_PARAMS"
    echo "[m1_map] restarting the mapper with an empty pose graph"
    pkill -INT -f async_slam_toolbox 2>/dev/null || true
    sleep 2
    pkill -9 -f async_slam_toolbox 2>/dev/null || true
    sleep 1
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
    nohup ros2 run slam_toolbox async_slam_toolbox_node \
        --ros-args --params-file "$SLAM_PARAMS" >/tmp/m1_slam.log 2>&1 &
    sleep 4
    map_running && echo "[m1_map] fresh mapper running" || die "restart failed"
}

cmd_stop() {
    local stopped=0
    for pat in async_slam_toolbox 'nav2_map_server' 'lifecycle_manager_localization'; do
        if pgrep -f "$pat" >/dev/null 2>&1; then
            pkill -INT -f "$pat" 2>/dev/null || true
            stopped=1
        fi
    done
    # AMCL last: the pattern is broad and would otherwise match its own helpers.
    if pgrep -f 'nav2_amcl/lib/nav2_amcl/amcl' >/dev/null 2>&1; then
        pkill -INT -f 'nav2_amcl/lib/nav2_amcl/amcl' 2>/dev/null || true
        stopped=1
    fi
    sleep 2
    for pat in async_slam_toolbox 'nav2_map_server' 'lifecycle_manager_localization' \
               'nav2_amcl/lib/nav2_amcl/amcl'; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
    [ "$stopped" -eq 1 ] && echo "[m1_map] stopped" || echo "[m1_map] nothing was running"
}

case "${1:-status}" in
    start)       cmd_start ;;
    save)        shift; cmd_save "${1:-}" ;;
    list)        cmd_list ;;
    localize)    shift; cmd_localize "${1:-}" ;;
    relocalize)  shift; cmd_relocalize "${1:-0}" "${2:-0}" "${3:-0}" ;;
    reset)       cmd_reset ;;
    status)      cmd_status ;;
    stop)        cmd_stop ;;
    -h|--help|help)
        sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'
        ;;
    *)
        echo "unknown command: $1" >&2
        echo "usage: $0 {start|save NAME|list|localize NAME|relocalize X Y YAW|reset|status|stop}" >&2
        exit 2
        ;;
esac
