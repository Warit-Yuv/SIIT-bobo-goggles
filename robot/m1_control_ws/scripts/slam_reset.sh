#!/usr/bin/env bash
# =============================================================================
# slam_reset.sh - start the SLAM mapper over, without touching anything else.
#
#   ./slam_reset.sh              reset the map, keep the current scan
#   ./slam_reset.sh --save NAME  save the map first, then reset
#   ./slam_reset.sh --status     report mapper state only
#
# ---------------------------------------------------------------------------
# WHY THIS EXISTS
# ---------------------------------------------------------------------------
# slam_toolbox in mapping mode has no idea it has been moved. If the robot is
# picked up and put down somewhere else, the odometry jumps, the next scan is
# matched against a pose that no longer means anything, and the map smears.
#
# This is the "kidnapped robot" problem and it is not a bug - a mapper
# accumulates, it does not re-localise. The honest options are:
#
#   * reset and map again (this script), or
#   * save the map and run in localisation mode against it, or
#   * use AMCL with a saved map.
#
# Humble's slam_toolbox exposes no /reset service, so the only reliable way to
# clear the pose graph is to restart the node. That is what this does.
#
# Killing and restarting is safe here because the operator launch does not use
# respawn - if it did, the node would come straight back with the old map still
# in its head and this would appear to do nothing.
# =============================================================================
set -uo pipefail

SLAM_NODE=/async_slam_toolbox_node
MAP_TOPIC=/map

status() {
    echo "=== mapper state ==="
    if pgrep -f async_slam_toolbox >/dev/null 2>&1; then
        echo "  process : running (pid $(pgrep -f async_slam_toolbox | head -1))"
    else
        echo "  process : not running"
    fi
    local n
    n=$(ros2 topic info "$MAP_TOPIC" 2>/dev/null | sed -n 's/Publisher count: //p')
    echo "  /map    : ${n:-0} publisher(s)"
    if [ -n "${n:-}" ] && [ "$n" -gt 1 ] 2>/dev/null; then
        echo "  WARNING : more than one mapper is publishing /map"
    fi
}

save_map() {
    local name="$1"
    local dir="$HOME/maps"
    mkdir -p "$dir"
    echo "[slam_reset] saving map as $dir/$name"
    # The service writes <name>.pgm and <name>.yaml into the given directory.
    if ros2 service call /slam_toolbox/save_map \
            slam_toolbox/srv/SaveMap "{name: {data: '$dir/$name'}}" \
            >/tmp/slam_save.log 2>&1; then
        echo "[slam_reset] saved:"
        ls -1 "$dir/$name".* 2>/dev/null | sed 's/^/    /'
        return 0
    fi
    echo "[slam_reset] save FAILED - see /tmp/slam_save.log" >&2
    tail -3 /tmp/slam_save.log >&2
    return 1
}

reset_map() {
    local params
    params=$(ros2 pkg prefix m1_control 2>/dev/null)/share/m1_control/config/slam_toolbox_m1.yaml
    if [ ! -f "$params" ]; then
        echo "[slam_reset] cannot find $params" >&2
        return 1
    fi

    echo "[slam_reset] stopping mapper"
    pkill -INT -f async_slam_toolbox 2>/dev/null || true
    for _ in $(seq 1 20); do
        pgrep -f async_slam_toolbox >/dev/null 2>&1 || break
        sleep 0.25
    done
    pkill -9 -f async_slam_toolbox 2>/dev/null || true
    sleep 0.5

    # The ROS daemon caches node lists; without this, "ros2 node list" keeps
    # reporting the mapper that was just killed and it looks like nothing
    # happened.
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true

    echo "[slam_reset] starting fresh mapper"
    nohup ros2 run slam_toolbox async_slam_toolbox_node \
        --ros-args --params-file "$params" \
        >/tmp/slam_reset.log 2>&1 &
    sleep 4

    if pgrep -f async_slam_toolbox >/dev/null 2>&1; then
        echo "[slam_reset] mapper restarted with an empty pose graph"
        return 0
    fi
    echo "[slam_reset] restart FAILED - see /tmp/slam_reset.log" >&2
    tail -5 /tmp/slam_reset.log >&2
    return 1
}

case "${1:-}" in
    --status)
        status
        exit 0
        ;;
    --save)
        save_map "${2:-map_$(date +%Y%m%d_%H%M%S)}" || exit 1
        reset_map || exit 1
        ;;
    "")
        reset_map || exit 1
        ;;
    *)
        echo "unknown argument: $1" >&2
        echo "usage: $0 [--save NAME|--status]" >&2
        exit 2
        ;;
esac

echo
status
