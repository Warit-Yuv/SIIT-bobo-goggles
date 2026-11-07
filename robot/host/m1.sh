#!/usr/bin/env bash
# =============================================================================
# m1.sh - host-side control panel for the ROSMASTER M1 (run over SSH on the Pi)
#
#   ./m1.sh start          start the full robot stack in the foreground
#   ./m1.sh bg             start it in the background, log to logs/robot.log
#   ./m1.sh stop           stop our nodes (sends a zero velocity first)
#   ./m1.sh stop --hard    also stop vendor nodes
#   ./m1.sh diag           run the health check
#   ./m1.sh status         quick topic/node snapshot
#   ./m1.sh shell          open a shell in the container
#   ./m1.sh teleop         keyboard teleoperation (foreground)
#   ./m1.sh test-eeg       simulate the EEG laptop against the local bridge
#   ./m1.sh logs           tail the background log
#   ./m1.sh devices        show device + container state
#   ./m1.sh build          rebuild the m1_control overlay (+ fix ownership)
#   ./m1.sh fix-perms      return workspace ownership to pi after a build
#
# Everything runs inside the container via docker exec; the host stays clean.
# =============================================================================
set -uo pipefail

NAME="ros-humble-dev"
WS_HOST="/home/pi/m1_control_ws"
WS_CTR="/root/m1_control_ws"
LOGDIR="${WS_HOST}/logs"
LOG="${LOGDIR}/robot.log"

c_ok()  { printf '\033[32m%s\033[0m\n' "$*"; }
c_err() { printf '\033[31m%s\033[0m\n' "$*" >&2; }

ensure_container() {
    if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
        c_err "[m1] container '$NAME' is not running - starting it..."
        /home/pi/m1_docker.sh up || { c_err "[m1] could not start container"; exit 1; }
    fi
}

in_ctr() {
    docker exec -it -w /root "$NAME" bash -lc "$*"
}

in_ctr_nc() {
    docker exec -w /root "$NAME" bash -lc "$*"
}

case "${1:-help}" in

    start)
        ensure_container
        exec docker exec -it -w /root "$NAME" \
            bash -lc "${WS_CTR}/scripts/start_robot.sh ${2:-}"
        ;;

    bg)
        ensure_container
        mkdir -p "$LOGDIR"
        c_ok "[m1] starting in background -> $LOG"
        docker exec -d -w /root "$NAME" bash -lc \
            "exec ${WS_CTR}/scripts/start_robot.sh ${2:-} > ${WS_CTR}/logs/robot.log 2>&1"
        sleep 4
        tail -n 20 "$LOG" 2>/dev/null || true
        echo
        c_ok "[m1] running. './m1.sh logs' to follow, './m1.sh stop' to stop."
        ;;

    stop)
        ensure_container
        # stop_all.sh now stops the launch parents too, and returns non-zero
        # if anything survived. Fall back to the hard reset so a single
        # command always gets you back to a startable state.
        if ! in_ctr_nc "${WS_CTR}/scripts/stop_all.sh ${2:-}"; then
            c_err "[m1] stop_all could not stop everything - running kill_all"
            in_ctr_nc "${WS_CTR}/scripts/kill_all.sh"
        fi
        ;;

    diag|diagnose)
        ensure_container
        in_ctr_nc "${WS_CTR}/scripts/diagnose.sh"
        ;;

    status)
        ensure_container
        in_ctr_nc "source ${WS_CTR}/scripts/env.sh >/dev/null 2>&1; \
            echo '--- nodes ---'; ros2 node list 2>/dev/null; \
            echo; echo '--- topics ---'; ros2 topic list 2>/dev/null"
        ;;

    shell)
        ensure_container
        exec docker exec -it -w /root "$NAME" /bin/bash
        ;;

    teleop)
        ensure_container
        exec docker exec -it -w /root "$NAME" bash -lc \
            "source ${WS_CTR}/scripts/env.sh && ros2 run m1_control wasd_teleop_node"
        ;;

    test-eeg)
        ensure_container
        in_ctr "python3 ${WS_CTR}/scripts/test_eeg_udp.py ${2:---ip 127.0.0.1} ${3:-}"
        ;;

    logs)
        mkdir -p "$LOGDIR"
        tail -f "$LOG"
        ;;

    devices)
        /home/pi/m1_docker.sh status
        ;;

    build)
        ensure_container
        in_ctr_nc "source /opt/ros/humble/setup.bash && cd ${WS_CTR} && \
            colcon build --symlink-install --packages-select m1_control"
        # The container runs as root, so build artefacts land root-owned and
        # then block editing from the host. Hand them back to pi.
        sudo chown -R pi:pi "${WS_HOST}/install" "${WS_HOST}/build" "${WS_HOST}/log" 2>/dev/null
        c_ok "[m1] build finished, ownership fixed"
        ;;

    fix-perms)
        sudo chown -R pi:pi "$WS_HOST" 2>/dev/null
        c_ok "[m1] ownership of $WS_HOST returned to pi"
        ;;

    help|*)
        sed -n '2,20p' "$0"
        ;;
esac
