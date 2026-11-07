#!/usr/bin/env bash
# =============================================================================
# m1_docker.sh - create / attach the ROSMASTER M1 ROS 2 Humble container
#
# The container is the "robot runtime". It is recreated from the vendor image
# and mounts everything stateful from the HOST, so destroying it loses nothing:
#
#   /home/pi/m1_control_ws   -> /root/m1_control_ws   (our overlay, git-able)
#   /home/pi/eeg_bridge      -> /root/eeg_bridge      (EEG UDP bridge)
#   /home/pi/share           -> /root/share           (scratch)
#   /dev                     -> /dev                  (LIVE device tree)
#
# Why mount all of /dev instead of single devices:
#   Bind-mounting /dev/myserial and /dev/ttyUSB0 freezes the device list at
#   container-start time. Unplug and replug the LiDAR and the container keeps a
#   dead node forever. Mounting the host /dev tree means udev symlinks
#   (/dev/myserial, /dev/rplidar) appear and disappear live, so a reconnected
#   sensor just works - and a missing one is simply absent, not broken.
#
# Why --init:
#   Without it, PID 1 is `sleep infinity`, which never reaps orphaned children.
#   Every `ros2 launch` that is killed leaves a zombie behind and they pile up
#   over a long session. With tini as PID 1 they are reaped immediately.
#
# Usage:
#   ./m1_docker.sh            create if needed, then attach
#   ./m1_docker.sh up         create if needed, leave running (no attach)
#   ./m1_docker.sh recreate   destroy and recreate from scratch
#   ./m1_docker.sh status     show container + device state
#   ./m1_docker.sh stop       stop (keeps the container)
# =============================================================================
set -euo pipefail

NAME="ros-humble-dev"
IMAGE="192.168.2.51:5000/ros-humble-m1:1.1.1-cam"

WS_HOST="/home/pi/m1_control_ws"
EEG_HOST="/home/pi/eeg_bridge"
SHARE_HOST="/home/pi/share"

log() { printf '\033[36m[m1_docker]\033[0m %s\n' "$*"; }
err() { printf '\033[31m[m1_docker]\033[0m %s\n' "$*" >&2; }

ensure_dirs() {
    mkdir -p "$WS_HOST" "$EEG_HOST" "$SHARE_HOST"
}

create_container() {
    ensure_dirs
    log "Creating container '$NAME' from $IMAGE"
    docker run -d \
        --name "$NAME" \
        --init \
        --net=host \
        --privileged \
        --security-opt apparmor:unconfined \
        --security-opt label=disable \
        --restart unless-stopped \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e PULSE_SERVER="unix:/run/user/1000/pulse/native" \
        -e ALSA_CARD=0 \
        -e ROS_DOMAIN_ID=30 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /dev:/dev \
        -v "${SHARE_HOST}:/root/share" \
        -v "${WS_HOST}:/root/m1_control_ws" \
        -v "${EEG_HOST}:/root/eeg_bridge" \
        "$IMAGE" \
        sleep infinity >/dev/null
    log "Container created (idle - nothing auto-starts)."
}

container_state() {
    docker inspect -f '{{.State.Status}}' "$NAME" 2>/dev/null || echo "missing"
}

do_up() {
    local state
    state="$(container_state)"
    case "$state" in
        running) log "Already running." ;;
        missing) create_container ;;
        *)       log "Starting stopped container..."; docker start "$NAME" >/dev/null ;;
    esac
}

do_attach() {
    do_up
    docker exec -it -w /root "$NAME" /bin/bash
}

do_recreate() {
    if [ "$(container_state)" != "missing" ]; then
        log "Removing existing container (host mounts are safe)..."
        docker rm -f "$NAME" >/dev/null
    fi
    create_container
}

do_status() {
    echo "=== container ==="
    docker ps -a --filter "name=^/${NAME}$" \
        --format 'status={{.Status}}  image={{.Image}}' || true
    echo
    echo "=== host devices ==="
    for d in /dev/myserial /dev/rplidar /dev/video0; do
        if [ -e "$d" ]; then
            printf '  %-16s -> %s\n' "$d" "$(readlink -f "$d")"
        else
            printf '  %-16s MISSING\n' "$d"
        fi
    done
    echo
    echo "=== inside container ==="
    if [ "$(container_state)" = "running" ]; then
        docker exec "$NAME" bash -c '
            for d in /dev/myserial /dev/rplidar /dev/video0; do
                if [ -e "$d" ]; then printf "  %-16s -> %s\n" "$d" "$(readlink -f "$d")";
                else printf "  %-16s MISSING\n" "$d"; fi
            done'
    else
        echo "  (container not running)"
    fi
}

case "${1:-attach}" in
    up)       do_up ;;
    attach|"") do_attach ;;
    recreate) do_recreate ;;
    status)   do_status ;;
    stop)     docker stop "$NAME" >/dev/null && log "Stopped." ;;
    *)        err "unknown command: $1"; exit 2 ;;
esac
