#!/usr/bin/env bash
# =============================================================================
# tune_network.sh - apply the ROS 2 / DDS network tuning on the Pi 5.
#
#   sudo ./tune_network.sh          apply (idempotent)
#   sudo ./tune_network.sh --check  show current values only
#   sudo ./tune_network.sh --revert remove the managed block
#
# Written as a script because the widely-copied ROS 2 tuning snippets were
# written for a Raspberry Pi 3B+ with 1 GB of RAM. Several of their values are
# actively harmful on this Pi 5 (8 GB), so the reasoning is recorded here.
#
# ---------------------------------------------------------------------------
# WHAT IS SET AND WHY
# ---------------------------------------------------------------------------
#   net.core.rmem_max = 16 MB
#   net.core.wmem_max =  8 MB
#       Ceilings that an application MAY request with setsockopt(SO_RCVBUF).
#       They cost nothing until something asks for them, so they are safe on a
#       busy machine. This is what lets a DDS implementation size its receive
#       buffer large enough to reassemble a fragmented 640x480 JPEG.
#
#   net.core.netdev_max_backlog = 5000
#       Lets the Wi-Fi stack queue a burst of incoming UDP frames instead of
#       dropping them at the interface. 1000 (default) is small for a 20 Hz
#       image stream plus a 10 Hz scan stream plus TF.
#
#   vm.swappiness = 10
#       Keeps ROS nodes resident. At the default 60 this Pi swaps active node
#       memory (VS Code server + Chromium + Jupyter are all resident here) and
#       a control loop that gets paged out can miss the 350 ms watchdog.
#
# ---------------------------------------------------------------------------
# WHAT IS DELIBERATELY *NOT* SET
# ---------------------------------------------------------------------------
#   net.core.rmem_default / wmem_default
#       These apply to EVERY socket, including ssh, apt and the VS Code server
#       tunnel. Setting them to 16 MB reserves memory across ordinary
#       connections and is counter-productive on any machine. The popular
#       tutorial value of 16 MB comes from a 1 GB Pi 3B+ where it was smaller
#       (4 MB) for a reason; neither belongs on a Pi 5. Left at the kernel
#       default (212992).
#
#   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#       CycloneDDS is NOT installed in this image. Exporting it makes every
#       node fail to start. FastDDS is the only RMW present, so the variable is
#       left unset (see scripts/env.sh).
#
#   net.ipv4.ipfrag_high_thresh
#       Already 4 MB by default on this kernel; the fragment-reassembly stall
#       described in the DDS tuning guide is solved here by BEST_EFFORT QoS on
#       sensor topics rather than by growing this global pool.
# =============================================================================
set -euo pipefail

MARK_BEGIN="# === ROSMASTER M1 DDS tuning (managed block) ==="
MARK_END="# === end ROSMASTER M1 DDS tuning ==="
CONF=/etc/sysctl.conf

BLOCK=$(cat <<EOF
${MARK_BEGIN}
# Applied by m1_control_ws/scripts/tune_network.sh - see that file for reasoning.
# Only ceilings are raised; *_default is intentionally left at the kernel value.
net.core.rmem_max=16777216
net.core.wmem_max=8388608
net.core.netdev_max_backlog=5000
vm.swappiness=10
${MARK_END}
EOF
)

KEYS="net.core.rmem_max net.core.wmem_max net.core.netdev_max_backlog vm.swappiness"
DEFAULT_KEYS="net.core.rmem_default net.core.wmem_default"
DEFAULT_VAL=212992

need_root() { [ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }; }

do_check() {
    echo "=== managed tuning values ==="
    for k in $KEYS; do printf '  %-32s %s\n' "$k" "$(sysctl -n "$k" 2>/dev/null)"; done
    echo "=== socket defaults (must stay at kernel default) ==="
    for k in $DEFAULT_KEYS; do printf '  %-32s %s\n' "$k" "$(sysctl -n "$k" 2>/dev/null)"; done
    echo "=== RMW ==="
    printf '  %-32s %s\n' "RMW_IMPLEMENTATION" "${RMW_IMPLEMENTATION:-<unset, FastDDS>}"
    if [ -f "$CONF" ] && grep -qF "$MARK_BEGIN" "$CONF"; then
        echo "=== persisted block: present in $CONF ==="
    else
        echo "=== persisted block: NOT present in $CONF ==="
    fi
}

do_revert() {
    need_root
    cp "$CONF" "${CONF}.bak.$(date +%Y%m%d%H%M%S)"
    # Delete the managed block, including its markers.
    sed -i "/$(printf '%s' "$MARK_BEGIN" | sed 's/[]\/$*.^[]/\\&/g')/,/$(printf '%s' "$MARK_END" | sed 's/[]\/$*.^[]/\\&/g')/d" "$CONF"
    sysctl -w net.core.rmem_default=$DEFAULT_VAL net.core.wmem_default=$DEFAULT_VAL >/dev/null
    echo "reverted; run 'sudo sysctl --system' to reload defaults from the system"
}

do_apply() {
    need_root
    cp "$CONF" "${CONF}.bak.$(date +%Y%m%d%H%M%S)"

    # Replace an existing managed block so repeated runs never duplicate it.
    if grep -qF "$MARK_BEGIN" "$CONF"; then
        sed -i "/$(printf '%s' "$MARK_BEGIN" | sed 's/[]\/$*.^[]/\\&/g')/,/$(printf '%s' "$MARK_END" | sed 's/[]\/$*.^[]/\\&/g')/d" "$CONF"
        echo "replaced existing managed block"
    fi

    printf '\n%s\n' "$BLOCK" >> "$CONF"

    # Guard against the harmful tutorial setting: force the socket defaults back
    # to the kernel default in case a previous run inflated them.
    sysctl -w net.core.rmem_default=$DEFAULT_VAL net.core.wmem_default=$DEFAULT_VAL >/dev/null 2>&1 || true

    sysctl -p >/dev/null
    echo "applied:"
    for k in $KEYS; do printf '  %-32s %s\n' "$k" "$(sysctl -n "$k")"; done
}

case "${1:-apply}" in
    --check)  do_check ;;
    --revert) do_revert ;;
    apply|"") do_apply ;;
    *) echo "usage: $0 [apply|--check|--revert]" >&2; exit 2 ;;
esac
