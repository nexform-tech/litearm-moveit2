#!/usr/bin/env bash
# acceptance_moveit.sh — MoveIt planning pipeline acceptance test (no hardware).
#
#   start move_group (control stack in dry-run) → plan to the ready pose →
#   optionally execute → tear the stack down
#
# Usage:
#   scripts/acceptance_moveit.sh                # plan only (does not drive the arm)
#   scripts/acceptance_moveit.sh --execute      # plan and execute
#   scripts/acceptance_moveit.sh --execute --real   # real robot
#
# Environment caveat (a pitfall we hit): the ROS 2 default domain is 0 and
# multicast discovery is network-wide. If other robots on the same subnet are
# running ROS 2, you will see **multiple /move_action servers**, and the
# planning goal may be taken over by a stranger node's move_group — it does not
# have your controllers, so it returns FAILURE; meanwhile your own move_group is
# executing as well, which shows up as "it reports failure but the arm really
# did move" — an extremely hard symptom to track down.
# So this script forces a dedicated domain ID + localhost only.

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Workspace root: the script may run from the source tree
# (<ws>/src/<pkg>/scripts/) or from the install tree
# (<ws>/install/<pkg>/lib/<pkg>/), which sit at different depths. So walk up
# looking for a directory that holds both src/ and install/, instead of
# deducing it from a fixed number of levels.
_find_ws() {
    local d="$SCRIPT_DIR"
    while [ "$d" != "/" ]; do
        if [ -f "$d/install/setup.bash" ] && [ -d "$d/src" ]; then
            echo "$d"
            return 0
        fi
        d="$(dirname "$d")"
    done
    return 1
}

if ! WS_DIR="$(_find_ws)"; then
    echo "cannot find the workspace root (needs both src/ and install/setup.bash): $SCRIPT_DIR" >&2
    exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

LOG="${TMPDIR:-/tmp}/litearm_acceptance_moveit.log"
PROBE="$SCRIPT_DIR/moveit_probe.py"

cd "$WS_DIR"
set +u
# ROS environment: do not hard-code the distro; locate the install prefix via
# $ROS_DISTRO, falling back to humble. If you have already sourced another
# distro, export ROS_DISTRO explicitly before running this script.
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

PROBE_ARGS=()
DRY="dry_run:=true"
for arg in "$@"; do
    case "$arg" in
        --execute) PROBE_ARGS+=("--execute") ;;
        --real)    DRY="dry_run:=false"
                   echo "[accept] ★ real-robot mode: the arm really moves — clear the space, hold the emergency stop" ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

rm -f "$LOG"
setsid ros2 launch litearm_moveit_config litearm_moveit.launch.py \
    "$DRY" use_rviz:=false >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "[accept] MoveIt stack started pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID log=$LOG"

cleanup() {
    local pgid
    pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
    if [ -n "$pgid" ]; then
        kill -TERM -"$pgid" 2>/dev/null
        sleep 5
        kill -KILL -"$pgid" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
    echo "[accept] stack torn down"
}
trap cleanup EXIT

deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
    if grep -q "You can start planning now" "$LOG" 2>/dev/null; then
        echo "[accept] move_group ready"
        break
    fi
    if grep -qiE "\[FATAL\]|Failed to load|error loading" "$LOG" 2>/dev/null; then
        echo "[accept] move_group failed to start:"
        grep -iE "\[FATAL\]|Failed to load|error" "$LOG" | head -20
        exit 1
    fi
    sleep 1
done
if ! grep -q "You can start planning now" "$LOG" 2>/dev/null; then
    echo "[accept] timed out waiting for readiness, tail of the log:"
    tail -40 "$LOG"
    exit 1
fi

python3 "$PROBE" ${PROBE_ARGS[@]+"${PROBE_ARGS[@]}"}
RC=$?

echo "[accept] ===== problems found in the log ====="
# The octomap line is MoveIt's standard warning when there is no 3D sensor; this
# configuration has no depth camera, so it is expected
grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" | grep -v "occupancy_map_monitor" | head -10 || echo "(none)"
echo "[accept] probe exit code=$RC"
exit $RC
