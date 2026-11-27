#!/usr/bin/env bash
# acceptance_carry.sh — 确定性仿真里的双臂同步搬运验收。
#
# 起双臂栈 + task:=carry，等待任务写出指标 JSON，检查 success，然后收栈。
#
#   ros2 run litearm_dual_arm acceptance_carry.sh
#   ros2 run litearm_dual_arm acceptance_carry.sh --plan-only
#
# 脚本固定 ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1，与 launch 默认一致。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
    echo "找不到工作区根（需要同时存在 src/ 与 install/setup.bash）：$SCRIPT_DIR" >&2
    exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

PLAN_ONLY="false"
if [ "${1:-}" = "--plan-only" ]; then
    PLAN_ONLY="true"
fi

LOG="${TMPDIR:-/tmp}/litearm_dual_carry_acceptance.log"
OUTPUT="${TMPDIR:-/tmp}/litearm_dual_carry_acceptance.json"

cd "$WS_DIR"
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

rm -f "$LOG" "$OUTPUT"
setsid ros2 launch litearm_dual_arm dual_arm_sim.launch.py \
    use_rviz:=false execute:=true task:=carry task_plan_only:="$PLAN_ONLY" \
    task_output:="$OUTPUT" >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "[carry] 双臂栈已启动 pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID plan_only=$PLAN_ONLY log=$LOG"

_collect_descendants() {
    local pid="$1"
    echo "$pid"
    local child
    for child in $(pgrep -P "$pid" 2>/dev/null); do
        _collect_descendants "$child"
    done
}

cleanup() {
    local pids
    pids="$(_collect_descendants "$LAUNCH_PID" | sort -u | tr '\n' ' ')"
    if [ -n "$pids" ]; then
        # shellcheck disable=SC2086
        kill -TERM $pids 2>/dev/null
        sleep 3
        # shellcheck disable=SC2086
        kill -KILL $pids 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
    echo "[carry] 已收栈"
}
trap cleanup EXIT

deadline=$((SECONDS + 300))
while (( SECONDS < deadline )); do
    if [ -s "$OUTPUT" ]; then
        break
    fi
    if grep -qiE "\[FATAL\]|Caught exception in launch" "$LOG" 2>/dev/null; then
        echo "[carry] 启动失败："
        grep -iE "\[FATAL\]|Caught exception|ERROR" "$LOG" | head -20
        exit 1
    fi
    sleep 1
done

if [ ! -s "$OUTPUT" ]; then
    echo "[carry] 超时未生成指标文件，日志尾部："
    tail -60 "$LOG"
    exit 1
fi

if /usr/bin/python3 - "$OUTPUT" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
transport = payload.get("transport") or {}
execution = payload.get("execution") or {}
print("success:", payload.get("success"))
print("error:", payload.get("error"))
print(
    "transport:",
    {
        "checked": transport.get("checked"),
        "contacts": transport.get("contacts"),
        "invalid_indices": transport.get("invalid_indices"),
        "max_position_error_m": transport.get("max_position_error_m"),
        "max_separation_error_m": transport.get("max_separation_error_m"),
        "passed": transport.get("passed"),
    },
)
print("execution:", execution)
if not payload.get("success"):
    raise SystemExit(1)
PY
then
    echo "[carry] 验收通过"
    exit 0
else
    echo "[carry] 验收失败，日志中的问题项："
    grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" | grep -v "occupancy_map_monitor" | head -20 || true
    exit 1
fi
