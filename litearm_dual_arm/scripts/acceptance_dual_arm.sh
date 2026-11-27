#!/usr/bin/env bash
# acceptance_dual_arm.sh — 第一步双臂规划链路验收（无硬件、无执行）。
#
# 起离线双臂栈（move_group + 状态发布 + 静态场景）→ 依次规划
# both_ready / both_safe / left_safe / right_safe → 检查 JSON 的 success → 收栈。
#
# 用法:
#   scripts/acceptance_dual_arm.sh
#   scripts/acceptance_dual_arm.sh --scene none
#   TARGETS="both_safe left_safe" scripts/acceptance_dual_arm.sh
#
# 注意：脚本固定 ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1，与 launch 的默认值
# 一致；在另一个终端手动跑 ros2 命令时也要 export 同样的值。

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

SCENE="dual_static"
if [ "${1:-}" = "--scene" ] && [ -n "${2:-}" ]; then
    SCENE="$2"
fi

TARGETS="${TARGETS:-both_ready both_safe left_safe right_safe}"
LOG="${TMPDIR:-/tmp}/litearm_dual_arm_acceptance.log"
OUT_DIR="${TMPDIR:-/tmp}/litearm_dual_arm_acceptance"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

rm -f "$LOG"
setsid ros2 launch litearm_dual_arm dual_arm_sim.launch.py \
    use_rviz:=false scene:="$SCENE" execute:=true >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "[accept] 双臂栈已启动 pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID scene=$SCENE log=$LOG"

_collect_descendants() {
    local pid="$1"
    echo "$pid"
    local child
    for child in $(pgrep -P "$pid" 2>/dev/null); do
        _collect_descendants "$child"
    done
}

cleanup() {
    # ros2 launch 的子进程有时会换进程组；先按父子关系收集整棵树，再统一收掉。
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
    echo "[accept] 已收栈"
}
trap cleanup EXIT

deadline=$((SECONDS + 180))
while (( SECONDS < deadline )); do
    if grep -q "You can start planning now" "$LOG" 2>/dev/null; then
        if [ "$SCENE" = "none" ] || grep -q "applied scene" "$LOG" 2>/dev/null; then
            echo "[accept] move_group 与场景就绪"
            break
        fi
    fi
    if grep -qiE "\[FATAL\]|Failed to load|Caught exception in launch" "$LOG" 2>/dev/null; then
        echo "[accept] 启动失败："
        grep -iE "\[FATAL\]|Failed to load|ERROR|Caught exception" "$LOG" | head -20
        exit 1
    fi
    sleep 1
done
if ! grep -q "You can start planning now" "$LOG" 2>/dev/null; then
    echo "[accept] 超时未就绪，日志尾部："
    tail -60 "$LOG"
    exit 1
fi

RC=0
for target in $TARGETS; do
    output="$OUT_DIR/$target.json"
    echo "[accept] ===== $target ====="
    if ! ros2 service call /litearm_dual_sim/reset \
        std_srvs/srv/Trigger "{}" >/dev/null 2>&1; then
        echo "[accept] reset service failed"
        RC=1
        continue
    fi
    sleep 1
    if ! ros2 run litearm_dual_arm dual_arm_probe.py \
        --target "$target" \
        --samples 7 \
        --planning-time 10.0 \
        --execute \
        --output "$output"; then
        RC=1
        continue
    fi
    if ! /usr/bin/python3 - "$output" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
if not payload.get("success"):
    print("probe did not succeed:", payload.get("error") or payload.get("planning"))
    raise SystemExit(1)
print("ok:", payload.get("group"), "samples:", payload.get("validation", {}).get("checked"))
PY
    then
        RC=1
    fi
done

echo "[accept] ===== both_ready via /move_action ====="
move_output="$OUT_DIR/both_ready_move_action.json"
if ! ros2 service call /litearm_dual_sim/reset \
    std_srvs/srv/Trigger "{}" >/dev/null 2>&1; then
    echo "[accept] reset service failed"
    RC=1
else
    sleep 1
    if ! ros2 run litearm_dual_arm dual_arm_probe.py \
        --target both_ready \
        --via-move-action \
        --planning-time 10.0 \
        --output "$move_output"; then
        RC=1
    elif ! /usr/bin/python3 - "$move_output" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
execution = payload.get("execution") or {}
if not payload.get("success") or not execution.get("success"):
    print("MoveGroup execution failed:", payload)
    raise SystemExit(1)
print("ok: MoveGroup execution, points:", execution.get("trajectory_points"))
PY
    then
        RC=1
    fi
fi

echo "[accept] ===== mutual_collision_case（预期无效）====="
collision_output="$OUT_DIR/mutual_collision_case.json"
if ! ros2 run litearm_dual_arm dual_arm_probe.py \
    --target mutual_collision_case \
    --check-state \
    --output "$collision_output"; then
    RC=1
else
    if ! /usr/bin/python3 - "$collision_output" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
if not payload.get("success"):
    print("collision case was not rejected:", payload)
    raise SystemExit(1)
if not payload.get("cross_arm_contacts"):
    print("collision case has no cross-arm contacts:", payload)
    raise SystemExit(1)
print("ok: state invalid, cross-arm contacts:", len(payload["cross_arm_contacts"]))
PY
    then
        RC=1
    fi
fi

echo "[accept] ===== fail-closed stop ====="
stop_output="$OUT_DIR/fail_closed_stop.json"
if ! ros2 service call /litearm_dual_sim/reset \
    std_srvs/srv/Trigger "{}" >/dev/null 2>&1; then
    echo "[accept] reset service failed"
    RC=1
elif ! ros2 service call /litearm_dual_arm/stop \
    std_srvs/srv/Trigger "{}" >/dev/null 2>&1; then
    echo "[accept] stop service failed"
    RC=1
else
    sleep 1
    if ros2 run litearm_dual_arm dual_arm_probe.py \
        --target both_ready \
        --execute \
        --planning-time 10.0 \
        --output "$stop_output"; then
        echo "[accept] bridge accepted a goal after fail-closed stop"
        RC=1
    elif ! /usr/bin/python3 - "$stop_output" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
execution = payload.get("execution") or {}
if payload.get("success") or execution.get("success"):
    print("fail-closed case unexpectedly succeeded:", payload)
    raise SystemExit(1)
print("ok: goal rejected after fail-closed stop")
PY
    then
        RC=1
    fi
fi

echo "[accept] ===== 日志中的问题项 ====="
# 两类预期日志：无 3D 传感器时的 Octomap 告警；fail-closed 负向用例主动触发
# 的停止/拒绝日志。
grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" \
    | grep -v "occupancy_map_monitor" \
    | grep -v "fail-closed" \
    | head -10 || echo "（无）"
echo "[accept] 退出码=$RC"
exit $RC
