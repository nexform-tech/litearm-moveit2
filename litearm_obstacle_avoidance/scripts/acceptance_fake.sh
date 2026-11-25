#!/usr/bin/env bash
# acceptance_fake.sh — 干跑验收（mock_components，不接机械臂）。
#
#   起干跑栈（假硬件 + move_group）→ 等 JTC active
#   → （可选）合成点云 + Octomap 就绪
#   → 规划并执行到 ready 位形 → 收栈
#
# 用法:
#   scripts/acceptance_fake.sh              # 执行链（默认带 scene.yaml 静态环境）
#   scripts/acceptance_fake.sh --cloud      # 执行链 + 合成点云 -> Octomap
#   scripts/acceptance_fake.sh --scene      # 另加阶段 1 场景验收（对象就位、
#                                           # 轨迹逐点有效、目标被包住时规划失败）
#   scripts/acceptance_fake.sh --margin     # 另加阶段 2 裕度验收（夹爪全行程
#                                           # 保守包络 + margin 确实参与碰撞检查）
#   scripts/acceptance_fake.sh --replan     # 另加阶段 3 验收（执行中注入障碍 ->
#                                           # 取消/重规划；目标被包住 -> 安全撤退/停靠）
#   scripts/acceptance_fake.sh --safety     # 另加阶段 4 验收（停掉
#                                           # joint_state_broadcaster -> 数据过期拒动）
#   scripts/acceptance_fake.sh --timing     # 另加性能基线（规划时间与
#                                           # 碰撞检查耗时，写入 JSON）
#
# 验证的是：/move_action -> move_group -> follow_joint_trajectory -> JTC
# -> mock 硬件 -> /joint_states 收敛到目标。不会动机械臂。
#
# 环境注意（与 acceptance_moveit.sh 相同）：强制独立域名 + 只走本机，避免
# 同网段其它机器人的 /move_action server 接管目标。

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
# ⚠ 两个值必须**显式传给 launch**（见下方启动命令）。launch 默认给节点设
#   ros_localhost_only=true；如果本脚本继承的是 0，脚本里的 Python 客户端
#   与栈就处在不同的发现配置里，双方看不到彼此——实测现象是订阅者
#   count_publishers('/joint_states') 返回 0、ros2 control 永远等不到服务。

# ⚠ 这里必须用系统解释器。PATH 里的 python3 可能是 conda 环境（本机是
# Python 3.11），与 ROS Humble 的 Python 3.10 绑定不兼容，探针会直接
# ModuleNotFoundError: rclpy._rclpy_pybind11。
PYTHON="${PYTHON:-/usr/bin/python3}"

LOG="${TMPDIR:-/tmp}/litearm_acceptance_fake.log"

cd "$WS_DIR"
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

PROBE="$(ros2 pkg prefix litearm_moveit_config)/lib/litearm_moveit_config/moveit_probe.py"
SCENE_PROBE="$SCRIPT_DIR/scene_probe.py"
MARGIN_PROBE="$SCRIPT_DIR/margin_probe.py"
REPLAN_PROBE="$SCRIPT_DIR/replan_probe.py"
SAFETY_PROBE="$SCRIPT_DIR/safety_probe.py"
TIMING_PROBE="$SCRIPT_DIR/timing_probe.py"

USE_CLOUD=0
USE_SCENE=0
USE_MARGIN=0
USE_REPLAN=0
USE_SAFETY=0
USE_TIMING=0
for arg in "$@"; do
    case "$arg" in
        --cloud) USE_CLOUD=1 ;;
        --scene) USE_SCENE=1 ;;
        --margin) USE_MARGIN=1 ;;
        --replan) USE_REPLAN=1 ;;
        --safety) USE_SAFETY=1 ;;
        --timing) USE_TIMING=1 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done
START_SUPERVISOR=0
if [ "$USE_REPLAN" = 1 ] || [ "$USE_SAFETY" = 1 ]; then
    START_SUPERVISOR=1
fi

PIDFILE="${TMPDIR:-/tmp}/litearm_acceptance_fake.pid"
rm -f "$LOG" "$PIDFILE"
# 必须把 launch 放进独立会话并**记录会话首进程的 pid**：
#   - 直接 `ros2 launch &` 时，后台进程与脚本同组，kill 到组会连脚本一起杀；
#   - `setsid cmd &` 的 $! 可能是 fork 出来的父进程，收栈时打不到真正进程组，
#     会留下多套同名栈（/controller_manager、/move_group 各两个），后续运行
#     全变成假故障（实测踩到过）。
# 这里由 bash 在被 setsid 放进新会话后先写 pid 再 exec，pid == 新会话的 PGID。
setsid bash -c 'echo $$ >"$1"; shift; exec "$@"' _ "$PIDFILE" \
    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py \
    use_rviz:=false use_synthetic_cloud:="$([ "$USE_CLOUD" = 1 ] && echo true || echo false)" \
    start_supervisor:="$([ "$START_SUPERVISOR" = 1 ] && echo true || echo false)" \
    ros_domain_id:="$ROS_DOMAIN_ID" ros_localhost_only:="$ROS_LOCALHOST_ONLY" \
    >"$LOG" 2>&1 </dev/null &
for _ in $(seq 1 100); do
    [ -s "$PIDFILE" ] && break
    sleep 0.1
done
LAUNCH_PID="$(cat "$PIDFILE" 2>/dev/null || true)"
if [ -z "$LAUNCH_PID" ]; then
    echo "[accept] 启动失败：没有拿到 launch 的进程组 pid"
    exit 1
fi
echo "[accept] 干跑栈已启动 pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID cloud=$USE_CLOUD log=$LOG"

cleanup() {
    # 负号 = 整个进程组；launch 与它的所有子节点都在这个新会话里。
    kill -INT -"$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 20); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -TERM -"$LAUNCH_PID" 2>/dev/null
    sleep 1
    kill -KILL -"$LAUNCH_PID" 2>/dev/null
    rm -f "$PIDFILE"
    echo "[accept] 已收栈（move_group 在本机 MoveIt 2.5.10 上收栈时会崩溃，属上游已知问题，见日志）"
}
trap cleanup EXIT

deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
    if grep -q "You can start planning now" "$LOG" 2>/dev/null; then
        echo "[accept] move_group 就绪"
        break
    fi
    sleep 1
done
if ! grep -q "You can start planning now" "$LOG" 2>/dev/null; then
    echo "[accept] 超时未就绪，日志尾部："
    tail -40 "$LOG"
    exit 1
fi

# JTC 的 action 在 spawner 激活后才出现，必须等它，否则执行会找不到控制器。
# 用 launch 日志判断而不是 `ros2 control list_controllers`：后者在服务发现
# 偶发卡住时没有超时，会把脚本永久挂住（实测踩到过）。
deadline=$((SECONDS + 60))
while (( SECONDS < deadline )); do
    if grep -q "Configured and activated .*joint_trajectory_controller" "$LOG" 2>/dev/null; then
        echo "[accept] JTC active"
        break
    fi
    sleep 1
done
if ! grep -q "Configured and activated .*joint_trajectory_controller" "$LOG" 2>/dev/null; then
    echo "[accept] JTC 未激活，日志尾部："
    tail -40 "$LOG"
    exit 1
fi
# 刻意不调用 `ros2 control list_controllers`：该 CLI 在服务发现未就绪时会
# 无限等待（没有超时），曾把脚本挂死。launch 日志里 spawner 的激活行
# 已经是权威信号，不需要再查一遍。

# 探针自己在 /joint_states 上只等 2 秒；这里先确认 7 个臂关节已经在图上，
# 否则刚激活 JTC 就调用会拿到空状态（实测踩到过）。
_joints_ready() {
    "$PYTHON" - <<'PY' >/dev/null 2>&1
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init()
node = Node("joint_states_probe")
seen = set()
def callback(msg):
    seen.update(msg.name)
node.create_subscription(JointState, "/joint_states", callback, 10)
for _ in range(100):
    rclpy.spin_once(node, timeout_sec=0.05)
    if all(f"joint{i}" in seen for i in range(1, 8)):
        break
ok = all(f"joint{i}" in seen for i in range(1, 8))
node.destroy_node()
if rclpy.ok():
    rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
}
deadline=$((SECONDS + 30))
until _joints_ready; do
    if (( SECONDS >= deadline )); then
        echo "[accept] /joint_states 未包含 7 个臂关节，日志尾部："
        tail -40 "$LOG"
        exit 1
    fi
    sleep 1
done
echo "[accept] /joint_states 就绪"

if [ "$USE_CLOUD" = 1 ]; then
    deadline=$((SECONDS + 60))
    bytes=0
    while (( SECONDS < deadline )); do
        bytes="$("$PYTHON" - <<'PY' 2>/dev/null
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init()
node = Node("octomap_probe")
client = node.create_client(GetPlanningScene, "/get_planning_scene")
value = 0
if client.wait_for_service(timeout_sec=5.0):
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.OCTOMAP
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is not None:
        value = len(response.scene.world.octomap.octomap.data)
print(value)
node.destroy_node()
if rclpy.ok():
    rclpy.shutdown()
PY
)"
        if [ "${bytes:-0}" -gt 0 ] 2>/dev/null; then
            echo "[accept] Octomap 已生成（${bytes} 字节）"
            break
        fi
        sleep 2
    done
    if [ "${bytes:-0}" -le 0 ] 2>/dev/null; then
        echo "[accept] Octomap 未生成（合成点云没有被 occupancy map monitor 接收）"
        tail -40 "$LOG"
        exit 1
    fi
fi

if [ "$USE_SCENE" = 1 ]; then
    echo "[accept] ===== 阶段 1：静态环境验收 ====="
    "$PYTHON" "$SCENE_PROBE"
    SCENE_RC=$?
    if [ "$SCENE_RC" -ne 0 ]; then
        echo "[accept] 场景验收失败（退出码 $SCENE_RC）"
        tail -40 "$LOG"
        exit "$SCENE_RC"
    fi
fi

if [ "$USE_MARGIN" = 1 ]; then
    echo "[accept] ===== 阶段 2：碰撞裕度验收 ====="
    "$PYTHON" "$MARGIN_PROBE"
    MARGIN_RC=$?
    if [ "$MARGIN_RC" -ne 0 ]; then
        echo "[accept] 裕度验收失败（退出码 $MARGIN_RC）"
        tail -40 "$LOG"
        exit "$MARGIN_RC"
    fi
fi

if [ "$USE_REPLAN" = 1 ]; then
    echo "[accept] ===== 阶段 3：重规划与安全撤退/停靠验收 ====="
    "$PYTHON" "$REPLAN_PROBE"
    REPLAN_RC=$?
    if [ "$REPLAN_RC" -ne 0 ]; then
        echo "[accept] 重规划验收失败（退出码 $REPLAN_RC）"
        tail -60 "$LOG"
        exit "$REPLAN_RC"
    fi
fi

if [ "$USE_SAFETY" = 1 ]; then
    echo "[accept] ===== 阶段 4：数据过期拒动验收 ====="
    "$PYTHON" "$SAFETY_PROBE"
    SAFETY_RC=$?
    if [ "$SAFETY_RC" -ne 0 ]; then
        echo "[accept] 安全验收失败（退出码 $SAFETY_RC）"
        tail -60 "$LOG"
        exit "$SAFETY_RC"
    fi
fi

if [ "$USE_TIMING" = 1 ]; then
    echo "[accept] ===== 性能基线：规划时间与碰撞检查耗时 ====="
    "$PYTHON" "$TIMING_PROBE"
    TIMING_RC=$?
    if [ "$TIMING_RC" -ne 0 ]; then
        echo "[accept] 性能基线采集失败（退出码 $TIMING_RC）"
        tail -40 "$LOG"
        exit "$TIMING_RC"
    fi
fi

"$PYTHON" "$PROBE" --execute
RC=$?

echo "[accept] ===== 运行期日志中的问题项 ====="
# 收栈时的 move_group 崩溃发生在 cleanup（trap）里，此处还没发生。
# 预期噪音：
#   - occupancy_map_monitor 的 "No 3D sensor plugin(s)"：未启用点云时的固定告警；
#   - shape_mask 的 "Missing transform for shape ..."：第一条点云到达时自滤波的
#     TF 还没就绪；以及 --safety 故意停掉 joint_state_broadcaster 期间 TF 停止更新。
#     两者都是预期窗口，恢复 /joint_states 后不再出现；
#   - OMPL 的 goal tree 报错：--scene 的反向用例故意把目标包住，规划失败是
#     预期结果。
grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" \
    | grep -v "occupancy_map_monitor" \
    | grep -v "Missing transform for shape" \
    | grep -v "Unable to sample any valid states for goal tree" \
    | grep -v "Insufficient states in sampleable goal region" \
    | grep -v "拒绝运动：stale_joint_states" \
    | head -10 || echo "（无）"
# 最后一条是 --safety 的预期结果：数据过期时 supervisor 主动拒绝。
echo "[accept] 探针退出码=$RC"
exit $RC
