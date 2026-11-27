#!/bin/bash
# acceptance_d435i.sh — 一键干跑验收：起栈 -> 等感知 -> 位姿规划执行 ->
# 动态障碍重规划 -> 收栈。
#
# 前提（已 build 并 source 工作区）：
#   source /opt/ros/humble/setup.bash
#   source ~/litearm_ws/install/setup.bash
#   sudo apt install ros-humble-gazebo-plugins
#
# 退出码 0 = 探针通过。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$(mktemp -t litearm_d435i_acceptance.XXXXXX.log)"

source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/litearm_ws/install/setup.bash" ]]; then
  source "${HOME}/litearm_ws/install/setup.bash"
fi

# 与 launch 默认值保持一致；探针和栈必须在同一个 domain。
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "启动仿真栈（日志: ${LOG_FILE}）..."
setsid ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
  gui:=false use_rviz:=false start_dynamic_obstacle:=false \
  > "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

echo "运行探针（最坏约 4 分钟）..."
ros2 run litearm_d435i_perception d435i_probe.py --timeout 120
RC=$?

if [[ ${RC} -ne 0 ]]; then
  echo "---- launch 日志末尾 ----"
  tail -n 80 "${LOG_FILE}" || true
fi
echo "验收退出码: ${RC}   完整日志: ${LOG_FILE}"
exit "${RC}"
