#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""safety_probe.py — 阶段 4 验收：数据过期时拒绝运动。

做法（不依赖停掉整个栈）：

  1. 确认 avoid_supervisor 在运行且空闲，/joint_states 正常
  2. 通过 /controller_manager/switch_controller 停掉
     joint_state_broadcaster —— /joint_states 随即停止
  3. 等到超过 supervisor 的 max_state_age（默认 1 s）
  4. 发一个目标：supervisor 必须以 failed/stale_joint_states 拒绝，
     且不能发布新轨迹
  5. 重新激活 joint_state_broadcaster，等 /joint_states 恢复

安全撤退由 replan_probe 的场景 B 验证（目标被包住 -> plan 失败 ->
retreated 或 safe_stop）。本探针只负责“数据过期拒动”。

退出码 0 = 符合预期。
"""

import json
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]


class SafetyProbe(Node):
    def __init__(self):
        super().__init__("litearm_safety_probe")
        self.status = None
        self.joints = {}
        self.last_joint_time = None
        self.trajectory_count = 0
        self.create_subscription(String, "/avoid_supervisor/status",
                                 self._on_status, 10)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)
        self.create_subscription(JointTrajectory,
                                 "/avoid_supervisor/planned_trajectory",
                                 self._on_trajectory, 10)
        self.goal_pub = self.create_publisher(
            JointState, "/avoid_supervisor/goal", 10)
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller")

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position
        self.last_joint_time = time.monotonic()

    def _on_trajectory(self, _msg):
        self.trajectory_count += 1

    def _wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        print(f"✗ 等待超时: {description}；最后状态 {self.status}")
        return False

    def _switch(self, activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(sec=5, nanosec=0)
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        response = future.result()
        return bool(response and response.ok)


def main():
    rclpy.init()
    node = SafetyProbe()
    failures = 0
    broadcaster_off = False
    try:
        if not node.switch_client.wait_for_service(timeout_sec=60.0):
            print("✗ /controller_manager/switch_controller 不可用")
            return 1
        if not node._wait_for(lambda: node.status is not None, 30,
                              "supervisor 状态"):
            print("✗ avoid_supervisor 未运行（用 start_supervisor:=true 启动）")
            return 1
        if not node._wait_for(
                lambda: all(name in node.joints for name in JOINTS), 30,
                "/joint_states 包含 7 个臂关节"):
            return 1
        if not node._wait_for(lambda: not node.status.get("busy", False), 60,
                              "supervisor 空闲"):
            return 1

        # 1) 停掉 joint_state_broadcaster
        if not node._switch(activate=[], deactivate=["joint_state_broadcaster"]):
            print("✗ 停用 joint_state_broadcaster 失败")
            return 1
        broadcaster_off = True
        print("  已停用 joint_state_broadcaster，等待 /joint_states 变旧…")
        if not node._wait_for(
                lambda: node.last_joint_time is not None
                and time.monotonic() - node.last_joint_time > 2.0,
                15, "/joint_states 停止发布"):
            return 1
        baseline = node.trajectory_count

        # 2) 发目标：必须被拒绝
        goal = JointState()
        goal.name = list(JOINTS)
        goal.position = [float(v) for v in GOAL]
        node.goal_pub.publish(goal)
        ok = node._wait_for(
            lambda: node.status.get("state") == "failed"
            and node.status.get("reason") == "stale_joint_states",
            15, "supervisor 拒绝过期状态")
        if not ok:
            failures += 1
            print("✗ 数据过期时没有拒绝运动")
        else:
            print("✓ 数据过期时拒绝运动（failed/stale_joint_states）")

        # 3) 不应发布任何新轨迹
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.trajectory_count != baseline:
            failures += 1
            print("✗ 被拒绝后仍然发布了新轨迹")
        else:
            print("✓ 被拒绝后没有下发任何轨迹")

        # 4) 恢复 broadcaster
        if not node._switch(activate=["joint_state_broadcaster"], deactivate=[]):
            failures += 1
            print("✗ 重新激活 joint_state_broadcaster 失败")
        else:
            broadcaster_off = False
            if node._wait_for(
                    lambda: node.last_joint_time is not None
                    and time.monotonic() - node.last_joint_time < 1.0,
                    30, "/joint_states 恢复"):
                print("✓ joint_state_broadcaster 已恢复")
            else:
                failures += 1
    finally:
        if broadcaster_off:
            node._switch(activate=["joint_state_broadcaster"], deactivate=[])
            print("  （finally）已尝试恢复 joint_state_broadcaster")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failures:
        print(f"\n✗ 数据过期安全验收失败（{failures} 项）")
        return 1
    print("\n✓ 数据过期拒动验收通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
