#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""replan_probe.py — 阶段 3 验收：执行中注入障碍 -> 取消 -> 重规划；失败 -> 安全停靠。

前提：干跑栈里启动了 avoid_supervisor（obstacle_fake.launch.py
start_supervisor:=true，acceptance_fake.sh --replan 会自动这样做）。

场景 A（重规划成功）
  1. 发目标 GOAL；等 supervisor 进入 executing 并拿到计划轨迹
  2. 取轨迹中段路点的 ee_link 位姿，在那里注入 12 cm 方块
  3. supervisor 应检测到场景变化 -> cancel -> replan
  4. 期望 status: replans >= 1 且最终 succeeded，关节收敛到 GOAL
  5. 移除方块

场景 B（重规划失败 -> 安全撤退/停靠）
  1. 发目标 ZERO；等 executing
  2. 在 ZERO 位姿的末端注入 30 cm 方块，使目标状态本身处于碰撞
  3. 重规划必然失败 -> supervisor 按安全策略处理：
       能规划到 ready 就撤退（retreated），否则保持停住（safe_stop）
  4. 检查机械臂停住（1 s 内关节角变化 < 0.02 rad）；
     retreated 时还要到达 ready，safe_stop 时不能已经到达 ZERO
  5. 移除方块

退出码 0 = 两个场景都符合预期。
"""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (CollisionObject, PlanningScene, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionFK,
                             GetStateValidity)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]
ZERO = [0.0] * 7
READY = [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]  # supervisor 默认撤退目标
HOLD_TOLERANCE = 0.02
GOAL_TOLERANCE = 0.08


class ReplanProbe(Node):
    def __init__(self):
        super().__init__("litearm_replan_probe")
        self.status = None
        self.trajectory = None
        self.trajectory_count = 0
        self.joints = {}
        self.create_subscription(String, "/avoid_supervisor/status",
                                 self._on_status, 10)
        self.create_subscription(JointTrajectory,
                                 "/avoid_supervisor/planned_trajectory",
                                 self._on_trajectory, 10)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)
        self.goal_pub = self.create_publisher(
            JointState, "/avoid_supervisor/goal", 10)
        self.apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity")

    # ── 回调 ──
    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_trajectory(self, msg):
        self.trajectory = msg
        self.trajectory_count += 1

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    # ── 工具 ──
    def _state(self):
        return self.status or {}

    def _wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        print(f"✗ 等待超时: {description}；最后状态 {self._state()}")
        return False

    def _call(self, client, request, timeout=20.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def _publish_goal(self, positions):
        msg = JointState()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in positions]
        self.goal_pub.publish(msg)

    def _fk(self, positions):
        request = GetPositionFK.Request()
        request.header = Header()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["ee_link"]
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS) + ["gripper_opening_joint"]
        request.robot_state.joint_state.position = \
            [float(v) for v in positions] + [0.0]
        response = self._call(self.fk_client, request)
        if response is None or response.error_code.val != 1 or not response.pose_stamped:
            return None
        return response.pose_stamped[0].pose

    def _validity(self, positions):
        request = GetStateValidity.Request()
        request.group_name = ""
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS) + ["gripper_opening_joint"]
        request.robot_state.joint_state.position = \
            [float(v) for v in positions] + [0.0]
        response = self._call(self.validity_client, request)
        return response is not None and bool(response.valid)

    def _apply_box(self, object_id, pose, size, add):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = object_id
        obj.operation = CollisionObject.ADD if add else CollisionObject.REMOVE
        if add:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(size)] * 3
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
        scene.world.collision_objects.append(obj)
        response = self._call(self.apply_client,
                              ApplyPlanningScene.Request(scene=scene))
        return bool(response and response.success)

    def _near(self, positions, target, tolerance):
        return max(abs(a - b) for a, b in zip(positions, target)) <= tolerance

    def _current(self):
        return [self.joints.get(name, 1e9) for name in JOINTS]


def main():
    rclpy.init()
    node = ReplanProbe()
    failures = 0
    boxes = []

    def remove_boxes():
        for object_id, pose, size in boxes:
            node._apply_box(object_id, pose, size, add=False)
        boxes.clear()

    try:
        for name, client in (("/apply_planning_scene", node.apply_client),
                             ("/compute_fk", node.fk_client),
                             ("/check_state_validity", node.validity_client)):
            if not client.wait_for_service(timeout_sec=60.0):
                print(f"✗ 服务 {name} 不可用")
                return 1

        if not node._wait_for(lambda: node.status is not None, 30,
                              "supervisor 状态"):
            print("✗ avoid_supervisor 未运行（用 start_supervisor:=true 启动）")
            return 1
        if not node._wait_for(
                lambda: all(name in node.joints for name in JOINTS), 30,
                "/joint_states 包含 7 个臂关节"):
            return 1

        # ── 场景 A：执行中注入障碍 -> 重规划成功 ──
        print("── 场景 A：执行中注入障碍 ──")
        baseline = node.trajectory_count
        node._publish_goal(GOAL)
        if not node._wait_for(
                lambda: node._state().get("state") == "executing"
                and node.trajectory_count > baseline,
                60, "进入 executing 并收到计划轨迹"):
            return 1
        trajectory = node.trajectory
        points = trajectory.points
        if len(points) < 3:
            print(f"✗ 轨迹点太少 ({len(points)})，无法在中段注入")
            return 1
        # 在轨迹中后段找一个注入位置：障碍要挡在“前面”，但不能碰到当前
        # 状态或目标状态，否则重规划的起点/终点本身就是碰撞。
        chosen = None
        for fraction in (0.4, 0.5, 0.6, 0.7, 0.8):
            index = min(len(points) - 2, max(1, int(len(points) * fraction)))
            pose = node._fk(points[index].positions)
            if pose is None:
                continue
            if not node._apply_box("replan_probe_a", pose, 0.05, add=True):
                print("✗ 注入方块失败")
                return 1
            if node._validity(node._current()) and node._validity(GOAL):
                chosen = pose
                print(f"  注入路点 #{index}/{len(points)}（{fraction:.0%}），末端 "
                      f"({pose.position.x:.3f}, {pose.position.y:.3f}, "
                      f"{pose.position.z:.3f})")
                boxes.append(("replan_probe_a", pose, 0.05))
                break
            node._apply_box("replan_probe_a", pose, 0.05, add=False)
            print(f"  路点 #{index} 会碰到当前/目标状态，换下一个")
        if chosen is None:
            print("✗ 找不到既不碰当前状态、也不碰目标的注入位置")
            return 1

        if not node._wait_for(lambda: node._state().get("replans", 0) >= 1,
                              40, "supervisor 开始重规划"):
            remove_boxes()
            return 1
        print(f"  supervisor 已重规划 {node._state().get('replans')} 次")
        if not node._wait_for(lambda: node._state().get("state") == "succeeded",
                              90, "场景 A 最终 succeeded"):
            remove_boxes()
            return 1
        remove_boxes()
        if not node._near(node._current(), GOAL, GOAL_TOLERANCE):
            failures += 1
            print(f"✗ 场景 A 结束位置未收敛到目标: "
                  f"{[round(v, 3) for v in node._current()]}")
        else:
            print("✓ 场景 A：注入障碍后先停、重规划、到达目标")

        # ── 场景 B：目标被包住 -> 安全停靠 ──
        print("── 场景 B：重规划失败 -> 安全撤退/停靠 ──")
        zero_pose = node._fk(ZERO)
        if zero_pose is None:
            print("✗ ZERO 位姿 compute_fk 失败")
            return 1
        baseline = node.trajectory_count
        node._publish_goal(ZERO)
        if not node._wait_for(
                lambda: node._state().get("state") == "executing"
                and node._state().get("replans", 0) == 0
                and node.trajectory_count > baseline,
                60, "场景 B 进入 executing"):
            return 1
        if not node._apply_box("replan_probe_b", zero_pose, 0.30, add=True):
            print("✗ 注入目标方块失败")
            return 1
        boxes.append(("replan_probe_b", zero_pose, 0.30))

        # 失败后按安全策略走：能规划到 ready 就撤退（retreated），
        # 否则保持停住（safe_stop）。两种都算正确，关键是不能继续往目标跑。
        if not node._wait_for(
                lambda: node._state().get("state") in ("safe_stop", "retreated"),
                120, "场景 B 进入 safe_stop / retreated"):
            remove_boxes()
            return 1
        final_state = node._state().get("state")
        print(f"  失败处理: {final_state}，原因: {node._state().get('reason')}")
        held = node._current()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        drift = max(abs(a - b) for a, b in zip(held, node._current()))
        remove_boxes()
        if drift > HOLD_TOLERANCE:
            failures += 1
            print(f"✗ 失败处理后机械臂仍在动（1 s 漂移 {drift:.3f} rad）")
        elif final_state == "retreated":
            if node._near(held, READY, 0.12):
                print(f"✓ 场景 B：重规划失败后安全撤退到 ready"
                      f"（漂移 {drift:.4f} rad）")
            else:
                failures += 1
                print(f"✗ 状态为 retreated，但未到达 ready: "
                      f"{[round(v, 3) for v in held]}")
        elif not node._near(held, ZERO, 0.1):
            print(f"✓ 场景 B：重规划失败后安全停靠（漂移 {drift:.4f} rad，"
                  "未到达目标）")
        else:
            failures += 1
            print("✗ 安全停靠时已经到达 ZERO，无法证明是中途停止")
    finally:
        remove_boxes()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failures:
        print(f"\n✗ 重规划/安全停靠验收失败（{failures} 项）")
        return 1
    print("\n✓ 重规划与安全停靠验收通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
