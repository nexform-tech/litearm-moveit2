#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""wrist_runtime_probe.py — 验证扫描执行中的剩余轨迹复检。

用法
====
先启动腕装扫描（无显示即可），再在另一个终端运行本探针：

    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    ros2 run litearm_d435i_perception wrist_runtime_probe.py

流程
====
  1. 订阅 /wrist_scan/planned_trajectory，取第一条足够长的轨迹；
  2. 取轨迹中点，用 /compute_fk 求 ee_link 位姿；
  3. 通过 /apply_planning_scene 在该位置注入一个 6 cm 方块；
  4. 等 /wrist_scan/status 进入 done/failed/cancelled。

通过条件：`runtime_replans >= 1`（扫描确实复检到新障碍并取消/重规划），
且最终状态不是一个卡死的中间态。退出码：

    0 = 复检触发并结束
    1 = 超时 / 未注入
    2 = 注入成功但没有报告 runtime_replans

探针会在结束时移除注入的方块（id=scan_runtime_probe）。
"""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionFK)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]
OBSTACLE_ID = "scan_runtime_probe"


class RuntimeProbe(Node):
    def __init__(self):
        super().__init__("wrist_runtime_probe")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene")
        self.status = {}
        self.pending = None
        self.injected = False
        self.create_subscription(
            JointTrajectory, "/wrist_scan/planned_trajectory",
            self._on_trajectory, 10)
        self.create_subscription(
            String, "/wrist_scan/status", self._on_status, 10)

    def _on_trajectory(self, msg):
        if self.injected or self.pending is not None:
            return
        if len(msg.points) < 8:
            return
        last = msg.points[-1].time_from_start
        if last.sec + last.nanosec * 1e-9 < 0.5:
            return
        self.pending = msg

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def _apply(self, obstacle):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.append(obstacle)
        response = self._call(
            self.apply_client, ApplyPlanningScene.Request(scene=scene))
        return bool(response is not None and response.success)

    def inject(self):
        trajectory = self.pending
        self.pending = None
        names = list(trajectory.joint_names)
        index = max(1, int(0.5 * (len(trajectory.points) - 1)))
        point = trajectory.points[index]
        by_name = dict(zip(names, point.positions))
        if any(name not in by_name for name in JOINTS):
            return False

        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["ee_link"]
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = [
            float(by_name[name]) for name in JOINTS]
        response = self._call(self.fk_client, request)
        if response is None or response.error_code.val != 1 \
                or not response.pose_stamped:
            self.get_logger().warning("FK 失败，未注入")
            return False
        pose = response.pose_stamped[0].pose
        self.get_logger().info(
            "在轨迹点 "
            f"#{index}/{len(trajectory.points) - 1} 注入 6 cm 障碍: ("
            f"{pose.position.x:.3f}, {pose.position.y:.3f}, "
            f"{pose.position.z:.3f})")

        obstacle = CollisionObject()
        obstacle.header.frame_id = "base_link"
        obstacle.id = OBSTACLE_ID
        obstacle.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.06, 0.06, 0.06]
        obstacle.primitives.append(primitive)
        box_pose = Pose()
        box_pose.position = pose.position
        box_pose.orientation.w = 1.0
        obstacle.primitive_poses.append(box_pose)
        if not self._apply(obstacle):
            self.get_logger().warning("apply_planning_scene 失败")
            return False
        self.injected = True
        return True

    def remove_obstacle(self):
        obstacle = CollisionObject()
        obstacle.header.frame_id = "base_link"
        obstacle.id = OBSTACLE_ID
        obstacle.operation = CollisionObject.REMOVE
        self._apply(obstacle)


def main():
    rclpy.init()
    node = RuntimeProbe()
    deadline = time.time() + 180.0
    result = 1
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending is not None and not node.injected:
                node.inject()
            if node.injected:
                state = node.status.get("state", "")
                if state in ("done", "failed", "cancelled"):
                    replans = int(node.status.get("runtime_replans", 0))
                    checks = int(node.status.get("validity_checks", 0))
                    print(json.dumps({
                        "state": state,
                        "runtime_replans": replans,
                        "validity_checks": checks,
                        "views_done": node.status.get("views_done"),
                        "views_failed": node.status.get("views_failed"),
                        "coverage": node.status.get("coverage"),
                    }, ensure_ascii=False))
                    result = 0 if replans >= 1 else 2
                    break
    finally:
        if node.injected:
            node.remove_obstacle()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if result == 2:
        print("FAIL: 已注入障碍但扫描没有报告 runtime_replans")
    elif result == 1:
        print("FAIL: 超时或未注入")
    return result


if __name__ == "__main__":
    sys.exit(main())
