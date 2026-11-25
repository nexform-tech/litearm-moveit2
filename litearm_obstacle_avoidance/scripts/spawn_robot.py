#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""spawn_robot.py — 从 /robot_description 把机器人插入正在运行的 Gazebo。

为什么不用 gazebo_ros/spawn_entity.py
=====================================
那个脚本的 shebang 是 `#!/usr/bin/env python3`，在装有 conda 的机器上会
解析到 conda 的 Python，缺少 lxml 直接 `ModuleNotFoundError`。本脚本用系统
解释器，并直接调用 `/spawn_entity` 服务（Gazebo 侧解析 URDF，客户端不需要
lxml）。

    /robot_description  (std_msgs/String, transient local)
        -> gazebo_msgs/SpawnEntity(name, xml, initial_pose, reference_frame)

退出码 0 = 插入成功（launch 据此启动控制器 spawner）。
"""

import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String


class SpawnRobot(Node):
    def __init__(self):
        super().__init__("litearm_gazebo_spawner")
        self.declare_parameter("entity_name", "litearm")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("service_name", "/spawn_entity")
        self.declare_parameter("z", 0.0)
        # Gazebo 侧模型与 ROS/MoveIt 侧模型可能不同（例如 link 改名后）。
        # 给了 robot_xml 就直接用它插入，不再等 /robot_description 话题。
        self.declare_parameter("robot_xml", "")
        self.description = None

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, self.get_parameter("robot_description_topic").value,
            self._on_description, qos)
        self.client = self.create_client(
            SpawnEntity, self.get_parameter("service_name").value)

    def _on_description(self, msg):
        self.description = msg.data


def main():
    rclpy.init()
    node = SpawnRobot()
    try:
        deadline = node.get_clock().now().nanoseconds / 1e9 + 60.0
        robot_xml = str(node.get_parameter("robot_xml").value or "")
        while ((not robot_xml and node.description is None)
               or not node.client.service_is_ready()):
            if node.get_clock().now().nanoseconds / 1e9 > deadline:
                print("✗ 等待 /robot_description 或 /spawn_entity 超时",
                      file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        request = SpawnEntity.Request()
        request.name = node.get_parameter("entity_name").value
        # gazebo_ros2_control 会把 robot_description 作为 `--param` 传给内部的
        # controller_manager；多行字符串会让 rcl 的参数覆盖解析器报
        # "Couldn't parse parameter override rule"。压成单行发送（只影响
        # Gazebo 插入的模型；RSP/MoveIt 仍用原始的 /robot_description）。
        description = robot_xml or node.description
        request.xml = " ".join(description.split())
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose = Pose()
        request.initial_pose.position.z = float(node.get_parameter("z").value)

        future = node.client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        response = future.result()
        if response is None:
            print("✗ /spawn_entity 调用超时", file=sys.stderr)
            return 1
        if not response.success:
            print(f"✗ 插入失败: {response.status_message}", file=sys.stderr)
            return 1
        node.get_logger().info(
            f"已插入 {request.name}（{response.status_message}）")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
