#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""gazebo_robot_description.py — 只提供 robot_description 参数的占位节点。

为什么需要它
============
Gazebo 侧 URDF 的 link 名被改成了 gz_joint1.. 规避 sdformat9 的
link/joint 同名冲突；但 MoveIt 的模型（以及 PointCloudOctomapUpdater 的
ShapeMask 自滤波）必须使用原始 link 名 joint1..。两者不能共用一个
robot_state_publisher：

  · robot_state_publisher 用 **MoveIt 模型** 发 TF（joint1..）；
  · gazebo_ros2_control 通过 plugin 里的 <robot_param_node> 找本节点，读取
    **Gazebo 模型**（gz_joint1..，带 <ros2_control>）的 robot_description。

本节点不发 TF、不订阅任何话题，只声明参数并保持参数服务可用。
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GazeboRobotDescription(Node):
    def __init__(self):
        super().__init__("gazebo_robot_description")
        self.declare_parameter("robot_description", "")
        description = self.get_parameter("robot_description").value
        if not description:
            self.get_logger().error("robot_description 为空")
        else:
            self.get_logger().info(
                f"gazebo robot_description 已就绪（{len(description)} 字符）")


def main():
    rclpy.init()
    node = GazeboRobotDescription()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
