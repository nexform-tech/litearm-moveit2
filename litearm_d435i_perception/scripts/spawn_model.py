#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""spawn_model.py — 把一个 SDF 文件插入正在运行的 Gazebo。

为什么不用 gazebo_ros/spawn_entity.py
=====================================
那个脚本的 shebang 是 `#!/usr/bin/env python3`，在装了 conda 的机器上会解析
到 conda 的 Python，缺 lxml 直接 ModuleNotFoundError。本脚本固定用系统
/usr/bin/python3，并直接调用 /spawn_entity 服务；SDF 由 Gazebo 侧解析，
客户端不需要 lxml。

参数
====
    entity_name      Gazebo 实体名
    model_file       .sdf 路径
    mount_file       可选 YAML，含 frame_id 与 pose.xyz / pose.rpy；
                     给了就优先于 x/y/z/roll/pitch/yaw
    x/y/z/roll/pitch/yaw  初始位姿（固定轴 ZYX）
    reference_frame  默认 world

退出码 0 = 插入成功。
"""

import math
import sys

import rclpy
import yaml
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def quaternion_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SpawnModel(Node):
    def __init__(self):
        super().__init__("d435i_spawn_model")
        self.declare_parameter("entity_name", "d435i")
        self.declare_parameter("model_file", "")
        self.declare_parameter("mount_file", "")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("service_name", "/spawn_entity")
        self.declare_parameter("timeout", 60.0)

        self.client = self.create_client(
            SpawnEntity, self.get_parameter("service_name").value)

    def pose_from_parameters(self):
        xyz = [float(self.get_parameter(name).value)
               for name in ("x", "y", "z")]
        rpy = [float(self.get_parameter(name).value)
               for name in ("roll", "pitch", "yaw")]
        mount_file = self.get_parameter("mount_file").value
        if mount_file:
            with open(mount_file, "r", encoding="utf-8") as handle:
                mount = yaml.safe_load(handle)
            pose = mount.get("pose", {})
            xyz = [float(v) for v in pose.get("xyz", xyz)]
            rpy = [float(v) for v in pose.get("rpy", rpy)]
        return xyz, rpy


def main():
    rclpy.init()
    node = SpawnModel()
    try:
        model_file = node.get_parameter("model_file").value
        if not model_file:
            print("✗ 参数 model_file 不能为空", file=sys.stderr)
            return 1
        with open(model_file, "r", encoding="utf-8") as handle:
            xml = handle.read()

        timeout = float(node.get_parameter("timeout").value)
        deadline = node.get_clock().now().nanoseconds / 1e9 + timeout
        try:
            while not node.client.service_is_ready():
                if node.get_clock().now().nanoseconds / 1e9 > deadline:
                    print("✗ 等待 /spawn_entity 超时", file=sys.stderr)
                    return 1
                rclpy.spin_once(node, timeout_sec=0.1)
        except ExternalShutdownException:
            print("✗ 启动被中断", file=sys.stderr)
            return 1

        xyz, rpy = node.pose_from_parameters()
        request = SpawnEntity.Request()
        request.name = node.get_parameter("entity_name").value
        request.xml = xml
        request.robot_namespace = ""
        request.reference_frame = node.get_parameter("reference_frame").value
        request.initial_pose = Pose()
        request.initial_pose.position.x = xyz[0]
        request.initial_pose.position.y = xyz[1]
        request.initial_pose.position.z = xyz[2]
        qx, qy, qz, qw = quaternion_from_euler(*rpy)
        request.initial_pose.orientation.x = qx
        request.initial_pose.orientation.y = qy
        request.initial_pose.orientation.z = qz
        request.initial_pose.orientation.w = qw

        future = node.client.call_async(request)
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        except ExternalShutdownException:
            print("✗ 启动被中断", file=sys.stderr)
            return 1
        response = future.result()
        if response is None:
            print("✗ /spawn_entity 调用超时", file=sys.stderr)
            return 1
        if not response.success:
            print(f"✗ 插入 {request.name} 失败: {response.status_message}",
                  file=sys.stderr)
            return 1
        node.get_logger().info(
            f"已插入 {request.name} at {xyz} rpy={rpy} "
            f"({response.status_message})")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
