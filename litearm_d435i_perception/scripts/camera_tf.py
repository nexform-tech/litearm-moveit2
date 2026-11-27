#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""camera_tf.py — 发布固定外置 D435i 的 TF 树。

从 config/camera_mount.yaml 读取相机安装位姿，发布：

    base_link -> camera_link                    (安装位姿)
    camera_link -> camera_depth_optical_frame   (rpy -pi/2, 0, -pi/2)
    camera_link -> camera_color_optical_frame   (同上)

为什么需要光学帧
================
Gazebo 相机插件的点云按光学约定（x 右、y 下、z 前）计算，但 header.frame_id
直接取 SDF 的 <frame_name>。真实 realsense2_camera 的 frame_id 是
camera_depth_optical_frame，且它的 optical_frame 相对 camera_link 的旋转正是
rpy(-pi/2, 0, -pi/2)。所以仿真里也发同样的静态变换，点云进入 Octomap 前
才能被 TF 正确变到 base_link；否则地图会整体绕相机旋转 90°。

固定外置场景下 base_link 不随臂运动，静态 TF 足够；不需要 robot_state_publisher
之外的动态广播。
"""

import math
import sys

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    """固定轴 ZYX 欧拉角 -> 四元数 (x, y, z, w)，与 Gazebo/URDF 约定一致。"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_transform(parent, child, xyz, rpy):
    msg = TransformStamped()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    qx, qy, qz, qw = quaternion_from_euler(*[float(v) for v in rpy])
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


class CameraTf(Node):
    def __init__(self):
        super().__init__("d435i_camera_tf")
        self.declare_parameter("mount_file", "")
        mount_file = self.get_parameter("mount_file").value
        if not mount_file:
            raise RuntimeError("参数 mount_file 不能为空")

        with open(mount_file, "r", encoding="utf-8") as handle:
            mount = yaml.safe_load(handle)

        base = mount.get("frame_id", "base_link")
        link = mount.get("camera_link", "camera_link")
        depth = mount.get("depth_optical_frame", "camera_depth_optical_frame")
        color = mount.get("color_optical_frame", "camera_color_optical_frame")
        pose = mount.get("pose", {})
        xyz = pose.get("xyz", [1.0, 0.0, 0.9])
        rpy = pose.get("rpy", [0.0, 0.0, 0.0])

        # REP 103：光学帧 z 前、x 右、y 下；相对机体的标准旋转。
        optical_rpy = [-math.pi / 2.0, 0.0, -math.pi / 2.0]

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform([
            make_transform(base, link, xyz, rpy),
            make_transform(link, depth, [0.0, 0.0, 0.0], optical_rpy),
            make_transform(link, color, [0.0, 0.0, 0.0], optical_rpy),
        ])
        self.get_logger().info(
            f"相机 TF: {base} -> {link} xyz={xyz} rpy={rpy}; "
            f"{link} -> {depth}/{color} rpy={optical_rpy}")


def main():
    rclpy.init()
    try:
        node = CameraTf()
    except Exception as exc:  # noqa: BLE001 - 把配置错误变成明确的退出码
        print(f"✗ camera_tf 启动失败: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
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
