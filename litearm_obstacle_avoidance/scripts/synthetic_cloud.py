#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""synthetic_cloud.py — 发布一个合成点云，用于在没有相机时打通 Octomap 链路。

⚠ 刻意用 /usr/bin/python3 而不是 `#!/usr/bin/env python3`：本机 PATH 里的
  python3 是 conda 环境，numpy/ROS 绑定混用容易出 ABI 问题（本仓库
  troubleshooting 里记录过同类问题）。ROS 的 Python 绑定装在系统解释器下。

在 frame_id（默认 base_link）下生成一个长方体的**表面点云**，按 rate_hz
周期发布到 point_cloud_topic。配合 config/sensors_3d.yaml，MoveIt 的
occupancy map monitor 会把这些点插入 Octomap，于是：

    /synthetic_cloud -> PointCloudOctomapUpdater -> Octomap -> PlanningScene

整条环境感知链路可以在没有深度相机的情况下验证。

参数（命令行或 launch 传入）：

    point_cloud_topic  /synthetic_cloud
    frame_id           base_link
    rate_hz            2.0
    box_center         [0.35, 0.0, 0.45]   长方体的中心（米）
    box_size           [0.10, 0.30, 0.10]  长方体的三边长度（米）
    point_step         0.01                表面点间距（米）

⚠ 这是测试夹具，不是传感器模型：没有噪声、没有遮挡、没有视野边界。
  它只回答「Octomap 链路是否通了」，不能用来评估感知质量。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _box_surface(center, size, step):
    """长方体六个面的规则点阵，返回 (N,3) float32。"""
    center = np.asarray(center, dtype=np.float64)
    half = np.asarray(size, dtype=np.float64) / 2.0
    if np.any(half <= 0.0) or step <= 0.0:
        raise ValueError("box_size 必须为正、point_step 必须为正")
    lo, hi = center - half, center + half
    axes = [np.arange(lo[i], hi[i] + step * 0.5, step) for i in range(3)]
    points = []
    for fixed_axis in range(3):
        other = [i for i in range(3) if i != fixed_axis]
        grid_a, grid_b = np.meshgrid(axes[other[0]], axes[other[1]], indexing="ij")
        for face in (lo[fixed_axis], hi[fixed_axis]):
            block = np.empty(grid_a.shape + (3,), dtype=np.float64)
            block[..., fixed_axis] = face
            block[..., other[0]] = grid_a
            block[..., other[1]] = grid_b
            points.append(block.reshape(-1, 3))
    return np.concatenate(points).astype(np.float32)


class SyntheticCloudPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_cloud")
        self.declare_parameter("point_cloud_topic", "/synthetic_cloud")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("box_center", [0.35, 0.0, 0.45])
        self.declare_parameter("box_size", [0.10, 0.30, 0.10])
        self.declare_parameter("point_step", 0.01)

        topic = self.get_parameter("point_cloud_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        rate = float(self.get_parameter("rate_hz").value)
        center = list(self.get_parameter("box_center").value)
        size = list(self.get_parameter("box_size").value)
        step = float(self.get_parameter("point_step").value)

        self.points = _box_surface(center, size, step)
        # 点云是传感器数据：用 SensorDataQoS，与真实相机和 MoveIt 订阅端一致。
        self.publisher = self.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(rate, 1e-3), self._publish)
        self.get_logger().info(
            f"发布合成点云: {topic}  frame={self.frame_id}  "
            f"points={len(self.points)}  rate={rate:g} Hz  "
            f"center={center} size={size}")

    def _publish(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, self.points)
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = SyntheticCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 的 SIGINT 可能已经关掉了 context，这时再 shutdown 会抛
        # "rcl_shutdown already called"；用 ok() 区分，避免非零退出码。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
