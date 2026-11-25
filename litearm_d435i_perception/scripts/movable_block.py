#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""movable_block.py — 让 Gazebo 里的演示物块可以随时改位置。

模型 models/movable_block.sdf 由 launch 通过 spawn_model.py 插入
（<static>true</static>，只有 visual），本节点只通过
/gazebo/set_entity_state 改写它的世界位姿。物块一移动，D435i 的深度相机
就在新位置渲染它，/camera/depth/color/points、Octomap 和
/monitored_planning_scene 随之更新。

坐标约定
========
本仿真里机器人基座固定在 world 原点、spawn z=0，所以 base_link 与 world
完全重合。目标位姿的 header.frame_id 接受 world 或 base_link（空字符串也
按 world 处理），其它 frame 会被忽略并在日志里告警。发布
/movable_block/pose 时固定用 base_link，RViz 的 Fixed Frame 就是它。

接口
====
  订阅 /movable_block/target_pose   geometry_msgs/PoseStamped
       改位置 + 绕 Z 轴朝向（roll/pitch 忽略）。
  订阅 /movable_block/target_point  geometry_msgs/Point
       只改位置，朝向保持当前值；命令行最短。
  发布 /movable_block/pose          geometry_msgs/PoseStamped
       当前目标位姿，方便 RViz 里观察。
  服务 /movable_block/reset         std_srvs/Trigger
       回 initial_pose。

命令行示例（另开终端，export 同样的 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY）：
  ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20
  ros2 run litearm_d435i_perception move_block.py --reset
"""

import math
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT_POSE = [0.30, 0.55, -0.06]
# base_link 与 world 在本仿真中重合，两者都接受。
SUPPORTED_FRAMES = ("", "world", "base_link")


class MovableBlock(Node):
    def __init__(self):
        super().__init__("movable_block")
        self.declare_parameter("entity_name", "movable_block")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("initial_pose", DEFAULT_POSE)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("target_pose_topic",
                               "/movable_block/target_pose")
        self.declare_parameter("target_point_topic",
                               "/movable_block/target_point")
        self.declare_parameter("pose_topic", "/movable_block/pose")
        self.declare_parameter("reset_service", "/movable_block/reset")
        self.declare_parameter("set_state_service",
                               "/gazebo/set_entity_state")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("error_log_period", 2.0)

        self.entity = str(self.get_parameter("entity_name").value)
        self.reference_frame = str(
            self.get_parameter("reference_frame").value) or "world"
        self.initial = self._initial_pose()
        self.command = list(self.initial)
        self.dirty = True          # 启动后先把初始位姿下发一次
        self.pending = False
        self.version = 0
        self._inflight_version = -1
        self._last_error_time = -1e9
        self.error_log_period = max(
            0.0, float(self.get_parameter("error_log_period").value))

        self.set_client = self.create_client(
            SetEntityState, self.get_parameter("set_state_service").value)
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter("target_pose_topic").value,
            self._on_target_pose, 10)
        self.create_subscription(
            Point, self.get_parameter("target_point_topic").value,
            self._on_target_point, 10)
        self.create_service(Trigger, self.get_parameter("reset_service").value,
                            self._reset_cb)

        period = 1.0 / max(1.0, float(
            self.get_parameter("publish_rate").value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"物块 {self.entity} 控制就绪：订阅 "
            f"{self.get_parameter('target_pose_topic').value} / "
            f"{self.get_parameter('target_point_topic').value}，"
            f"初始位姿 {self.initial[:3]}")

    # ── 参数 ──
    def _initial_pose(self):
        try:
            xyz = [float(v) for v in self.get_parameter("initial_pose").value]
        except (TypeError, ValueError):
            xyz = []
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            self.get_logger().warning(
                f"initial_pose 非法，回退到 {DEFAULT_POSE}")
            xyz = list(DEFAULT_POSE)
        try:
            yaw = float(self.get_parameter("initial_yaw").value)
        except (TypeError, ValueError):
            yaw = 0.0
        if not math.isfinite(yaw):
            yaw = 0.0
        return xyz + [yaw]

    # ── 目标输入 ──
    def _on_target_pose(self, msg):
        frame = msg.header.frame_id.strip().lower()
        if frame not in SUPPORTED_FRAMES:
            self.get_logger().warning(
                f"忽略 frame_id={msg.header.frame_id!r} 的目标：本仿真只接受 "
                f"world/base_link（二者与 base_link 重合）")
            return
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._set_target([msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z, yaw])

    def _on_target_point(self, msg):
        self._set_target([msg.x, msg.y, msg.z, self.command[3]])

    def _set_target(self, pose):
        values = [float(v) for v in pose]
        if len(values) != 4 or not all(math.isfinite(v) for v in values):
            self.get_logger().warning("忽略非法目标（坐标必须是有限数值）")
            return
        self.command = values
        self.version += 1
        self.dirty = True
        self._try_send()
        self.get_logger().info(
            "物块目标 -> x={:.3f} y={:.3f} z={:.3f} yaw={:.3f}".format(
                *values), throttle_duration_sec=1.0)

    # ── 下发 / 发布 ──
    def _try_send(self):
        if not self.dirty or self.pending:
            return
        if not self.set_client.service_is_ready():
            return

        state = EntityState()
        state.name = self.entity
        state.reference_frame = self.reference_frame
        state.pose = Pose()
        state.pose.position.x = self.command[0]
        state.pose.position.y = self.command[1]
        state.pose.position.z = self.command[2]
        half = self.command[3] * 0.5
        state.pose.orientation.z = math.sin(half)
        state.pose.orientation.w = math.cos(half)

        self.pending = True
        self._inflight_version = self.version
        future = self.set_client.call_async(
            SetEntityState.Request(state=state))
        future.add_done_callback(self._on_send_done)

    def _on_send_done(self, future):
        self.pending = False
        version = self._inflight_version
        self._inflight_version = -1
        error = future.exception()
        response = None if error is not None else future.result()
        if error is not None or response is None or not response.success:
            if error is not None:
                message = str(error)
            elif response is not None:
                message = response.status_message
            else:
                message = "无响应"
            now = time.monotonic()
            if now - self._last_error_time >= self.error_log_period:
                self._last_error_time = now
                self.get_logger().warning(
                    f"移动物块失败（会继续重试）: {message}")
            return

        # 只清除与本次请求对应的目标；期间若又来了新目标，继续保持 dirty。
        if version == self.version:
            self.dirty = False
        self.get_logger().info(
            f"物块已移到 x={self.command[0]:.3f} "
            f"y={self.command[1]:.3f} z={self.command[2]:.3f}",
            throttle_duration_sec=1.0)

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 仿真里 base_link 与 world 重合；用 base_link 是因为 RViz 的
        # Fixed Frame 就是它，不需要额外的 world->base_link TF。
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(self.command[0])
        msg.pose.position.y = float(self.command[1])
        msg.pose.position.z = float(self.command[2])
        half = self.command[3] * 0.5
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self.pose_pub.publish(msg)

    def _tick(self):
        self._try_send()
        self._publish_pose()

    def _reset_cb(self, _request, response):
        self._set_target(list(self.initial))
        response.success = True
        response.message = (
            f"reset requested: {self.initial[0]:.3f} "
            f"{self.initial[1]:.3f} {self.initial[2]:.3f}")
        return response


def main():
    rclpy.init()
    node = MovableBlock()
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
