#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""move_block.py — 一条命令移动 Gazebo 里的演示物块。

物块由 launch 默认插入，movable_block.py 负责改位。本脚本只发一条
PoseStamped 到 /movable_block/target_pose，然后退出；物块移动后 D435i
立即在新位置渲染，/camera/depth/color/points 与 Octomap 实时更新。

示例（另开终端，export 同样的 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY）
====================================================================
  # 摆到工作台上方（world 与 base_link 重合）
  ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20

  # 再绕 Z 轴转 45 度
  ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20 --yaw 0.785

  # 回初始停车位
  ros2 run litearm_d435i_perception move_block.py --reset

退出码 0 = 目标已发出（或 reset 成功）。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


def _build_parser():
    parser = argparse.ArgumentParser(
        description="移动 Gazebo 演示物块（movable_block）")
    parser.add_argument("xyz", nargs="*", type=float, metavar="XYZ",
                        help="目标位置 x y z（m）")
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="绕 Z 轴朝向（rad，默认 0）")
    parser.add_argument("--frame", default="world",
                        help="目标坐标系，world 或 base_link（默认 world）")
    parser.add_argument("--topic", default="/movable_block/target_pose",
                        help="目标位姿话题")
    parser.add_argument("--reset", action="store_true",
                        help="回 initial_pose，不要求 xyz")
    parser.add_argument("--reset-service", default="/movable_block/reset",
                        help="reset 服务名")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="等待订阅者/服务的超时（秒）")
    return parser


def _spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def _reset(node, service, timeout):
    client = node.create_client(Trigger, service)
    if not _spin_until(node, client.service_is_ready, timeout):
        print(f"✗ 服务不可用: {service}（launch 是否在运行？"
              f"ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 是否一致？）",
              file=sys.stderr)
        return 1
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        print("✗ reset 调用超时", file=sys.stderr)
        return 1
    response = future.result()
    if not response.success:
        print(f"✗ reset 失败: {response.message}", file=sys.stderr)
        return 1
    print(f"✓ {response.message}")
    return 0


def _publish(node, topic, xyz, yaw, frame, timeout):
    publisher = node.create_publisher(PoseStamped, topic, 10)
    if not _spin_until(node, lambda: publisher.get_subscription_count() > 0,
                       timeout):
        print(f"✗ 没有节点订阅 {topic}（launch 是否在运行？"
              f"ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 是否一致？）",
              file=sys.stderr)
        return 1

    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    half = float(yaw) * 0.5
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)

    # 连发几帧：订阅关系刚建立时单帧偶发丢失，这里留出容错。
    for _ in range(3):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"✓ 物块目标已发送: x={xyz[0]:.3f} y={xyz[1]:.3f} "
          f"z={xyz[2]:.3f} yaw={yaw:.3f} frame={frame}")
    return 0


def main():
    parser = _build_parser()
    args, ros_args = parser.parse_known_args()

    if not args.reset:
        if len(args.xyz) != 3:
            parser.error("需要 3 个坐标 X Y Z，或使用 --reset")
        if not all(math.isfinite(v) for v in args.xyz):
            parser.error("坐标必须是有限数值")
    if not math.isfinite(args.yaw):
        parser.error("--yaw 必须是有限数值")

    rclpy.init(args=ros_args)
    node = Node("move_block_cli")
    try:
        if args.reset:
            return _reset(node, args.reset_service, args.timeout)
        return _publish(node, args.topic, args.xyz, args.yaw, args.frame,
                        args.timeout)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
