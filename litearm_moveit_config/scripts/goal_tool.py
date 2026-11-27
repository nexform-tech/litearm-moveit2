#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""goal_tool.py — 查看当前末端位姿 / 关节角，或给 d435i_supervisor 发目标。

不传 --pose / --joints 时只打印当前状态，适合快速确认机械臂在哪：

    ros2 run litearm_moveit_config goal_tool.py
    ros2 run litearm_moveit_config goal_tool.py --watch

发位姿目标（坐标在 base_link，姿态用 ZYX 固定轴 roll/pitch/yaw，单位 rad）：

    ros2 run litearm_moveit_config goal_tool.py \
        --pose 0.25 0.00 0.25 3.14159 0.0 0.0

发关节目标：

    ros2 run litearm_moveit_config goal_tool.py \
        --joints 0 0 0 -1.5708 0 0 0

回零点（七个关节全 0）：

    ros2 run litearm_moveit_config goal_tool.py --home

目标发布到 supervisor（不是 RViz 的 move_group Execute），执行期间会打印
/d435i_supervisor/status；默认需要输入 y 确认，脚本里加 --yes。
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

JOINTS = [f"joint{i}" for i in range(1, 8)]
TERMINAL_STATES = {"succeeded", "failed", "safe_stop", "retreated"}


def _clamp(value, low, high):
    return max(low, min(high, value))


def quaternion_from_euler(roll, pitch, yaw):
    """固定轴 ZYX 欧拉角 -> 四元数 (x, y, z, w)。"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(q):
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(_clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def format_pose(pose):
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    rpy = [math.degrees(v) for v in rpy_from_quaternion(q)]
    return (
        f"position=({pose.position.x:.4f}, {pose.position.y:.4f}, "
        f"{pose.position.z:.4f}) m; "
        f"rpy=({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}) deg; "
        f"quat=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})")


def _spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def _spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return predicate()


def _pose_from_tf(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w
    return pose


def _try_lookup(buffer, frame, ee_link, state):
    try:
        state["tf"] = buffer.lookup_transform(frame, ee_link, Time())
        return True
    except Exception:  # noqa: BLE001 - TF 未就绪时继续等
        return False


def _print_current(state, ee_link, frame):
    tf = state.get("tf")
    if tf is None:
        print(f"没有 TF {frame} -> {ee_link}；"
              "控制栈 / robot_state_publisher 是否在运行？")
        return
    print(f"当前 {frame} -> {ee_link}:")
    print("  " + format_pose(_pose_from_tf(tf.transform)))
    joints = state.get("joints")
    if joints is not None:
        text = "  ".join(f"{name}={value:.4f}"
                         for name, value in zip(JOINTS, joints))
        print("  关节 (rad): " + text)
    else:
        print("  关节: /joint_states 尚未收到")


def main():
    parser = argparse.ArgumentParser(
        description="查看当前末端位姿/关节角，或向 supervisor 发目标")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--pose", nargs=6, type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="ee_link 目标位姿，base_link 下，米/弧度")
    group.add_argument(
        "--joints", nargs=7, type=float,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        help="七个关节目标角，弧度")
    group.add_argument(
        "--home", action="store_true",
        help="回零点：七个关节全部 0（要回 ready 用 --joints 0 0 0 -1.5708 0 0 0）")
    parser.add_argument("--frame", default="base_link",
                        help="参考坐标系（默认 base_link）")
    parser.add_argument("--ee-link", default="ee_link",
                        help="末端 link（默认 ee_link）")
    parser.add_argument("--goal-topic", default="/d435i_supervisor/goal_pose",
                        help="supervisor 位姿目标话题")
    parser.add_argument("--joint-topic", default="/d435i_supervisor/goal_joints",
                        help="supervisor 关节目标话题")
    parser.add_argument("--status-topic", default="/d435i_supervisor/status",
                        help="supervisor 状态话题")
    parser.add_argument("--watch", action="store_true",
                        help="持续打印当前位姿/关节角")
    parser.add_argument("--yes", action="store_true",
                        help="跳过发送前确认（脚本里用）")
    parser.add_argument("--no-wait", action="store_true",
                        help="发布后不等待 supervisor 状态")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="等待 supervisor 终态的时间（秒，默认 60）")
    parser.add_argument("--tf-timeout", type=float, default=3.0,
                        help="等待当前 TF/关节状态的时间（秒，默认 3）")
    args = parser.parse_args()

    if args.watch and (args.pose is not None or args.joints is not None
                       or args.home):
        parser.error("--watch 不能和 --pose/--joints/--home 同时使用")

    rclpy.init()
    node = Node("goal_tool")
    state = {"tf": None, "joints": None, "status": None}

    def on_joints(msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINTS):
            state["joints"] = [float(positions[name]) for name in JOINTS]

    def on_status(msg):
        state["status"] = msg.data

    buffer = Buffer()
    TransformListener(buffer, node)
    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    node.create_subscription(String, args.status_topic, on_status, 10)

    try:
        if args.watch:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=1.0)
                _try_lookup(buffer, args.frame, args.ee_link, state)
                _print_current(state, args.ee_link, args.frame)
                print()
            return 0

        ready = _spin_until(
            node,
            lambda: _try_lookup(buffer, args.frame, args.ee_link, state)
            and state["joints"] is not None,
            args.tf_timeout)
        if not ready:
            _print_current(state, args.ee_link, args.frame)
            print("当前状态不完整；先确认真机控制栈已启动。")
            return 1

        _print_current(state, args.ee_link, args.frame)

        if args.pose is None and args.joints is None and not args.home:
            return 0

        if args.pose is not None:
            x, y, z, roll, pitch, yaw = args.pose
            q = quaternion_from_euler(roll, pitch, yaw)
            target = Pose()
            target.position.x, target.position.y, target.position.z = x, y, z
            target.orientation.x, target.orientation.y = q[0], q[1]
            target.orientation.z, target.orientation.w = q[2], q[3]
            message = PoseStamped()
            message.header.frame_id = args.frame
            message.pose = target
            publisher = node.create_publisher(
                PoseStamped, args.goal_topic, 10)
            print(f"\n目标 {args.ee_link} @ {args.frame}: {format_pose(target)}")
        else:
            target = (list(args.joints) if args.joints is not None
                      else [0.0] * len(JOINTS))
            message = JointState()
            message.name = list(JOINTS)
            message.position = [float(v) for v in target]
            publisher = node.create_publisher(
                JointState, args.joint_topic, 10)
            label = "回零点" if args.home else "目标关节"
            print(f"\n{label} (rad): " + "  ".join(
                f"{name}={value:.4f}"
                for name, value in zip(JOINTS, target)))

        if not args.yes:
            try:
                answer = input("发送? [y/N] ").strip().lower()
            except EOFError:
                answer = ""
            if answer not in ("y", "yes"):
                print("已取消。")
                return 0

        matched = _spin_until(
            node, lambda: publisher.get_subscription_count() > 0, 3.0)
        if not matched:
            print(f"✗ {publisher.topic_name} 没有订阅者；"
                  "supervisor 是否在运行？")
            return 1

        if hasattr(message, "header"):
            message.header.stamp = node.get_clock().now().to_msg()
        _spin_for(node, 0.3)  # 让 status 订阅完成匹配
        publisher.publish(message)
        print("已发布。")

        if args.no_wait:
            return 0

        last_status = None
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            status = state["status"]
            if status is None or status == last_status:
                continue
            last_status = status
            print("status:", status)
            try:
                payload = json.loads(status)
            except json.JSONDecodeError:
                continue
            if payload.get("state") in TERMINAL_STATES:
                print(f"\n结束: state={payload.get('state')} "
                      f"reason={payload.get('reason', '')} "
                      f"replans={payload.get('replans')}")
                return 0 if payload.get("state") == "succeeded" else 2
        print("等待 supervisor 终态超时；用 ros2 topic echo "
              f"{args.status_topic} 继续观察，或调大 --timeout。")
        return 3
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
