#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""d435i_probe.py — D435i 感知 + 规划 + 动态障碍重规划的验收探针。

检查顺序
========
  1. 相机点云在发、/monitored_planning_scene 里出现非空 Octomap，
     且 supervisor 已确认地图建立（require_octomap 门槛）。
  2. 用 /compute_fk 把已知可达关节位形换算成 ee_link 位姿，作为
     PoseStamped 目标发给 supervisor；验证它经 /compute_ik 规划并执行到
     同一位置（位置误差 < 5 cm，姿态误差 < 0.2 rad）。
  3. 回到 ready 位形。
  4. 再发同一个位姿目标；执行开始后把 Gazebo 里的 dynamic_obstacle 移到
     轨迹中段的 ee_link 位姿，等待 supervisor 报告 replans >= 1 且最终
     succeeded。
     —— 这一步证明“相机看到新障碍 -> Octomap 更新 -> 剩余轨迹复检 ->
        取消 -> 重规划 -> 继续执行”整条链路是通的。
     若动态障碍没有启动（start_dynamic_obstacle:=false，默认），第 4/5 步
     自动跳过，退出码仍为 0；此时验收覆盖 1-3（感知 + 静态位姿）。

退出码 0 = 全部通过。
"""

import argparse
import json
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]
GROUP = "litearm_arm"
EE_LINK = "ee_link"
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]
READY = [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]

POSITION_TOLERANCE = 0.05   # m
ORIENTATION_TOLERANCE = 0.20  # rad


def quaternion_angle(q1, q2):
    dot = abs(q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w)
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


class D435iProbe(Node):
    def __init__(self):
        super().__init__("d435i_probe")
        # use_sim_time 由 rclpy 作为内置参数声明，不能再 declare 一次。
        self.declare_parameter("pose_goal_topic",
                               "/d435i_supervisor/goal_pose")
        self.declare_parameter("joint_goal_topic",
                               "/d435i_supervisor/goal_joints")
        self.declare_parameter("status_topic", "/d435i_supervisor/status")
        self.declare_parameter("dynamic_reset_service",
                               "/dynamic_obstacle/reset")

        self.cloud_count = 0
        self.octomap_seen = False
        self.status = {}
        self.joints = {}
        self.last_status = ""
        self.status_seq = 0
        self.max_replans = 0
        self.scene_updates = 0
        self.trajectory = None
        self.trajectory_seq = 0

        self.create_subscription(PointCloud2, "/camera/depth/color/points",
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(PlanningScene, "/monitored_planning_scene",
                                 self._on_scene, 10)
        self.create_subscription(String, self.get_parameter(
            "status_topic").value, self._on_status, 10)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)
        self.create_subscription(
            JointTrajectory, "/d435i_supervisor/planned_trajectory",
            self._on_trajectory, 10)

        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_goal_topic").value, 10)
        self.joint_pub = self.create_publisher(
            JointState, self.get_parameter("joint_goal_topic").value, 10)

        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.reset_client = self.create_client(
            Trigger, self.get_parameter("dynamic_reset_service").value)
        self.set_state_client = self.create_client(
            SetEntityState, "/gazebo/set_entity_state")

    # ── 回调 ──
    def _on_cloud(self, _msg):
        self.cloud_count += 1

    def _on_scene(self, msg):
        if len(msg.world.octomap.octomap.data) > 0:
            self.octomap_seen = True

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.status_seq += 1
        self.last_status = msg.data
        self.max_replans = max(self.max_replans,
                               int(self.status.get("replans", 0)))
        self.scene_updates = max(self.scene_updates,
                                 int(self.status.get("scene_updates", 0)))

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    def _on_trajectory(self, msg):
        self.trajectory = msg
        self.trajectory_seq += 1

    # ── 工具 ──
    def _spin_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f"超时：{description}")
        return False

    def _call(self, client, request, timeout, description):
        if not client.service_is_ready():
            if not self._spin_for(client.service_is_ready, timeout,
                                  f"{description} 服务就绪"):
                return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f"超时：{description}")
            return None
        return future.result()

    def _fk(self, positions):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = [EE_LINK]
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = [float(v) for v in positions]
        response = self._call(self.fk_client, request, 10.0, "compute_fk")
        if response is None or response.error_code.val != 1 \
                or not response.pose_stamped:
            return None
        return response.pose_stamped[0].pose

    def _send_joint_goal(self, positions):
        msg = JointState()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in positions]
        self.joint_pub.publish(msg)

    def _send_pose_goal(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        self.pose_pub.publish(msg)

    def _wait_for_trajectory(self, seq, timeout, description):
        def ready():
            return (self.trajectory_seq > seq and self.trajectory is not None
                    and len(self.trajectory.points) > 0)
        return self._spin_for(ready, timeout, description)

    def _wait_started(self, seq, timeout, description):
        """等 supervisor 对新目标的一条状态：已接收，或直接拒动。"""
        def started():
            if self.status_seq <= seq:
                return False
            return bool(self.status.get("busy")) \
                or self.status.get("state") in ("failed", "safe_stop")
        return self._spin_for(started, timeout, description)

    def _wait_terminal(self, timeout, description):
        def done():
            return (self.status.get("state") in
                    ("succeeded", "failed", "safe_stop", "retreated")
                    and not self.status.get("busy", True))
        return self._spin_for(done, timeout, description)

    def _wait_executing(self, timeout, description):
        def executing():
            return self.status.get("state") in (
                "executing", "waiting_controller")
        return self._spin_for(executing, timeout, description)

    def _joint_error(self, target):
        if not all(name in self.joints for name in JOINTS):
            return float("inf")
        return max(abs(self.joints[name] - value)
                   for name, value in zip(JOINTS, target))

    # ── 测试步骤 ──
    def run(self, timeout, skip_dynamic):
        self.get_logger().info("等待相机点云 / Octomap / supervisor ...")
        if not self._spin_for(
                lambda: self.cloud_count > 0 and self.octomap_seen
                and self.status.get("octomap_seen") is True
                and all(name in self.joints for name in JOINTS)
                # 等地图稳定几帧再发第一个目标；否则首次规划可能在
                # 稀疏/抖动的地图上失败。
                and int(self.status.get("scene_updates", 0)) >= 5,
                timeout, "点云、Octomap 与关节状态稳定"):
            return False
        self.get_logger().info(
            f"感知就绪：点云 {self.cloud_count} 帧，Octomap 已建立")

        # 先回 ready：零位在奇异附近，KDL 从零位求 IK 容易 NO_IK_SOLUTION，
        # 同时这一步也验证关节目标规划/执行链路。
        self.get_logger().info("先移动到 ready（作为 IK 种子）...")
        seq = self.status_seq
        self._send_joint_goal(READY)
        if not self._wait_started(seq, timeout, "supervisor 接收 ready 目标"):
            return False
        if not self._wait_terminal(timeout, "移动到 ready"):
            return False
        if self.status.get("state") != "succeeded":
            self.get_logger().error(f"移动到 ready 失败: {self.last_status}")
            return False

        goal_pose = self._fk(GOAL)
        if goal_pose is None:
            self.get_logger().error("无法计算目标位姿的 FK")
            return False
        self.get_logger().info(
            f"目标位姿: x={goal_pose.position.x:.3f} "
            f"y={goal_pose.position.y:.3f} z={goal_pose.position.z:.3f}")

        # 1) 位姿目标：规划 + 执行
        seq = self.status_seq
        self._send_pose_goal(goal_pose)
        if not self._wait_started(seq, timeout, "supervisor 接收位姿目标"):
            return False
        if not self._wait_terminal(timeout, "位姿目标执行完成"):
            return False
        if self.status.get("state") != "succeeded":
            self.get_logger().error(f"位姿目标失败: {self.last_status}")
            return False
        reached = self._fk([self.joints.get(n, 0.0) for n in JOINTS])
        if reached is None:
            return False
        position_error = math.sqrt(
            (reached.position.x - goal_pose.position.x) ** 2
            + (reached.position.y - goal_pose.position.y) ** 2
            + (reached.position.z - goal_pose.position.z) ** 2)
        orientation_error = quaternion_angle(reached.orientation,
                                             goal_pose.orientation)
        self.get_logger().info(
            f"到位误差: {position_error * 1000:.1f} mm, "
            f"{orientation_error:.3f} rad")
        if position_error > POSITION_TOLERANCE \
                or orientation_error > ORIENTATION_TOLERANCE:
            self.get_logger().error("位姿误差超限")
            return False

        # 2) 回 ready
        seq = self.status_seq
        self._send_joint_goal(READY)
        if not self._wait_started(seq, timeout, "supervisor 接收 ready 目标"):
            return False
        if not self._wait_terminal(timeout, "回到 ready"):
            return False
        if self.status.get("state") != "succeeded":
            self.get_logger().error(f"回 ready 失败: {self.last_status}")
            return False

        if skip_dynamic:
            self.get_logger().info("跳过动态障碍测试 (--skip-dynamic)")
            return True

        # 动态障碍默认不启动；没有 /dynamic_obstacle/reset 就跳过，
        # 验收只覆盖感知 + 静态位姿。
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline                 and not self.reset_client.service_is_ready():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.reset_client.service_is_ready():
            self.get_logger().info(
                "动态障碍未启动（start_dynamic_obstacle:=false），跳过动态测试")
            return True

        # 3) 动态障碍：执行中插入 -> 重规划
        self._call(self.reset_client, Trigger.Request(), 10.0,
                   "dynamic_obstacle/reset")
        before_replans = self.max_replans
        before_scene = self.scene_updates
        seq = self.status_seq
        traj_seq = self.trajectory_seq
        self._send_pose_goal(goal_pose)
        if not self._wait_started(seq, timeout, "supervisor 接收动态测试目标"):
            return False
        if not self._wait_executing(timeout, "开始执行动态障碍测试轨迹"):
            return False
        if not self._wait_for_trajectory(traj_seq, timeout, "拿到本轮规划轨迹"):
            return False

        # 取轨迹中点附近的 ee_link 位姿：它在剩余路径上、又离终点足够远，
        # 重规划有空间绕过去。把 Gazebo 里的障碍直接移到这里——相机照到它，
        # Octomap 更新，supervisor 复检剩余轨迹时判定被挡。
        names = list(self.trajectory.joint_names)
        index = max(0, min(len(self.trajectory.points) - 1,
                           int(len(self.trajectory.points) * 0.5)))
        point = self.trajectory.points[index]
        by_name = dict(zip(names, point.positions))
        positions = [float(by_name.get(name, self.joints.get(name, 0.0)))
                     for name in JOINTS]
        obstacle_pose = self._fk(positions)
        if obstacle_pose is None:
            self.get_logger().error("无法计算障碍注入位姿")
            return False
        self.get_logger().info(
            "轨迹执行中，把 Gazebo 障碍移到未来路点 "
            f"#{index}/{len(self.trajectory.points) - 1}: "
            f"x={obstacle_pose.position.x:.3f} "
            f"y={obstacle_pose.position.y:.3f} "
            f"z={obstacle_pose.position.z:.3f}")
        # 盒子中心放在未来 ee_link 路点上，+x 面（朝向相机）在路点前方
        # 0.07 m，不会被夹爪完全挡住；相机能看到它，Octomap 才会记录。
        # 障碍模型是 visual-only，所以重规划晚一拍也不会让 JTC 物理 abort。
        # 本仿真里机器人基座固定在 world 原点，base_link 与 world 重合，
        # /compute_fk 的 base_link 位姿可以直接当成 Gazebo world 位姿。
        state = EntityState()
        state.name = "dynamic_obstacle"
        state.reference_frame = "world"
        state.pose = obstacle_pose
        response = self._call(
            self.set_state_client, SetEntityState.Request(state=state), 10.0,
            "gazebo/set_entity_state")
        if response is None or not response.success:
            self.get_logger().error(
                "移动动态障碍失败: "
                f"{None if response is None else response.status_message}")
            return False

        if not self._wait_terminal(timeout * 2, "动态障碍下的重规划完成"):
            return False
        final_state = self.status.get("state")
        self.get_logger().info(
            f"动态测试结束: state={final_state} "
            f"replans={self.max_replans} scene_updates={self.scene_updates}")

        if final_state != "succeeded":
            self.get_logger().error(
                f"动态障碍下最终未到达目标: {self.last_status}")
            return False
        if self.max_replans <= before_replans:
            self.get_logger().error("没有发生重规划，动态障碍可能没有进入路径")
            return False
        if self.scene_updates <= before_scene:
            self.get_logger().error("场景没有更新，相机可能没看到动态障碍")
            return False

        # 收尾：障碍回 park
        self._call(self.reset_client, Trigger.Request(), 10.0,
                   "dynamic_obstacle/reset")
        self.get_logger().info(
            f"动态障碍重规划通过：replans {before_replans} -> "
            f"{self.max_replans}")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0,
                        help="单步超时（秒）")
    parser.add_argument("--skip-dynamic", action="store_true",
                        help="只验证感知与静态避障，不测动态重规划")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = D435iProbe()
    try:
        ok = node.run(args.timeout, args.skip_dynamic)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("✓ d435i 验收通过" if ok else "✗ d435i 验收失败")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
