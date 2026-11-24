#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""avoid_supervisor.py — 规划/执行监督：停 → 重规划；失败 → 安全撤退/停靠。

状态机
======
    idle -> planning -> executing -> succeeded
                  ^         |
                  |         +-- 场景变化 -> canceling -> replanning -> executing
                  |                                    |
                  +------------------------------------+
    重规划失败/超次数/控制器 abort/未到目标
        -> retreating（默认目标是 ready 位形，低速）
            成功 -> retreated；失败 -> safe_stop
    规划失败（还没动）/数据过期/传感器过期
        -> safe_stop（不再下发任何轨迹）

安全策略（阶段 4）
==================
  * 数据新鲜度：/joint_states 或（配置了 sensor_topic 时）点云的最后一帧
    超过阈值，拒绝新的运动；执行中变过期则立即取消并保持。阈值 0 表示
    只要求“收到过”。
  * 安全撤退：可配置 retreat_positions（默认 ready = 0,0,0,-1.5708,0,0,0），
    用 retreat_velocity_scaling（默认 0.1）单独规划并执行；撤退路径也要
    通过碰撞检查，失败就停在原地。
  * 速度缩放：正常执行用 velocity_scaling（默认 0.15），撤退用
    retreat_velocity_scaling。JTC 与固件的限幅仍然是硬边界。
  * 急停：软件停靠替代不了物理急停。

接口
====
    goal      sensor_msgs/JointState         /avoid_supervisor/goal
    status    std_msgs/String                /avoid_supervisor/status
              JSON: {"state":..., "replans":n, "busy":bool, "reason":...}
    planned   trajectory_msgs/JointTrajectory /avoid_supervisor/planned_trajectory

⚠ 本节点不做 MoveIt Servo 式的实时避障，语义是「停 → 重规划 → 继续」。
  重规划预算默认 2 s / 5 次尝试；失败后按上面的安全策略撤退或停住。
"""

import json
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.executors import ExternalShutdownException
from moveit_msgs.msg import (Constraints, DisplayTrajectory, JointConstraint,
                             MotionPlanRequest, PlanningScene, RobotState,
                             RobotTrajectory, WorkspaceParameters)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]


class AvoidSupervisor(Node):
    def __init__(self):
        super().__init__("avoid_supervisor")
        self.declare_parameter("group_name", "litearm_arm")
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("acceleration_scaling", 0.15)
        self.declare_parameter("planning_time", 2.0)
        self.declare_parameter("plan_attempts", 5)
        self.declare_parameter("max_replans", 3)
        self.declare_parameter("goal_tolerance", 0.06)
        self.declare_parameter("cancel_settle", 0.3)
        self.declare_parameter("goal_topic", "/avoid_supervisor/goal")
        self.declare_parameter("status_topic", "/avoid_supervisor/status")
        self.declare_parameter("trajectory_topic", "/avoid_supervisor/planned_trajectory")
        self.declare_parameter("display_trajectory_topic", "/display_planned_path")

        # 阶段 4：安全
        self.declare_parameter("max_state_age", 1.0)
        self.declare_parameter("sensor_topic", "")
        self.declare_parameter("max_sensor_age", 0.0)
        self.declare_parameter("retreat_enabled", True)
        self.declare_parameter("retreat_positions",
                               [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0])
        self.declare_parameter("retreat_velocity_scaling", 0.1)
        self.declare_parameter("clear_octomap_on_stale", True)

        self.state = "idle"
        self.reason = ""
        self.busy = False
        self.replans = 0
        self.goal = None
        self.current = {}
        self.scene_changed = False
        self.cancel_requested = False
        self.result_received = False
        self.plan_deadline = 0.0
        self.replan_due = 0.0
        self.active_handle = None
        self.pending_trajectory = None
        self.pending_state = "executing"
        self.plan_kind = ""
        self.retreat_reason = ""
        self.exec_seq = 0
        self.last_joint_time = None
        self.last_sensor_time = None

        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.clear_octomap_client = self.create_client(Empty, "/clear_octomap")
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")
        self.goal_sub = self.create_subscription(
            JointState, self.get_parameter("goal_topic").value, self._on_goal, 10)
        self.scene_sub = self.create_subscription(
            PlanningScene, "/monitored_planning_scene", self._on_scene, 10)
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10)
        sensor_topic = self.get_parameter("sensor_topic").value
        if sensor_topic:
            self.sensor_sub = self.create_subscription(
                PointCloud2, sensor_topic, self._on_sensor,
                qos_profile_sensor_data)
        else:
            self.sensor_sub = None
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        # RViz 的 MoveIt MotionPlanning 面板订阅这个话题；每次规划/重规划都发一份，
        # 便于在 RViz 里直接看到“新路径长什么样”。
        self.display_pub = self.create_publisher(
            DisplayTrajectory,
            self.get_parameter("display_trajectory_topic").value, 10)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.2, self._publish_status)
        retreat = list(self.get_parameter("retreat_positions").value)
        self.retreat_ok = len(retreat) == len(JOINTS)
        if not self.retreat_ok:
            self.get_logger().error("retreat_positions 必须 7 个值，撤退已禁用")
        self.get_logger().info(
            "avoid_supervisor 就绪（安全策略: max_state_age="
            f"{self.get_parameter('max_state_age').value}s, sensor="
            f"{sensor_topic or '未配置'}, retreat="
            f"{self.get_parameter('retreat_enabled').value}）")

    # ── 工具 ──
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _status(self, state, reason=""):
        self.state = state
        self.reason = reason
        if state in ("succeeded", "safe_stop", "failed", "retreated"):
            self.busy = False
        self._publish_status()
        self.get_logger().info(
            f"state={state} replans={self.replans} reason={reason or '-'}")

    def _publish_status(self):
        payload = {"state": self.state, "replans": self.replans, "busy": self.busy}
        if self.reason:
            payload["reason"] = self.reason
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _freshness_error(self):
        age = (None if self.last_joint_time is None
               else self._now() - self.last_joint_time)
        max_age = float(self.get_parameter("max_state_age").value)
        if age is None or (max_age > 0.0 and age > max_age):
            return "stale_joint_states"
        sensor_topic = self.get_parameter("sensor_topic").value
        max_sensor_age = float(self.get_parameter("max_sensor_age").value)
        if sensor_topic and max_sensor_age > 0.0:
            sensor_age = (None if self.last_sensor_time is None
                          else self._now() - self.last_sensor_time)
            if sensor_age is None or sensor_age > max_sensor_age:
                return "stale_sensor"
        return ""

    # ── 输入 ──
    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current[name] = position
        self.last_joint_time = self._now()

    def _on_sensor(self, _msg):
        self.last_sensor_time = self._now()

    def _on_goal(self, msg):
        if self.busy:
            self.get_logger().warning("收到新目标，但当前仍在执行；忽略")
            return
        error = self._freshness_error()
        if error:
            self.get_logger().error(f"拒绝运动：{error}")
            self._status("failed", error)
            return
        positions = dict(zip(msg.name, msg.position))
        missing = [name for name in JOINTS if name not in positions]
        if missing:
            self.get_logger().error(f"目标缺少关节 {missing}，忽略")
            return
        self.goal = [float(positions[name]) for name in JOINTS]
        self.replans = 0
        self.scene_changed = False
        self.cancel_requested = False
        self.busy = True
        self.get_logger().info(f"新目标: {[round(v, 3) for v in self.goal]}")
        self._start_plan("initial")

    def _on_scene(self, msg):
        # 只关心世界碰撞对象的变化；机器人状态更新不触发。
        if not msg.world.collision_objects:
            return
        if self.busy and self.state in ("planning", "replanning",
                                        "waiting_controller", "executing"):
            if not self.scene_changed:
                self.get_logger().info("检测到场景变化，准备重规划")
            self.scene_changed = True

    # ── 规划 ──
    def _start_plan(self, kind):
        error = self._freshness_error()
        if error:
            self._safe_stop(error)
            return
        if kind == "replan":
            self.replans += 1
        self.plan_kind = kind
        self.scene_changed = False
        self.cancel_requested = False
        state = {"initial": "planning", "replan": "replanning",
                 "retreat": "retreat_planning"}[kind]
        self._status(state, self.retreat_reason if kind == "retreat" else "")
        self.plan_deadline = self._now() + float(
            self.get_parameter("planning_time").value) + 5.0
        scaling = (float(self.get_parameter("retreat_velocity_scaling").value)
                   if kind == "retreat"
                   else float(self.get_parameter("velocity_scaling").value))
        future = self.plan_client.call_async(
            GetMotionPlan.Request(
                motion_plan_request=self._plan_request(scaling)))
        future.add_done_callback(lambda done: self._plan_done(done, kind))

    def _plan_request(self, scaling):
        request = MotionPlanRequest()
        request.group_name = self.get_parameter("group_name").value
        request.pipeline_id = "ompl"
        request.planner_id = "RRTConnectkConfigDefault"
        request.num_planning_attempts = int(self.get_parameter("plan_attempts").value)
        request.allowed_planning_time = float(self.get_parameter("planning_time").value)
        request.max_velocity_scaling_factor = scaling
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value)

        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0

        request.start_state = RobotState()
        request.start_state.is_diff = True

        constraints = Constraints()
        for name, value in zip(JOINTS, self.goal):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = value
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)
        return request

    def _plan_done(self, future, kind):
        expected = {"initial": "planning", "replan": "replanning",
                    "retreat": "retreat_planning"}[kind]
        if self.state != expected or self.plan_kind != kind:
            return
        response = future.result()
        trajectory = None
        if response is not None and response.motion_plan_response.error_code.val == 1:
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
        if trajectory is None or not trajectory.points:
            if kind == "retreat":
                self._safe_stop("retreat_failed")
            elif kind == "replan":
                self._fail("replan_failed")
            else:
                self._safe_stop("plan_failed")
            return
        if self.scene_changed and kind != "retreat":
            self.scene_changed = False
            if kind == "replan" and self.replans >= int(
                    self.get_parameter("max_replans").value):
                self._fail("max_replans")
            else:
                self._start_plan("replan")
            return

        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "base_link"
        self.trajectory_pub.publish(trajectory)
        self._publish_display(trajectory)
        self._start_execution(
            trajectory, "retreating" if kind == "retreat" else "executing")

    def _publish_display(self, trajectory):
        display = DisplayTrajectory()
        display.model_id = ""
        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = trajectory
        display.trajectory.append(robot_trajectory)
        display.trajectory_start.is_diff = False
        display.trajectory_start.joint_state.name = list(JOINTS) + [
            "gripper_opening_joint"]
        display.trajectory_start.joint_state.position = [
            float(self.current.get(name, 0.0)) for name in JOINTS] + [0.0]
        self.display_pub.publish(display)

    # ── 执行 ──
    def _start_execution(self, trajectory, state_name):
        self.pending_trajectory = trajectory
        self.pending_state = state_name
        self._status(state_name)
        if not self.action_client.server_is_ready():
            self._status("waiting_controller")
            return
        self._send_trajectory()

    def _send_trajectory(self):
        self.exec_seq += 1
        seq = self.exec_seq
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.pending_trajectory
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._goal_response(done, seq))

    def _goal_response(self, future, seq):
        if seq != self.exec_seq:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._safe_stop("controller_rejected_goal")
            return
        if self.cancel_requested:
            handle.cancel_goal_async()
            return
        self.active_handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._exec_result(done, seq))

    def _exec_result(self, future, seq):
        if seq != self.exec_seq:
            return
        if self.state == "canceling":
            self.result_received = True
            return
        if self.state not in ("executing", "retreating"):
            return
        result = future.result()
        code = result.result.error_code if result is not None else -999
        if self.state == "retreating":
            self.active_handle = None
            if code != 0:
                self._safe_stop(f"retreat_aborted:{code}")
                return
            error = max(abs(self.current.get(name, 1e9) - value)
                        for name, value in zip(JOINTS, self.goal))
            if error > float(self.get_parameter("goal_tolerance").value):
                self._safe_stop(f"retreat_not_reached:{error:.3f}")
                return
            self._status("retreated", self.retreat_reason)
            return
        self.active_handle = None
        if code != 0:
            self._fail(f"controller_aborted:{code}")
            return
        error = max(abs(self.current.get(name, 1e9) - value)
                    for name, value in zip(JOINTS, self.goal))
        if error > float(self.get_parameter("goal_tolerance").value):
            self._fail(f"goal_not_reached:{error:.3f}")
            return
        self._status("succeeded")

    # ── 取消 / 重规划 / 失败处理 ──
    def _begin_cancel_and_replan(self):
        self._status("canceling", "scene_changed")
        self.cancel_requested = True
        self.result_received = False
        self.replan_due = self._now() + float(
            self.get_parameter("cancel_settle").value)
        if self.active_handle is not None:
            self.active_handle.cancel_goal_async()

    def _fail(self, reason):
        """执行已经停止（没有活动 goal）后的失败处理：尽量撤退，否则停住。"""
        if (self.retreat_ok
                and bool(self.get_parameter("retreat_enabled").value)
                and self.active_handle is None
                and not self._freshness_error()):
            self.retreat_reason = reason
            self.goal = [float(v) for v in
                         self.get_parameter("retreat_positions").value]
            self._start_plan("retreat")
        else:
            self._safe_stop(reason)

    def _clear_octomap_if_stale(self, reason):
        """数据过期后 Octomap 可能掺入自滤波错误/陈旧点，不再可信，清掉。"""
        if not reason.startswith("stale_"):
            return
        if not bool(self.get_parameter("clear_octomap_on_stale").value):
            return
        if not self.clear_octomap_client.service_is_ready():
            return
        future = self.clear_octomap_client.call_async(Empty.Request())
        future.add_done_callback(
            lambda _done: self.get_logger().info("数据过期，已清除 Octomap"))

    def _safe_stop(self, reason):
        self.cancel_requested = True
        if self.active_handle is not None:
            self.active_handle.cancel_goal_async()
            self.active_handle = None
        self.pending_trajectory = None
        self._clear_octomap_if_stale(reason)
        self._status("safe_stop", reason)

    def _tick(self):
        now = self._now()

        # 数据新鲜度看门狗：运动中任一来源过期 -> 立即取消并停住。
        if self.busy:
            error = self._freshness_error()
            if error:
                self._safe_stop(error)
                return

        if self.state in ("planning", "replanning", "retreat_planning") \
                and now > self.plan_deadline:
            if self.state == "retreat_planning":
                self._safe_stop("retreat_timeout")
            elif self.state == "replanning":
                self._fail("plan_timeout")
            else:
                self._safe_stop("plan_timeout")
            return

        if self.state == "waiting_controller":
            if self.scene_changed and self.pending_state == "executing":
                self.scene_changed = False
                if self.replans >= int(self.get_parameter("max_replans").value):
                    self._fail("max_replans")
                else:
                    self._start_plan("replan")
                return
            if self.action_client.server_is_ready():
                self._status(self.pending_state)
                self._send_trajectory()
            return

        if self.state == "executing" and self.scene_changed \
                and not self.cancel_requested:
            if self.replans >= int(self.get_parameter("max_replans").value):
                # 还在执行，不能直接撤退；先取消，随后停在原地。
                self.scene_changed = False
                self._safe_stop("max_replans")
            else:
                self.scene_changed = False
                self._begin_cancel_and_replan()
            return

        if self.state == "canceling":
            if self.result_received or now >= self.replan_due:
                self.active_handle = None
                self.cancel_requested = False
                self.result_received = False
                self._start_plan("replan")


def main():
    rclpy.init()
    node = AvoidSupervisor()
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
