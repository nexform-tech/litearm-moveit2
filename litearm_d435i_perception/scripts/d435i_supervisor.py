#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""d435i_supervisor.py — 相机场景下的规划 / 执行 / 重规划监督。

和 litearm_obstacle_avoidance/avoid_supervisor.py 的区别
=======================================================
旧 supervisor 用「world.collision_objects 变化」作为触发条件；相机链路
的场景变化主要来自 **Octomap**（world.octomap），而且点云每帧都在刷新，
不能一有更新就无脑重规划。本节点因此改成：

    1. 先按目标规划并执行；
    2. 执行中监听 /monitored_planning_scene；Octomap 或碰撞对象有更新时，
       只对**剩余轨迹**做 /check_state_validity 复检（限频、降采样）；
    3. 剩余轨迹真的被挡住 -> 取消当前 JTC goal -> 从当前状态重规划 ->
       继续执行；
    4. 重规划次数超限 / 规划失败 -> 低速撤退到 ready，撤退也失败则停住。

目标输入
========
    /d435i_supervisor/goal_pose    geometry_msgs/PoseStamped
        base_link 下的 ee_link 目标位姿，先经 /compute_ik 求关节解。
    /d435i_supervisor/goal_joints  sensor_msgs/JointState
        直接给七轴关节目标（探针/脚本用）。

输出
====
    /d435i_supervisor/status                 std_msgs/String (JSON)
    /d435i_supervisor/planned_trajectory     trajectory_msgs/JointTrajectory
    /d435i_supervisor/cancel                 std_srvs/Trigger

安全
====
    * /joint_states 或传感器心跳（默认 /camera/color/camera_info）过期
      -> 拒绝新目标；执行中则取消并停住。订阅的是 CameraInfo 而不是
      整份点云，避免每帧反序列化几百 KB 的 PointCloud2。
    * require_octomap=true 时，在收到第一份非空 Octomap 前拒绝运动，
      避免“以为在避障、其实地图是空的”。
    * 这是「停 -> 复检 -> 重规划」，不是 MoveIt Servo 那种毫秒级实时避障。
"""

import json
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningScene, RobotState, WorkspaceParameters)
from moveit_msgs.srv import (GetMotionPlan, GetPositionIK, GetStateValidity)
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]


class D435iSupervisor(Node):
    def __init__(self):
        super().__init__("d435i_supervisor")
        self.declare_parameter("group_name", "litearm_arm")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("velocity_scaling", 0.20)
        self.declare_parameter("acceleration_scaling", 0.20)
        self.declare_parameter("planning_time", 2.0)
        self.declare_parameter("plan_attempts", 5)
        self.declare_parameter("max_replans", 5)
        self.declare_parameter("goal_tolerance", 0.06)
        self.declare_parameter("cancel_settle", 0.3)
        self.declare_parameter("ik_timeout", 0.5)

        # 数据新鲜度与“地图已建立”门槛
        self.declare_parameter("max_state_age", 1.0)
        self.declare_parameter("sensor_topic", "/camera/color/camera_info")
        self.declare_parameter("max_sensor_age", 1.0)
        self.declare_parameter("require_sensor", True)
        self.declare_parameter("require_octomap", True)

        # 剩余轨迹复检
        self.declare_parameter("validity_check_period", 0.5)
        self.declare_parameter("validity_lookahead", 0.4)
        self.declare_parameter("validity_stride", 3)
        # base_link 是固定基座，和台面/安装板天然接触；规划器按规划组检查
        # 时不会把它当成可避让碰撞。复检如果检查全部 link，会每个轨迹点都
        # 判无效，导致取消—重规划—再取消的抖动。默认忽略只涉及 base_link
        # 的接触；涉及其他 link 的接触仍然照常拦截。
        self.declare_parameter("ignore_base_link_contacts", True)

        # 失败后的低速撤退
        self.declare_parameter("retreat_enabled", True)
        self.declare_parameter("retreat_positions",
                               [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0])
        self.declare_parameter("retreat_velocity_scaling", 0.1)
        self.declare_parameter("clear_octomap_on_stale", True)

        # 接口
        self.declare_parameter("pose_goal_topic", "/d435i_supervisor/goal_pose")
        self.declare_parameter("joint_goal_topic",
                               "/d435i_supervisor/goal_joints")
        self.declare_parameter("status_topic", "/d435i_supervisor/status")
        self.declare_parameter("trajectory_topic",
                               "/d435i_supervisor/planned_trajectory")

        # ── 状态 ──
        self.state = "idle"
        self.reason = ""
        self.busy = False
        self.replans = 0
        self.goal = None
        self.goal_source = ""
        self.current = {}
        self.last_joint_time = None
        self.last_cloud_time = None
        self.cloud_seen = False
        self.octomap_seen = False
        self.scene_dirty = False
        self.scene_updates = 0
        self.last_collision_sig = None
        self.validity_checks = 0
        self.path_invalid = False
        self.check_queue = []
        self.check_pending = False
        self.current_check_index = None
        self.last_check_time = 0.0
        self.trajectory = None
        self.exec_start_time = None
        self.pending_trajectory = None
        self.pending_state = "executing"
        self.plan_kind = ""
        self.plan_deadline = 0.0
        self.retreat_reason = ""
        self.cancel_requested = False
        self.result_received = False
        self.replan_due = 0.0
        self.active_handle = None
        self.exec_seq = 0
        self.last_status_time = 0.0
        self.last_log_key = None
        self.last_log_time = 0.0
        self.last_base_ignore_log = 0.0

        # ── 接口 ──
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity")
        self.clear_octomap_client = self.create_client(Empty, "/clear_octomap")
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")

        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10)

        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)
        self.create_subscription(PlanningScene, "/monitored_planning_scene",
                                 self._on_scene, 10)
        sensor_topic = self.get_parameter("sensor_topic").value
        if sensor_topic:
            # 只用来判断传感器是否还在发；用 CameraInfo（KB 级）而不是
            # PointCloud2（几百 KB、Python 每帧反序列化很贵）。
            self.create_subscription(CameraInfo, sensor_topic,
                                     self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.get_parameter("pose_goal_topic").value,
            self._on_pose_goal, 10)
        self.create_subscription(
            JointState, self.get_parameter("joint_goal_topic").value,
            self._on_joint_goal, 10)
        self.create_service(Trigger, "/d435i_supervisor/cancel",
                            self._cancel_cb)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"d435i_supervisor 就绪: group={self.get_parameter('group_name').value}, "
            f"sensor={sensor_topic or '无'}, "
            f"require_octomap={bool(self.get_parameter('require_octomap').value)}")

    # ── 工具 ──
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _status(self, state, reason=""):
        self.state = state
        self.reason = reason
        if state in ("succeeded", "failed", "safe_stop", "retreated"):
            self.busy = False
        self._publish_status()

    def _publish_status(self):
        now = self._now()
        if now - self.last_status_time < 0.1 and self.state not in (
                "succeeded", "failed", "safe_stop", "retreated"):
            return
        self.last_status_time = now
        payload = {
            "state": self.state,
            "busy": self.busy,
            "replans": self.replans,
            "reason": self.reason,
            "goal_source": self.goal_source,
            "octomap_seen": self.octomap_seen,
            "cloud_seen": self.cloud_seen,
            "scene_updates": self.scene_updates,
            "validity_checks": self.validity_checks,
            "path_invalid": self.path_invalid,
        }
        if self.goal is not None:
            payload["goal"] = [round(v, 3) for v in self.goal]
        if self.last_joint_time is not None:
            payload["joint_age"] = round(now - self.last_joint_time, 3)
        if self.last_cloud_time is not None:
            payload["cloud_age"] = round(now - self.last_cloud_time, 3)
        self.status_pub.publish(String(data=json.dumps(payload)))

        # 状态 topic 保持 10 Hz（探针要看实时状态），日志只在状态变化或
        # 每 2 s 打一条，否则执行期间会刷屏。
        key = (self.state, self.reason)
        if key != self.last_log_key or now - self.last_log_time >= 2.0:
            self.last_log_key = key
            self.last_log_time = now
            self.get_logger().info(
                f"state={self.state} replans={self.replans} "
                f"reason={self.reason or '-'}")

    def _freshness_error(self):
        now = self._now()
        if not self.current:
            return "no_joint_states"
        max_age = float(self.get_parameter("max_state_age").value)
        if max_age > 0.0 and now - self.last_joint_time > max_age:
            return "stale_joint_states"
        sensor_topic = self.get_parameter("sensor_topic").value
        if sensor_topic and bool(self.get_parameter("require_sensor").value):
            if not self.cloud_seen:
                return "no_sensor"
            max_sensor_age = float(self.get_parameter("max_sensor_age").value)
            if max_sensor_age > 0.0 \
                    and now - self.last_cloud_time > max_sensor_age:
                return "stale_sensor"
        if bool(self.get_parameter("require_octomap").value) \
                and not self.octomap_seen:
            return "no_octomap"
        return ""

    # ── 输入 ──
    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current[name] = position
        self.last_joint_time = self._now()

    def _on_cloud(self, _msg):
        self.cloud_seen = True
        self.last_cloud_time = self._now()

    def _on_scene(self, msg):
        octomap = msg.world.octomap.octomap
        if len(octomap.data) > 0:
            self.octomap_seen = True
            self.scene_updates += 1
            if self.busy and self.state in ("executing", "waiting_controller"):
                self.scene_dirty = True

        # 碰撞对象（scene_builder / 用户手动添加）也纳入变化检测。
        signature = []
        for obj in msg.world.collision_objects:
            pose = obj.primitive_poses[0] if obj.primitive_poses else None
            shape = obj.primitives[0].type if obj.primitives else -1
            dims = tuple(obj.primitives[0].dimensions) if obj.primitives else ()
            signature.append((
                obj.id, obj.header.frame_id, shape, dims,
                None if pose is None else (
                    round(pose.position.x, 4), round(pose.position.y, 4),
                    round(pose.position.z, 4)),
            ))
        signature = tuple(signature)
        if signature != self.last_collision_sig:
            self.last_collision_sig = signature
            self.scene_updates += 1
            if self.busy and self.state in ("executing", "waiting_controller"):
                self.scene_dirty = True

    def _on_pose_goal(self, msg):
        if self.busy:
            self.get_logger().warning("收到位姿目标，但当前仍在执行；忽略")
            return
        error = self._freshness_error()
        if error:
            self.get_logger().error(f"拒绝运动：{error}")
            self._status("failed", error)
            return
        if not self.ik_client.service_is_ready():
            self._status("failed", "ik_service_unavailable")
            return
        self.goal_source = "pose"
        self.busy = True
        self.plan_kind = "ik"
        self._status("ik", "")
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.get_parameter("group_name").value
        request.ik_request.ik_link_name = self.get_parameter("ee_link").value
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = "base_link"
        pose.pose = msg.pose
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(JOINTS)
        state.joint_state.position = [
            float(self.current.get(name, 0.0)) for name in JOINTS]
        request.ik_request.robot_state = state
        timeout = float(self.get_parameter("ik_timeout").value)
        request.ik_request.timeout = Duration(
            sec=int(timeout), nanosec=int((timeout % 1.0) * 1e9))
        future = self.ik_client.call_async(request)
        future.add_done_callback(self._ik_done)

    def _ik_done(self, future):
        if self.state != "ik":
            return
        response = future.result()
        if response is None or response.error_code.val != 1:
            code = None if response is None else response.error_code.val
            self._status("failed", f"ik_failed:{code}")
            return
        positions = dict(zip(response.solution.joint_state.name,
                             response.solution.joint_state.position))
        missing = [name for name in JOINTS if name not in positions]
        if missing:
            self._status("failed", f"ik_missing_joints:{missing}")
            return
        self.goal = [float(positions[name]) for name in JOINTS]
        self.replans = 0
        self.scene_dirty = False
        self.cancel_requested = False
        self._start_plan("initial")

    def _on_joint_goal(self, msg):
        if self.busy:
            self.get_logger().warning("收到关节目标，但当前仍在执行；忽略")
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
        if not self.plan_client.service_is_ready():
            self._status("failed", "plan_service_unavailable")
            return
        self.goal_source = "joints"
        self.goal = [float(positions[name]) for name in JOINTS]
        self.replans = 0
        self.scene_dirty = False
        self.cancel_requested = False
        self.busy = True
        self._start_plan("initial")

    # ── 规划 ──
    def _start_plan(self, kind):
        error = self._freshness_error()
        if error:
            self._safe_stop(error)
            return
        if not self.plan_client.service_is_ready():
            self._safe_stop("plan_service_unavailable")
            return
        if kind == "replan":
            max_replans = int(self.get_parameter("max_replans").value)
            if self.replans >= max_replans:
                self._fail("max_replans")
                return
            self.replans += 1
        self.plan_kind = kind
        self.scene_dirty = False
        self.cancel_requested = False
        self.path_invalid = False
        self.check_queue = []
        self.check_pending = False
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
        request.pipeline_id = self.get_parameter("pipeline_id").value
        request.planner_id = self.get_parameter("planner_id").value
        request.num_planning_attempts = int(
            self.get_parameter("plan_attempts").value)
        request.allowed_planning_time = float(
            self.get_parameter("planning_time").value)
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
        if response is not None \
                and response.motion_plan_response.error_code.val == 1:
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
        if trajectory is None or not trajectory.points:
            if kind == "retreat":
                self._safe_stop("retreat_failed")
            elif kind == "replan":
                # Octomap 在规划期间还在更新；OMPL 返回的路径可能被规划后
                # 校验判为无效。只要重规划预算还没用完，就再试一次，
                # 而不是立刻撤退。
                max_replans = int(self.get_parameter("max_replans").value)
                if self.replans < max_replans:
                    self.get_logger().warning(
                        f"replan 失败，重试 ({self.replans}/{max_replans})")
                    self._start_plan("replan")
                else:
                    self._fail("replan_failed")
            else:
                self._safe_stop("plan_failed")
            return

        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "base_link"
        self.trajectory_pub.publish(trajectory)
        self._start_execution(
            trajectory, "retreating" if kind == "retreat" else "executing")

    # ── 执行 ──
    def _start_execution(self, trajectory, state_name):
        self.pending_trajectory = trajectory
        self.pending_state = state_name
        self.trajectory = trajectory
        self.exec_start_time = self._now()
        self.scene_dirty = False
        self.path_invalid = False
        self.check_queue = []
        self.check_pending = False
        self._status(state_name, self.retreat_reason if state_name == "retreating" else "")
        if not self.action_client.server_is_ready():
            self._status("waiting_controller",
                         self.retreat_reason if state_name == "retreating" else "")
            return
        self._send_trajectory()

    def _send_trajectory(self):
        self.exec_seq += 1
        seq = self.exec_seq
        self.pending_trajectory.header.stamp = self.get_clock().now().to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.pending_trajectory
        self.result_received = False
        self.cancel_requested = False
        self.exec_start_time = self._now()
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._goal_response(done, seq))

    def _goal_response(self, future, seq):
        if seq != self.exec_seq:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._fail("controller_rejected_goal")
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

    # ── 剩余轨迹复检 ──
    def _closest_waypoint_index(self):
        if self.trajectory is None or not self.current:
            return 0
        best, best_error = 0, float("inf")
        for index, point in enumerate(self.trajectory.points):
            error = 0.0
            for name, position in zip(self.trajectory.joint_names,
                                      point.positions):
                error += abs(self.current.get(name, position) - position)
            if error < best_error:
                best, best_error = index, error
        return best

    def _maybe_check_path(self, now):
        if self.state != "executing" or not self.busy:
            return
        if not self.scene_dirty or self.check_pending:
            return
        if self.trajectory is None or not self.trajectory.points:
            return
        period = float(self.get_parameter("validity_check_period").value)
        if now - self.last_check_time < period:
            return
        if not self.validity_client.service_is_ready():
            self.scene_dirty = False
            return

        elapsed = 0.0 if self.exec_start_time is None \
            else max(0.0, now - self.exec_start_time)
        lookahead = float(self.get_parameter("validity_lookahead").value)
        start_index = 0
        for index, point in enumerate(self.trajectory.points):
            stamp = point.time_from_start.sec + \
                point.time_from_start.nanosec * 1e-9
            if stamp >= elapsed + lookahead:
                start_index = index
                break
        start_index = max(start_index, self._closest_waypoint_index() + 1)
        if start_index >= len(self.trajectory.points):
            self.scene_dirty = False
            return

        stride = max(1, int(self.get_parameter("validity_stride").value))
        indices = list(range(start_index, len(self.trajectory.points), stride))
        if indices[-1] != len(self.trajectory.points) - 1:
            indices.append(len(self.trajectory.points) - 1)
        names = list(self.trajectory.joint_names)
        self.check_queue = []
        for index in indices:
            point = self.trajectory.points[index]
            by_name = dict(zip(names, point.positions))
            self.check_queue.append((
                index,
                [float(by_name.get(name, self.current.get(name, 0.0)))
                 for name in JOINTS]))
        self.check_pending = True
        self.path_invalid = False
        self.scene_dirty = False
        self.last_check_time = now
        self._issue_next_validity()
        self.get_logger().info(
            f"场景更新，复检剩余 {len(self.check_queue)} 个轨迹点")

    def _issue_next_validity(self):
        if not self.check_queue:
            self.check_pending = False
            if self.path_invalid:
                if self.replans >= int(self.get_parameter("max_replans").value):
                    self._fail("max_replans")
                else:
                    self._begin_cancel_and_replan("path_blocked")
            return
        index, positions = self.check_queue.pop(0)
        self.current_check_index = index
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = positions[:len(JOINTS)]
        # 和规划器保持一致：只查规划组，而不是=""（全部 link，包含固定
        # 的 base_link）。后者会把基座与台面的安装接触误判为路径被挡。
        request.group_name = str(self.get_parameter("group_name").value)
        future = self.validity_client.call_async(request)
        future.add_done_callback(self._validity_done)

    def _validity_done(self, future):
        self.validity_checks += 1
        response = future.result()
        if response is None:
            self.path_invalid = True
            self.check_queue = []
            self.get_logger().warning(
                f"复检服务无响应: 轨迹点 #{self.current_check_index}")
            self._issue_next_validity()
            return
        if response.valid:
            self._issue_next_validity()
            return

        contacts = list(response.contacts)
        ignore_base = bool(
            self.get_parameter("ignore_base_link_contacts").value)
        relevant = [
            c for c in contacts
            if not (ignore_base and "base_link" in
                    (c.contact_body_1, c.contact_body_2))]
        if ignore_base and contacts and not relevant:
            # 只涉及固定 base_link 的接触：规划器按组检查时不算碰撞，复检
            # 也不能算，否则基座/台面的安装接触会让每个点都“被挡”，造成
            # 反复取消—重规划。继续检查剩余轨迹点。
            now = self._now()
            if now - self.last_base_ignore_log > 10.0:
                self.last_base_ignore_log = now
                self.get_logger().info(
                    "复检忽略 base_link 与场景的固定接触（基座/台面安装面）")
            self._issue_next_validity()
            return

        self.path_invalid = True
        # 已经确定被挡，不用再查剩余点；尽快取消/重规划。
        self.check_queue = []
        pairs = [(c.contact_body_1, c.contact_body_2)
                 for c in contacts[:3]]
        self.get_logger().warning(
            f"复检无效: 轨迹点 #{self.current_check_index} "
            f"contacts={pairs}")
        self._issue_next_validity()

    # ── 取消 / 重规划 / 失败 ──
    def _begin_cancel_and_replan(self, reason):
        self._status("canceling", reason)
        self.cancel_requested = True
        self.result_received = False
        self.replan_due = self._now() + float(
            self.get_parameter("cancel_settle").value)
        if self.active_handle is not None:
            self.active_handle.cancel_goal_async()
        else:
            self._start_plan("replan")

    def _fail(self, reason):
        if (bool(self.get_parameter("retreat_enabled").value)
                and self.active_handle is None
                and not self._freshness_error()):
            self.retreat_reason = reason
            self.goal = [float(v) for v in
                         self.get_parameter("retreat_positions").value]
            self.busy = True
            self._start_plan("retreat")
        else:
            self._safe_stop(reason)

    def _clear_octomap_if_stale(self, reason):
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

    def _cancel_cb(self, _request, response):
        self._safe_stop("user_cancel")
        response.success = True
        response.message = "cancelled"
        return response

    # ── 主循环 ──
    def _tick(self):
        now = self._now()

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
            if self.action_client.server_is_ready():
                self._status(self.pending_state,
                             self.retreat_reason
                             if self.pending_state == "retreating" else "")
                self._send_trajectory()
            return

        if self.state == "executing":
            self._maybe_check_path(now)
            # 复检队列没跑完时不要提前取消：_issue_next_validity() 在队列
            # 清空后会自己触发取消/重规划。
            if self.path_invalid and not self.check_pending:
                if self.replans >= int(self.get_parameter("max_replans").value):
                    self._fail("max_replans")
                else:
                    self._begin_cancel_and_replan("path_blocked")
                return

        if self.state == "canceling":
            if self.result_received or now >= self.replan_due:
                self.active_handle = None
                self.cancel_requested = False
                self.result_received = False
                self._start_plan("replan")

        self._publish_status()


def main():
    rclpy.init()
    node = D435iSupervisor()
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
