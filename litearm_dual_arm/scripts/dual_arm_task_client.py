#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_arm_task_client.py — 双臂任务共用的 MoveIt / bridge 客户端。

同步搬运和交替作业都会重复这些动作，所以集中在这里：

    * 等待 MoveIt 服务、/joint_states、bridge action；
    * 应用静态场景；
    * IK、规划、轨迹采样复检、FK；
    * 通过 bridge 执行单臂/双臂轨迹；
    * 复位仿真、输出 JSON。

任务脚本只负责“任务怎么编排”，不重复这些细节。
"""

from __future__ import annotations

import json
import math
import time
from typing import Mapping, Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


LEFT_JOINTS = tuple(f"left_joint{index}" for index in range(1, 8))
RIGHT_JOINTS = tuple(f"right_joint{index}" for index in range(1, 8))
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS
GROUP_JOINTS = {
    "left_arm": LEFT_JOINTS,
    "right_arm": RIGHT_JOINTS,
    "both_arms": ALL_JOINTS,
}
PLANNERS = ("RRTConnectkConfigDefault", "RRTstar", "PRM")


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _wait_future(node: Node, future, timeout: float, label: str):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    if not future.done():
        raise TimeoutError(f"timeout waiting for {label}")
    return future.result()


class DualArmTaskClient(Node):
    def __init__(self, name: str = "litearm_dual_arm_task") -> None:
        super().__init__(name)
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.reset_client = self.create_client(Trigger, "/litearm_dual_sim/reset")
        self.stop_client = self.create_client(Trigger, "/litearm_dual_arm/stop")
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
        )
        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory",
        )

    # ── readiness / state ───────────────────────────────────────────────
    def _on_state(self, message: JointState) -> None:
        values = {
            name: float(position)
            for name, position in zip(message.name, message.position)
            if name in ALL_JOINTS
        }
        if set(values) == set(ALL_JOINTS) and all(
            math.isfinite(value) for value in values.values()
        ):
            self._positions = values

    def wait_until_ready(
        self,
        timeout: float = 60.0,
        *,
        require_actions: bool = False,
        require_ik: bool = False,
        require_cartesian: bool = False,
    ) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            services = [self.plan_client, self.validity_client]
            if require_ik:
                services.append(self.ik_client)
            if require_cartesian:
                services.append(self.cartesian_client)
            services_ready = all(client.service_is_ready() for client in services)
            actions_ready = (
                not require_actions
                or (
                    self.left_client.server_is_ready()
                    and self.right_client.server_is_ready()
                )
            )
            if (
                services_ready
                and actions_ready
                and set(self._positions) == set(ALL_JOINTS)
            ):
                return
        raise TimeoutError(
            "dual-arm task stack not ready: "
            f"services={[client.service_is_ready() for client in (self.plan_client, self.validity_client, self.ik_client, self.cartesian_client)]}, "
            f"actions={self.left_client.server_is_ready()}/{self.right_client.server_is_ready()}, "
            f"joints={len(self._positions)}"
        )

    def positions(self) -> dict[str, float]:
        if set(self._positions) != set(ALL_JOINTS):
            raise RuntimeError("joint state is incomplete")
        return dict(self._positions)

    def _full_state(self, overrides: Mapping[str, float]) -> RobotState:
        current = self.positions()
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = list(ALL_JOINTS)
        state.joint_state.position = [
            float(overrides.get(joint, current[joint])) for joint in ALL_JOINTS
        ]
        return state

    # ── scene ───────────────────────────────────────────────────────────
    def apply_scene(self, config_path: str, scene_name: str, frame: str) -> int:
        if not scene_name or scene_name.lower() in ("none", "off", "false"):
            return 0
        from dual_arm_scene import build_scene_message, load_scene

        objects, colors = load_scene(config_path, scene_name, frame)
        message = build_scene_message(objects, colors)
        if not self.scene_client.service_is_ready():
            if not self.scene_client.wait_for_service(timeout_sec=60.0):
                raise TimeoutError("/apply_planning_scene is not available")
        response = _wait_future(
            self,
            self.scene_client.call_async(ApplyPlanningScene.Request(scene=message)),
            10.0,
            "apply scene",
        )
        if not response.success:
            raise RuntimeError("MoveIt rejected the planning scene")
        self.get_logger().info(f"applied scene {scene_name!r}: {len(objects)} objects")
        return len(objects)

    # ── IK / FK / validity ──────────────────────────────────────────────
    def compute_ik(
        self,
        group: str,
        pose: Pose,
        seeds: Sequence[Mapping[str, float]],
    ) -> dict[str, float]:
        joints = GROUP_JOINTS[group]
        for index, seed in enumerate(seeds):
            request = GetPositionIK.Request()
            request.ik_request.group_name = group
            request.ik_request.robot_state = self._full_state(seed)
            request.ik_request.avoid_collisions = False
            request.ik_request.timeout.sec = 1
            target = PoseStamped()
            target.header.frame_id = "world"
            target.pose = pose
            request.ik_request.pose_stamped = target
            response = _wait_future(
                self,
                self.ik_client.call_async(request),
                5.0,
                f"{group} IK seed {index}",
            )
            if response.error_code.val != 1:
                continue
            values = dict(
                zip(
                    response.solution.joint_state.name,
                    response.solution.joint_state.position,
                )
            )
            if set(joints) <= set(values):
                return {joint: float(values[joint]) for joint in joints}
        raise RuntimeError(f"{group}: no IK solution for the requested pose")

    def compute_fk(self, positions: Mapping[str, float]) -> dict[str, Pose]:
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = ["left_ee_link", "right_ee_link"]
        request.robot_state = self._full_state(positions)
        response = _wait_future(
            self, self.fk_client.call_async(request), 5.0, "compute_fk"
        )
        if response.error_code.val != 1 or len(response.pose_stamped) != 2:
            raise RuntimeError("compute_fk failed")
        return {
            name: stamped.pose
            for name, stamped in zip(response.fk_link_names, response.pose_stamped)
        }

    def check_state(self, positions: Mapping[str, float]) -> tuple[bool, list[str]]:
        request = GetStateValidity.Request()
        request.group_name = "both_arms"
        request.robot_state = self._full_state(positions)
        response = _wait_future(
            self, self.validity_client.call_async(request), 5.0, "check state"
        )
        contacts = sorted(
            {
                f"{contact.contact_body_1}<>{contact.contact_body_2}"
                for contact in response.contacts
            }
        )
        return bool(response.valid), contacts

    # ── planning ────────────────────────────────────────────────────────
    def plan_joint(
        self,
        group: str,
        target: Mapping[str, float],
        *,
        planning_time: float = 10.0,
        velocity_scaling: float = 0.10,
        acceleration_scaling: float = 0.10,
        goal_tolerance: float = 0.02,
    ):
        errors = []
        for planner_id in PLANNERS:
            request = MotionPlanRequest()
            request.group_name = group
            request.planner_id = planner_id
            request.num_planning_attempts = 5
            request.allowed_planning_time = float(planning_time)
            request.max_velocity_scaling_factor = float(velocity_scaling)
            request.max_acceleration_scaling_factor = float(acceleration_scaling)
            request.start_state = self._full_state({})
            constraints = Constraints()
            for joint in GROUP_JOINTS[group]:
                constraints.joint_constraints.append(
                    JointConstraint(
                        joint_name=joint,
                        position=float(target[joint]),
                        tolerance_above=float(goal_tolerance),
                        tolerance_below=float(goal_tolerance),
                        weight=1.0,
                    )
                )
            request.goal_constraints = [constraints]
            response = _wait_future(
                self,
                self.plan_client.call_async(
                    GetMotionPlan.Request(motion_plan_request=request)
                ),
                float(planning_time) + 20.0,
                f"plan {group} with {planner_id}",
            )
            result = response.motion_plan_response
            trajectory = result.trajectory.joint_trajectory
            if result.error_code.val == 1 and trajectory.points:
                return result.trajectory, {
                    "success": True,
                    "planner_id": planner_id,
                    "planning_time_s": round(float(result.planning_time), 6),
                    "trajectory_points": len(trajectory.points),
                    "errors": errors,
                }
            errors.append(
                {
                    "planner_id": planner_id,
                    "error_code": int(result.error_code.val),
                }
            )
        return None, {"success": False, "errors": errors}

    def cartesian_path(
        self,
        group: str,
        link_name: str,
        start_positions: Mapping[str, float],
        waypoints: Sequence[Pose],
        *,
        max_step: float = 0.0025,
        jump_threshold: float = 5.0,
        revolute_jump_threshold: float = 0.50,
        velocity_scaling: float = 0.10,
        acceleration_scaling: float = 0.10,
    ):
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.start_state = self._full_state(start_positions)
        request.group_name = group
        request.link_name = link_name
        request.max_step = float(max_step)
        request.jump_threshold = float(jump_threshold)
        request.revolute_jump_threshold = float(revolute_jump_threshold)
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = float(velocity_scaling)
        request.max_acceleration_scaling_factor = float(acceleration_scaling)
        request.waypoints = list(waypoints)
        response = _wait_future(
            self,
            self.cartesian_client.call_async(request),
            30.0,
            f"{group} Cartesian path",
        )
        if response.error_code.val != 1 or response.fraction < 0.999:
            raise RuntimeError(
                f"{group} Cartesian path failed: code={response.error_code.val}, "
                f"fraction={response.fraction:.3f}"
            )
        return response.solution.joint_trajectory, {
            "fraction": round(float(response.fraction), 6),
            "points": len(response.solution.joint_trajectory.points),
        }

    def validate_trajectory(
        self,
        trajectory,
        samples: int = 20,
    ) -> dict:
        points = trajectory.points
        names = list(trajectory.joint_names)
        count = max(2, int(samples))
        indices = sorted(
            {
                int(round(index * (len(points) - 1) / (count - 1)))
                for index in range(count)
            }
        )
        invalid = []
        contacts = set()
        for index in indices:
            overrides = dict(zip(names, points[index].positions))
            valid, state_contacts = self.check_state(overrides)
            if not valid:
                invalid.append(index)
            contacts.update(state_contacts)
        return {
            "checked": len(indices),
            "valid": len(indices) - len(invalid),
            "invalid_indices": invalid,
            "contacts": sorted(contacts),
        }

    # ── execution ───────────────────────────────────────────────────────
    @staticmethod
    def _split_trajectory(source, joints: Sequence[str]):
        names = list(source.joint_names)
        indices = [names.index(joint) for joint in joints]
        goal = FollowJointTrajectory.Goal()
        target = goal.trajectory
        target.header = source.header
        target.joint_names = list(joints)
        for source_point in source.points:
            point = type(source_point)()
            point.time_from_start = source_point.time_from_start
            point.positions = [
                float(source_point.positions[index]) for index in indices
            ]
            if len(source_point.velocities) == len(names):
                point.velocities = [
                    float(source_point.velocities[index]) for index in indices
                ]
            if len(source_point.accelerations) == len(names):
                point.accelerations = [
                    float(source_point.accelerations[index]) for index in indices
                ]
            target.points.append(point)
        return goal

    def _send_and_accept(self, client, goal, label: str):
        handle = _wait_future(
            self,
            client.send_goal_async(goal),
            10.0,
            f"{label} goal acceptance",
        )
        return handle, time.monotonic()

    def _wait_two_results(self, left_future, right_future, timeout: float):
        deadline = time.monotonic() + timeout
        left_finish = None
        right_finish = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if left_finish is None and left_future.done():
                left_finish = time.monotonic()
            if right_finish is None and right_future.done():
                right_finish = time.monotonic()
            if left_finish is not None and right_finish is not None:
                break
        return left_finish, right_finish

    def _stop_bridge(self) -> None:
        if self.stop_client.service_is_ready():
            try:
                _wait_future(
                    self,
                    self.stop_client.call_async(Trigger.Request()),
                    2.0,
                    "bridge stop",
                )
            except Exception:
                pass

    def execute_dual(self, trajectory, goal_tolerance: float = 0.04) -> dict:
        if not (
            self.left_client.server_is_ready()
            and self.right_client.server_is_ready()
        ):
            raise RuntimeError("bridge action servers are not available")
        left_goal = self._split_trajectory(trajectory, LEFT_JOINTS)
        right_goal = self._split_trajectory(trajectory, RIGHT_JOINTS)
        try:
            left_handle, left_accepted_at = self._send_and_accept(
                self.left_client, left_goal, "left"
            )
            right_handle, right_accepted_at = self._send_and_accept(
                self.right_client, right_goal, "right"
            )
        except Exception:
            self._stop_bridge()
            raise
        if not left_handle.accepted or not right_handle.accepted:
            if left_handle.accepted:
                left_handle.cancel_goal_async()
            if right_handle.accepted:
                right_handle.cancel_goal_async()
            self._stop_bridge()
            raise RuntimeError(
                f"dual goals rejected: left={left_handle.accepted}, "
                f"right={right_handle.accepted}"
            )
        left_future = left_handle.get_result_async()
        right_future = right_handle.get_result_async()
        duration = max(
            _duration_seconds(left_goal.trajectory.points[-1].time_from_start),
            _duration_seconds(right_goal.trajectory.points[-1].time_from_start),
        )
        left_finish, right_finish = self._wait_two_results(
            left_future, right_future, duration + 30.0
        )
        wrapped_left = left_future.result() if left_future.done() else None
        wrapped_right = right_future.result() if right_future.done() else None
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.01)
        final = self.positions()
        names = list(trajectory.joint_names)
        last = trajectory.points[-1]
        left_error = max(
            abs(final[joint] - float(last.positions[names.index(joint)]))
            for joint in LEFT_JOINTS
        )
        right_error = max(
            abs(final[joint] - float(last.positions[names.index(joint)]))
            for joint in RIGHT_JOINTS
        )
        left_code = (
            int(wrapped_left.result.error_code) if wrapped_left is not None else None
        )
        right_code = (
            int(wrapped_right.result.error_code) if wrapped_right is not None else None
        )
        success = (
            left_code == FollowJointTrajectory.Result.SUCCESSFUL
            and right_code == FollowJointTrajectory.Result.SUCCESSFUL
            and left_error <= goal_tolerance
            and right_error <= goal_tolerance
        )
        if not success:
            self._stop_bridge()
        sync_error = None
        if left_finish is not None and right_finish is not None:
            sync_error = max(
                abs(left_accepted_at - right_accepted_at),
                abs(left_finish - right_finish),
            )
        return {
            "left": {
                "accepted": bool(left_handle.accepted),
                "error_code": left_code,
                "final_error_rad": round(left_error, 6),
                "finished_at_s": (
                    round(left_finish - left_accepted_at, 6)
                    if left_finish is not None
                    else None
                ),
            },
            "right": {
                "accepted": bool(right_handle.accepted),
                "error_code": right_code,
                "final_error_rad": round(right_error, 6),
                "finished_at_s": (
                    round(right_finish - left_accepted_at, 6)
                    if right_finish is not None
                    else None
                ),
            },
            "sync_error_s": round(sync_error, 6) if sync_error is not None else None,
            "success": success,
        }

    def execute_single(
        self,
        group: str,
        trajectory,
        *,
        goal_tolerance: float = 0.04,
        idle_tolerance: float = 0.02,
    ) -> dict:
        if group not in ("left_arm", "right_arm"):
            raise ValueError("execute_single requires left_arm or right_arm")
        side = "left" if group == "left_arm" else "right"
        client = self.left_client if side == "left" else self.right_client
        idle_joints = RIGHT_JOINTS if side == "left" else LEFT_JOINTS
        if not client.server_is_ready():
            raise RuntimeError(f"{side} bridge action server is not available")
        initial = self.positions()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        try:
            handle, accepted_at = self._send_and_accept(client, goal, side)
        except Exception:
            self._stop_bridge()
            raise
        if not handle.accepted:
            self._stop_bridge()
            raise RuntimeError(f"{side} goal rejected")
        result_future = handle.get_result_async()
        duration = _duration_seconds(trajectory.points[-1].time_from_start)
        deadline = time.monotonic() + duration + 30.0
        finish = None
        while time.monotonic() < deadline and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        if result_future.done():
            finish = time.monotonic()
        wrapped = result_future.result() if result_future.done() else None
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.01)
        final = self.positions()
        names = list(trajectory.joint_names)
        last = trajectory.points[-1]
        active_error = max(
            abs(final[joint] - float(last.positions[names.index(joint)]))
            for joint in (LEFT_JOINTS if side == "left" else RIGHT_JOINTS)
        )
        idle_deviation = max(
            abs(final[joint] - initial[joint]) for joint in idle_joints
        )
        error_code = int(wrapped.result.error_code) if wrapped is not None else None
        success = (
            error_code == FollowJointTrajectory.Result.SUCCESSFUL
            and active_error <= goal_tolerance
            and idle_deviation <= idle_tolerance
        )
        if not success:
            self._stop_bridge()
        return {
            "group": group,
            "accepted": bool(handle.accepted),
            "error_code": error_code,
            "active_final_error_rad": round(active_error, 6),
            "idle_arm_max_deviation_rad": round(idle_deviation, 6),
            "finished_at_s": (
                round(finish - accepted_at, 6) if finish is not None else None
            ),
            "success": success,
        }

    # ── sim / output ────────────────────────────────────────────────────
    def reset_sim(self) -> bool:
        if not self.reset_client.service_is_ready():
            return False
        try:
            response = _wait_future(
                self,
                self.reset_client.call_async(Trigger.Request()),
                5.0,
                "sim reset",
            )
            return bool(response.success)
        except Exception:
            return False


def write_json(path: str, payload: Mapping) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
