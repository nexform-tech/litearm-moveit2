#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_arm_probe.py — 双臂模型/规划组/互碰检查/执行验收探针。

默认只规划与复检；`--execute` 把轨迹直接发给 bridge，`--via-move-action`
走 MoveIt 的 /move_action。规划流程：

    读取 dual_goals.yaml 的目标
      → 等 /joint_states、/plan_kinematic_path、/check_state_validity
      → 对指定组规划（RRTConnect → RRTstar → PRM 回退）
      → 对轨迹均匀采样，逐点调用 /check_state_validity
      → 用 /compute_fk 记录左右 ee_link 的位姿
      → 可选：执行并记录同步/最终误差
      → 输出 JSON

    ros2 run litearm_dual_arm dual_arm_probe.py --target both_safe
    ros2 run litearm_dual_arm dual_arm_probe.py --target left_safe
    ros2 run litearm_dual_arm dual_arm_probe.py --target both_ready --samples 31
    ros2 run litearm_dual_arm dual_arm_probe.py --target both_safe --execute
    ros2 run litearm_dual_arm dual_arm_probe.py --target both_ready --via-move-action

    # 负向用例：预期目标位形被判为无效，且存在左右臂接触
    ros2 run litearm_dual_arm dual_arm_probe.py \
      --target mutual_collision_case --check-state

判据：规划模式下 `success=true` 表示规划成功且全部采样点有效；`contacts`
非空说明有碰撞。`--execute` / `--via-move-action` 还要求 `execution.success`
为 true。`--check-state` 模式下 `success=true` 表示目标位形**无效**且
`cross_arm_contacts` 非空——这是互碰约束生效的负向证明。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Mapping, Sequence

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
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


def load_targets(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    targets = config.get("targets")
    if not isinstance(targets, dict) or not targets:
        raise ValueError(f"goals file has no targets mapping: {path}")
    return targets


def load_target(targets: Mapping, name: str) -> tuple[str, dict]:
    if name not in targets:
        raise ValueError(f"unknown target {name!r}; available: {sorted(targets)}")
    entry = targets[name]
    if not isinstance(entry, dict):
        raise ValueError(f"target {name!r} must be a mapping")
    group = str(entry.get("group", "")).strip()
    if group not in GROUP_JOINTS:
        raise ValueError(f"target {name!r} has unknown group {group!r}")
    positions = entry.get("positions")
    if not isinstance(positions, dict):
        raise ValueError(f"target {name!r} has no positions mapping")
    parsed = {}
    for joint, value in positions.items():
        if joint not in GROUP_JOINTS[group]:
            raise ValueError(f"target {name!r} joint {joint!r} is not in group {group!r}")
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"target {name!r} joint {joint!r} is not finite")
        parsed[joint] = number
    if set(parsed) != set(GROUP_JOINTS[group]):
        missing = sorted(set(GROUP_JOINTS[group]) - set(parsed))
        raise ValueError(f"target {name!r} misses joints: {missing}")
    return group, parsed


def _wait_future(node: Node, future, timeout: float, label: str):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    if not future.done():
        raise TimeoutError(f"timeout waiting for {label}")
    return future.result()


class DualArmProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("litearm_dual_arm_probe")
        self.args = args
        self._positions: dict[str, float] = {}
        self._state_stamp = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
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
        self.stop_client = self.create_client(Trigger, "/litearm_dual_arm/stop")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")

    def _on_state(self, message: JointState) -> None:
        values = {}
        for name, position in zip(message.name, message.position):
            if name in ALL_JOINTS:
                values[name] = float(position)
        if set(values) == set(ALL_JOINTS) and all(
            math.isfinite(value) for value in values.values()
        ):
            self._positions = values
            self._state_stamp = time.monotonic()

    def wait_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            services = (self.plan_client, self.validity_client, self.fk_client)
            if self.args.via_move_action:
                actions_ready = self.move_group_client.server_is_ready()
            elif self.args.execute:
                actions_ready = (
                    self.left_client.server_is_ready()
                    and self.right_client.server_is_ready()
                )
            else:
                actions_ready = True
            if (
                all(client.service_is_ready() for client in services)
                and actions_ready
                and set(self._positions) == set(ALL_JOINTS)
            ):
                return
        raise TimeoutError(
            "MoveIt probe stack not ready: "
            f"services={[client.service_is_ready() for client in (self.plan_client, self.validity_client, self.fk_client)]}, "
            f"bridge_actions={self.left_client.server_is_ready()}/{self.right_client.server_is_ready()}, "
            f"move_action={self.move_group_client.server_is_ready()}, "
            f"execute={self.args.execute}, via_move_action={self.args.via_move_action}, "
            f"joints={len(self._positions)}"
        )

    def _state(self) -> dict[str, float]:
        if set(self._positions) != set(ALL_JOINTS):
            raise RuntimeError("joint state is incomplete")
        return dict(self._positions)

    def _request(
        self,
        group: str,
        target: Mapping[str, float],
        planner_id: str,
    ) -> MotionPlanRequest:
        current = self._state()
        request = MotionPlanRequest()
        request.group_name = group
        request.planner_id = planner_id
        request.num_planning_attempts = 5
        request.allowed_planning_time = float(self.args.planning_time)
        request.max_velocity_scaling_factor = float(self.args.velocity_scaling)
        request.max_acceleration_scaling_factor = float(self.args.acceleration_scaling)

        start = RobotState()
        start.is_diff = False
        start.joint_state.name = list(ALL_JOINTS)
        start.joint_state.position = [current[joint] for joint in ALL_JOINTS]
        request.start_state = start

        constraints = Constraints()
        for joint in GROUP_JOINTS[group]:
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=joint,
                    position=float(target[joint]),
                    tolerance_above=float(self.args.goal_tolerance),
                    tolerance_below=float(self.args.goal_tolerance),
                    weight=1.0,
                )
            )
        request.goal_constraints = [constraints]
        return request

    def plan(self, group: str, target: Mapping[str, float]) -> tuple[object, dict]:
        errors = []
        for planner_id in PLANNERS:
            started = time.monotonic()
            response = _wait_future(
                self,
                self.plan_client.call_async(
                    GetMotionPlan.Request(
                        motion_plan_request=self._request(group, target, planner_id)
                    )
                ),
                float(self.args.planning_time) + 20.0,
                f"plan {group} with {planner_id}",
            )
            wall = time.monotonic() - started
            result = response.motion_plan_response
            trajectory = result.trajectory.joint_trajectory
            if result.error_code.val == 1 and trajectory.points:
                return result.trajectory, {
                    "success": True,
                    "planner_id": planner_id,
                    "planning_time_s": round(float(result.planning_time), 6),
                    "planning_wall_s": round(wall, 6),
                    "trajectory_points": len(trajectory.points),
                    "trajectory_duration_s": round(
                        float(trajectory.points[-1].time_from_start.sec)
                        + float(trajectory.points[-1].time_from_start.nanosec) * 1e-9,
                        6,
                    ),
                    "errors": errors,
                }
            errors.append(
                {
                    "planner_id": planner_id,
                    "error_code": int(result.error_code.val),
                    "planning_time_s": round(float(result.planning_time), 6),
                }
            )
        return None, {"success": False, "errors": errors}

    def _full_state(self, overrides: Mapping[str, float]) -> RobotState:
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = list(ALL_JOINTS)
        state.joint_state.position = [
            float(overrides.get(joint, self._positions[joint])) for joint in ALL_JOINTS
        ]
        return state

    def _fk(self, state: RobotState) -> dict:
        request = GetPositionFK.Request()
        request.header = Header(frame_id="world")
        request.fk_link_names = ["left_ee_link", "right_ee_link"]
        request.robot_state = state
        response = _wait_future(
            self, self.fk_client.call_async(request), 5.0, "compute_fk"
        )
        if response.error_code.val != 1 or len(response.pose_stamped) != 2:
            return {}
        return {
            name: {
                "position": [
                    round(pose.pose.position.x, 6),
                    round(pose.pose.position.y, 6),
                    round(pose.pose.position.z, 6),
                ],
                "orientation_xyzw": [
                    round(pose.pose.orientation.x, 6),
                    round(pose.pose.orientation.y, 6),
                    round(pose.pose.orientation.z, 6),
                    round(pose.pose.orientation.w, 6),
                ],
            }
            for name, pose in zip(response.fk_link_names, response.pose_stamped)
        }

    def validate(self, group: str, robot_trajectory) -> dict:
        trajectory = robot_trajectory.joint_trajectory
        points = trajectory.points
        names = list(trajectory.joint_names)
        if not points:
            return {"checked": 0, "valid": 0, "invalid_indices": [], "contacts": []}

        count = max(2, int(self.args.samples))
        indices = sorted(
            set(
                int(round(index * (len(points) - 1) / (count - 1)))
                for index in range(count)
            )
        )
        invalid = []
        contacts = set()
        fk_samples = []
        for index in indices:
            overrides = dict(zip(names, points[index].positions))
            state = self._full_state(overrides)
            request = GetStateValidity.Request()
            # 无论规划的是单臂组还是 both_arms，复检都覆盖两条臂：单臂执行时
            # 空闲臂仍然在场景里，必须确认它在整段轨迹中都没有被碰到。
            request.group_name = "both_arms"
            request.robot_state = state
            response = _wait_future(
                self,
                self.validity_client.call_async(request),
                5.0,
                f"validate sample {index}",
            )
            if not response.valid:
                invalid.append(index)
            for contact in response.contacts:
                contacts.add(f"{contact.contact_body_1}<>{contact.contact_body_2}")
            fk = self._fk(state)
            if fk:
                fk_samples.append({"index": index, **fk})

        return {
            "checked": len(indices),
            "valid": len(indices) - len(invalid),
            "invalid_indices": invalid,
            "contacts": sorted(contacts),
            "fk_samples": fk_samples,
        }

    def check_state(self, group: str, target: Mapping[str, float]) -> dict:
        """只检查目标位形本身；用于预期碰撞的负向用例。"""
        overrides = dict(self._positions)
        overrides.update(target)
        request = GetStateValidity.Request()
        request.group_name = "both_arms"
        request.robot_state = self._full_state(overrides)
        response = _wait_future(
            self, self.validity_client.call_async(request), 10.0, "check state validity"
        )
        contacts = sorted(
            {f"{contact.contact_body_1}<>{contact.contact_body_2}" for contact in response.contacts}
        )
        cross_arm = sorted(
            {
                f"{contact.contact_body_1}<>{contact.contact_body_2}"
                for contact in response.contacts
                if (
                    contact.contact_body_1.startswith("left_")
                    and contact.contact_body_2.startswith("right_")
                )
                or (
                    contact.contact_body_1.startswith("right_")
                    and contact.contact_body_2.startswith("left_")
                )
            }
        )
        return {
            "mode": "check-state",
            "checked_group": group,
            "state_valid": bool(response.valid),
            "contacts": contacts,
            "cross_arm_contacts": cross_arm,
            # 负向用例：目标位形必须被判为无效，且至少有一对左右臂接触。
            "success": (not response.valid) and bool(cross_arm),
        }

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
        if left_finish is None and left_future.done():
            left_finish = time.monotonic()
        if right_finish is None and right_future.done():
            right_finish = time.monotonic()
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

    @staticmethod
    def _final_error(trajectory, final: Mapping[str, float], joints: Sequence[str]) -> float:
        names = list(trajectory.joint_names)
        last = trajectory.points[-1]
        return max(
            abs(final[joint] - float(last.positions[names.index(joint)]))
            for joint in joints
        )

    @staticmethod
    def _duration(trajectory) -> float:
        last = trajectory.points[-1].time_from_start
        return float(last.sec) + float(last.nanosec) * 1e-9

    def execute(self, group: str, robot_trajectory) -> dict:
        trajectory = robot_trajectory.joint_trajectory
        tolerance = max(0.02, float(self.args.goal_tolerance) * 2.0)
        idle_tolerance = max(0.01, float(self.args.goal_tolerance))
        current = dict(self._positions)

        if group != "both_arms":
            side = "left" if group == "left_arm" else "right"
            active_joints = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
            idle_joints = RIGHT_JOINTS if side == "left" else LEFT_JOINTS
            client = self.left_client if side == "left" else self.right_client
            goal = self._split_trajectory(trajectory, active_joints)
            try:
                handle, accepted_at = self._send_and_accept(client, goal, side)
            except Exception as error:
                self._stop_bridge()
                return {"mode": "execute", "success": False, "error": str(error)}
            if not handle.accepted:
                self._stop_bridge()
                return {
                    "mode": "execute",
                    "success": False,
                    "error": f"{side} goal rejected",
                }
            result_future = handle.get_result_async()
            finish = None
            deadline = time.monotonic() + self._duration(goal.trajectory) + 30.0
            while time.monotonic() < deadline and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.01)
            if result_future.done():
                finish = time.monotonic()
            wrapped = result_future.result() if result_future.done() else None
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.01)
            final = dict(self._positions)
            final_error = self._final_error(trajectory, final, active_joints)
            idle_deviation = max(
                abs(final[joint] - current[joint]) for joint in idle_joints
            )
            error_code = (
                int(wrapped.result.error_code) if wrapped is not None else None
            )
            success = (
                error_code == FollowJointTrajectory.Result.SUCCESSFUL
                and final_error <= tolerance
                and idle_deviation <= idle_tolerance
            )
            if not success:
                self._stop_bridge()
            return {
                "mode": "execute",
                "group": group,
                side: {
                    "accepted": bool(handle.accepted),
                    "finished_at_s": (
                        round(finish - accepted_at, 6) if finish is not None else None
                    ),
                    "error_code": error_code,
                    "final_error_rad": round(final_error, 6),
                },
                "idle_arm_max_deviation_rad": round(idle_deviation, 6),
                "success": success,
            }

        left_goal = self._split_trajectory(trajectory, LEFT_JOINTS)
        right_goal = self._split_trajectory(trajectory, RIGHT_JOINTS)
        try:
            left_handle, left_accepted_at = self._send_and_accept(
                self.left_client, left_goal, "left"
            )
            right_handle, right_accepted_at = self._send_and_accept(
                self.right_client, right_goal, "right"
            )
        except Exception as error:
            self._stop_bridge()
            return {"mode": "execute", "success": False, "error": str(error)}

        if not left_handle.accepted or not right_handle.accepted:
            if left_handle.accepted:
                left_handle.cancel_goal_async()
            if right_handle.accepted:
                right_handle.cancel_goal_async()
            self._stop_bridge()
            return {
                "mode": "execute",
                "success": False,
                "error": (
                    f"dual goals rejected: left={left_handle.accepted}, "
                    f"right={right_handle.accepted}"
                ),
            }

        left_future = left_handle.get_result_async()
        right_future = right_handle.get_result_async()
        duration = max(
            self._duration(left_goal.trajectory),
            self._duration(right_goal.trajectory),
        )
        left_finish, right_finish = self._wait_two_results(
            left_future, right_future, duration + 30.0
        )
        wrapped_left = left_future.result() if left_future.done() else None
        wrapped_right = right_future.result() if right_future.done() else None
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.01)
        final = dict(self._positions)
        left_error = self._final_error(trajectory, final, LEFT_JOINTS)
        right_error = self._final_error(trajectory, final, RIGHT_JOINTS)
        left_code = (
            int(wrapped_left.result.error_code) if wrapped_left is not None else None
        )
        right_code = (
            int(wrapped_right.result.error_code) if wrapped_right is not None else None
        )
        success = (
            left_code == FollowJointTrajectory.Result.SUCCESSFUL
            and right_code == FollowJointTrajectory.Result.SUCCESSFUL
            and left_error <= tolerance
            and right_error <= tolerance
        )
        if left_finish is not None and right_finish is not None:
            sync_error = max(
                abs(left_accepted_at - right_accepted_at),
                abs(left_finish - right_finish),
            )
        else:
            sync_error = None
        if not success:
            self._stop_bridge()
        return {
            "mode": "execute",
            "group": group,
            "left": {
                "accepted": bool(left_handle.accepted),
                "accepted_at_s": 0.0,
                "finished_at_s": (
                    round(left_finish - left_accepted_at, 6)
                    if left_finish is not None
                    else None
                ),
                "error_code": left_code,
                "final_error_rad": round(left_error, 6),
            },
            "right": {
                "accepted": bool(right_handle.accepted),
                "accepted_at_s": round(
                    right_accepted_at - left_accepted_at, 6
                ),
                "finished_at_s": (
                    round(right_finish - left_accepted_at, 6)
                    if right_finish is not None
                    else None
                ),
                "error_code": right_code,
                "final_error_rad": round(right_error, 6),
            },
            "sync_error_s": round(sync_error, 6) if sync_error is not None else None,
            "success": success,
        }

    def run_move_action(self, group: str, target: Mapping[str, float]) -> dict:
        """走 MoveIt 的 /move_action，验证 simple controller manager → bridge 链路。"""
        goal = MoveGroup.Goal()
        goal.request = self._request(group, target, "RRTConnectkConfigDefault")
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        base = {"target": self.args.target, "group": group, "mode": "move-action"}
        initial = dict(self._positions)
        try:
            handle = _wait_future(
                self,
                self.move_group_client.send_goal_async(goal),
                10.0,
                "move action acceptance",
            )
            if handle is None or not handle.accepted:
                base.update(
                    {
                        "execution": {
                            "mode": "move-action",
                            "success": False,
                            "error": "MoveGroup goal rejected",
                        },
                        "success": False,
                    }
                )
                return base
            wrapped = _wait_future(
                self,
                handle.get_result_async(),
                max(60.0, float(self.args.planning_time) + 60.0),
                "move action result",
            )
            result = wrapped.result
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.01)
            final = dict(self._positions)
            group_joints = GROUP_JOINTS[group]
            final_error = max(
                abs(final[joint] - float(target[joint])) for joint in group_joints
            )
            idle_joints = set(ALL_JOINTS) - set(group_joints)
            idle_deviation = (
                max(abs(final[joint] - initial[joint]) for joint in idle_joints)
                if idle_joints
                else 0.0
            )
            tolerance = max(0.04, float(self.args.goal_tolerance) * 2.0)
            idle_tolerance = max(0.02, float(self.args.goal_tolerance))
            trajectory = result.planned_trajectory.joint_trajectory
            success = (
                int(result.error_code.val) == 1
                and final_error <= tolerance
                and idle_deviation <= idle_tolerance
            )
            base.update(
                {
                    "execution": {
                        "mode": "move-action",
                        "error_code": int(result.error_code.val),
                        "planning_time_s": round(float(result.planning_time), 6),
                        "trajectory_points": len(trajectory.points),
                        "final_error_rad": round(final_error, 6),
                        "idle_arm_max_deviation_rad": round(idle_deviation, 6),
                        "success": success,
                    },
                    "success": success,
                }
            )
            return base
        except Exception as error:
            base.update(
                {
                    "execution": {
                        "mode": "move-action",
                        "success": False,
                        "error": str(error),
                    },
                    "success": False,
                }
            )
            return base

    def run(self) -> dict:
        targets = load_targets(self.args.goals)
        target_name = self.args.target
        group, target = load_target(targets, target_name)
        if self.args.group and self.args.group != group:
            raise ValueError(
                f"target {target_name!r} belongs to group {group!r}, "
                f"but --group {self.args.group!r} was requested"
            )
        self.wait_until_ready(float(self.args.timeout))
        if self.args.check_state:
            result = {
                "target": target_name,
                "group": group,
                "start_state": {
                    joint: round(self._positions[joint], 6) for joint in ALL_JOINTS
                },
            }
            result.update(self.check_state(group, target))
            return result
        if self.args.via_move_action:
            return self.run_move_action(group, target)

        trajectory, planning = self.plan(group, target)
        result = {
            "target": target_name,
            "group": group,
            "mode": "plan",
            "start_state": {joint: round(self._positions[joint], 6) for joint in ALL_JOINTS},
            "planning": planning,
            "validation": None,
            "execution": None,
            "success": False,
        }
        if trajectory is None:
            return result
        validation = self.validate(group, trajectory)
        result["validation"] = validation
        result["success"] = (
            planning.get("success", False)
            and validation["checked"] > 0
            and not validation["invalid_indices"]
            and not validation["contacts"]
        )
        if self.args.execute:
            if result["success"]:
                execution = self.execute(group, trajectory)
                result["execution"] = execution
                result["success"] = bool(execution.get("success"))
            else:
                result["execution"] = {
                    "mode": "execute",
                    "success": False,
                    "error": "planning or validation failed; not executed",
                }
        return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="both_safe")
    parser.add_argument(
        "--check-state",
        action="store_true",
        help="只检查目标位形本身（用于预期碰撞的负向用例），不规划",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="规划并复检通过后，把轨迹发给 bridge 执行（确定性仿真）",
    )
    parser.add_argument(
        "--via-move-action",
        action="store_true",
        help="走 MoveIt 的 /move_action 执行，验证 simple controller manager → bridge",
    )
    parser.add_argument("--group", default="", help="optional group cross-check")
    parser.add_argument("--goals", default="", help="dual_goals.yaml path")
    parser.add_argument("--output", default="/tmp/litearm_dual_arm_probe.json")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.10)
    parser.add_argument("--acceleration-scaling", type=float, default=0.10)
    parser.add_argument("--goal-tolerance", type=float, default=0.02)
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args(argv)


def default_goals() -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("litearm_dual_arm"), "config", "dual_goals.yaml"
    )


def _write(path: str, payload: Mapping) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(remove_ros_args(argv or sys.argv)[1:])
    if args.samples < 2:
        print("dual_arm_probe: --samples must be at least 2", file=sys.stderr)
        return 2
    if not args.goals:
        args.goals = default_goals()

    rclpy.init()
    node = None
    try:
        node = DualArmProbe(args)
        payload = node.run()
    except Exception as error:
        payload = {
            "target": args.target,
            "success": False,
            "error": str(error),
        }
        print(f"dual_arm_probe: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    _write(args.output, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if payload.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
