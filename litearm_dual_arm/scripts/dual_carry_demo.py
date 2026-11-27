#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_carry_demo.py — 双臂同步搬运（托盘）确定性仿真。

不需要夹爪。左右工具被视为托盘的两个虚拟夹持点：

    1. 从 dual_carry.yaml 读取托盘起止中心、间距、方向和工具姿态；
    2. 用 /compute_ik 求左右起始关节；
    3. both_arms 规划到起始位形，并逐点复检；
    4. 左右分别做笛卡尔直线；
    5. 按共同进度拼成 14 关节同步轨迹；
    6. 每个采样点检查互碰、间距、共面、姿态；
    7. 通过 bridge 同时执行（--plan-only 时只规划）。

    ros2 run litearm_dual_arm dual_carry_demo.py --plan-only
    ros2 run litearm_dual_arm dual_carry_demo.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Mapping, Sequence

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dual_arm_task_client import (
    ALL_JOINTS,
    GROUP_JOINTS,
    DualArmTaskClient,
    write_json,
)


def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _vector(value, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{field} must be a list of {length} numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} contains a non-finite number")
    return result


def _normalize(vector: Sequence[float], field: str) -> tuple[float, ...]:
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-9:
        raise ValueError(f"{field} must be non-zero")
    return tuple(value / length for value in vector)


def _normalize_quat(value, field: str) -> tuple[float, float, float, float]:
    quat = _vector(value, 4, field)
    length = math.sqrt(sum(item * item for item in quat))
    if length <= 1e-9:
        raise ValueError(f"{field} must be a non-zero quaternion")
    return tuple(item / length for item in quat)


def _quaternion_error_rad(
    first: Sequence[float], second: Sequence[float]
) -> float:
    dot = abs(sum(a * b for a, b in zip(first, second)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _pose(xyz: Sequence[float], quat: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [
        float(v) for v in quat
    ]
    return pose


def load_carry_config(path: str) -> dict:
    config = _load_yaml(path)
    if not isinstance(config, dict):
        raise ValueError("carry configuration must be a mapping")
    frame = str(config.get("frame", "world")).strip()
    if frame != "world":
        raise ValueError("carry configuration currently supports frame=world only")
    center = _vector(config.get("start_center"), 3, "start_center")
    direction = _normalize(
        _vector(config.get("direction"), 3, "direction"), "direction"
    )
    separation_axis = _normalize(
        _vector(config.get("separation_axis"), 3, "separation_axis"),
        "separation_axis",
    )
    distance = float(config.get("distance_m", 0.0))
    separation = float(config.get("hand_separation_m", 0.0))
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("distance_m must be positive")
    if not math.isfinite(separation) or separation <= 0.0:
        raise ValueError("hand_separation_m must be positive")
    left_quat = _normalize_quat(config.get("left_xyzw"), "left_xyzw")
    right_quat = _normalize_quat(config.get("right_xyzw"), "right_xyzw")
    start_left_rad = config.get("start_left_rad")
    start_right_rad = config.get("start_right_rad")
    if (start_left_rad is None) != (start_right_rad is None):
        raise ValueError("start_left_rad and start_right_rad must be provided together")
    if start_left_rad is not None:
        start_left_rad = _vector(start_left_rad, 7, "start_left_rad")
        start_right_rad = _vector(start_right_rad, 7, "start_right_rad")
    motion = config.get("motion") or {}
    validation = config.get("validation") or {}
    result = {
        "frame": frame,
        "scene": str(config.get("scene", "none")).strip() or "none",
        "start_center": center,
        "direction": direction,
        "distance_m": distance,
        "hand_separation_m": separation,
        "separation_axis": separation_axis,
        "left_xyzw": left_quat,
        "right_xyzw": right_quat,
        "start_left_rad": start_left_rad,
        "start_right_rad": start_right_rad,
        "motion": {
            "planning_time_s": float(motion.get("planning_time_s", 10.0)),
            "velocity_scaling": float(motion.get("velocity_scaling", 0.10)),
            "acceleration_scaling": float(motion.get("acceleration_scaling", 0.10)),
            "max_cartesian_step_m": float(motion.get("max_cartesian_step_m", 0.0025)),
            "cartesian_jump_threshold": float(
                motion.get("cartesian_jump_threshold", 5.0)
            ),
            "revolute_jump_threshold_rad": float(
                motion.get("revolute_jump_threshold_rad", 0.50)
            ),
            "path_duration_s": float(motion.get("path_duration_s", 8.0)),
            "samples": int(motion.get("samples", 41)),
            "goal_tolerance_rad": float(motion.get("goal_tolerance_rad", 0.02)),
            "execution_timeout_s": float(motion.get("execution_timeout_s", 30.0)),
        },
        "validation": {
            "position_tolerance_m": float(validation.get("position_tolerance_m", 0.002)),
            "orientation_tolerance_rad": float(
                validation.get("orientation_tolerance_rad", 0.01)
            ),
            "separation_tolerance_m": float(
                validation.get("separation_tolerance_m", 0.002)
            ),
            "plane_tolerance_m": float(validation.get("plane_tolerance_m", 0.002)),
        },
    }
    if result["motion"]["samples"] < 2:
        raise ValueError("motion.samples must be at least 2")
    if result["motion"]["path_duration_s"] <= 0.0:
        raise ValueError("motion.path_duration_s must be positive")
    return result


def _carry_poses(config: Mapping) -> dict:
    center = config["start_center"]
    direction = config["direction"]
    axis = config["separation_axis"]
    half = config["hand_separation_m"] / 2.0
    end_center = tuple(
        center[index] + direction[index] * config["distance_m"]
        for index in range(3)
    )
    left_start = tuple(center[index] + axis[index] * half for index in range(3))
    right_start = tuple(center[index] - axis[index] * half for index in range(3))
    left_end = tuple(end_center[index] + axis[index] * half for index in range(3))
    right_end = tuple(end_center[index] - axis[index] * half for index in range(3))
    return {
        "start_center": center,
        "end_center": end_center,
        "left_start": left_start,
        "right_start": right_start,
        "left_end": left_end,
        "right_end": right_end,
        "left_pose_start": _pose(left_start, config["left_xyzw"]),
        "right_pose_start": _pose(right_start, config["right_xyzw"]),
        "left_pose_end": _pose(left_end, config["left_xyzw"]),
        "right_pose_end": _pose(right_end, config["right_xyzw"]),
    }


def _interpolate_path(trajectory, fraction: float) -> list[float]:
    points = trajectory.points
    if len(points) == 1:
        return [float(value) for value in points[0].positions]
    position = max(0.0, min(1.0, fraction)) * (len(points) - 1)
    lower = int(math.floor(position))
    upper = min(lower + 1, len(points) - 1)
    blend = position - lower
    return [
        float(points[lower].positions[index]) * (1.0 - blend)
        + float(points[upper].positions[index]) * blend
        for index in range(len(points[lower].positions))
    ]


def _path_axis_samples(
    client,
    trajectory,
    start_positions: Mapping[str, float],
    link_name: str,
    axis_index: int,
) -> list[tuple[float, list[float]]]:
    """把一条关节轨迹转成 (工具轴向坐标, 关节角) 序列。

    左右 Cartesian 路径的点数可能不同，按点序号插值会让两侧进度错开；
    这里用 FK 得到的工具坐标做共同进度基准。
    """
    axis_name = ("x", "y", "z")[axis_index]
    samples = []
    for point in trajectory.points:
        combined = dict(start_positions)
        combined.update(
            {
                joint: float(value)
                for joint, value in zip(trajectory.joint_names, point.positions)
            }
        )
        fk = client.compute_fk(combined)
        coordinate = getattr(fk[link_name].position, axis_name)
        samples.append((float(coordinate), [float(value) for value in point.positions]))
    if len(samples) > 1 and samples[-1][0] < samples[0][0]:
        samples.reverse()
    coordinates = [item[0] for item in samples]
    if any(
        coordinates[index + 1] < coordinates[index] - 1e-9
        for index in range(len(coordinates) - 1)
    ):
        raise RuntimeError(f"{link_name}: Cartesian path is not monotonic along {axis_name}")
    return samples


def _interpolate_axis(
    samples: Sequence[tuple[float, Sequence[float]]],
    target: float,
) -> list[float]:
    if not samples:
        raise ValueError("axis sample list is empty")
    if target <= samples[0][0]:
        return [float(value) for value in samples[0][1]]
    if target >= samples[-1][0]:
        return [float(value) for value in samples[-1][1]]
    for index in range(len(samples) - 1):
        left_coordinate, left_values = samples[index]
        right_coordinate, right_values = samples[index + 1]
        if left_coordinate <= target <= right_coordinate:
            span = right_coordinate - left_coordinate
            blend = 0.0 if span <= 1e-9 else (target - left_coordinate) / span
            return [
                float(left_values[joint]) * (1.0 - blend)
                + float(right_values[joint]) * blend
                for joint in range(len(left_values))
            ]
    return [float(value) for value in samples[-1][1]]


def _duration(seconds: float) -> Duration:
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - int(seconds)) * 1e9))
    if duration.nanosec >= 1000000000:
        duration.sec += 1
        duration.nanosec -= 1000000000
    return duration


def _build_synchronized_trajectory(
    left_samples: Sequence[Sequence[float]],
    right_samples: Sequence[Sequence[float]],
    duration_s: float,
) -> JointTrajectory:
    if len(left_samples) != len(right_samples) or len(left_samples) < 2:
        raise ValueError("left/right sample counts must match and be at least 2")
    trajectory = JointTrajectory()
    trajectory.joint_names = list(ALL_JOINTS)
    count = len(left_samples)
    dt = duration_s / (count - 1)
    for index in range(count):
        point = JointTrajectoryPoint()
        point.positions = [
            float(value) for value in left_samples[index]
        ] + [float(value) for value in right_samples[index]]
        point.velocities = []
        if 0 < index < count - 1:
            point.velocities = [
                (left_samples[index + 1][joint] - left_samples[index - 1][joint])
                / (2.0 * dt)
                for joint in range(7)
            ] + [
                (right_samples[index + 1][joint] - right_samples[index - 1][joint])
                / (2.0 * dt)
                for joint in range(7)
            ]
        else:
            point.velocities = [0.0] * len(ALL_JOINTS)
        point.time_from_start = _duration(index * dt)
        trajectory.points.append(point)
    return trajectory


def _geometry(
    fk: Mapping[str, Pose],
    config: Mapping,
    fraction: float,
) -> dict:
    center = config["start_center"]
    direction = config["direction"]
    axis = config["separation_axis"]
    half = config["hand_separation_m"] / 2.0
    offset = config["distance_m"] * fraction
    desired_center = tuple(
        center[index] + direction[index] * offset for index in range(3)
    )
    desired_left = tuple(
        desired_center[index] + axis[index] * half for index in range(3)
    )
    desired_right = tuple(
        desired_center[index] - axis[index] * half for index in range(3)
    )
    left = fk["left_ee_link"]
    right = fk["right_ee_link"]
    left_xyz = (left.position.x, left.position.y, left.position.z)
    right_xyz = (right.position.x, right.position.y, right.position.z)
    difference = tuple(right_xyz[i] - left_xyz[i] for i in range(3))
    separation = abs(sum(difference[i] * axis[i] for i in range(3)))
    perpendicular = math.sqrt(
        max(
            0.0,
            sum(value * value for value in difference) - separation * separation,
        )
    )
    left_error = math.sqrt(
        sum((left_xyz[i] - desired_left[i]) ** 2 for i in range(3))
    )
    right_error = math.sqrt(
        sum((right_xyz[i] - desired_right[i]) ** 2 for i in range(3))
    )
    left_quat = (
        left.orientation.x,
        left.orientation.y,
        left.orientation.z,
        left.orientation.w,
    )
    right_quat = (
        right.orientation.x,
        right.orientation.y,
        right.orientation.z,
        right.orientation.w,
    )
    return {
        "fraction": round(fraction, 6),
        "left_position": [round(value, 6) for value in left_xyz],
        "right_position": [round(value, 6) for value in right_xyz],
        "separation_m": round(separation, 6),
        "perpendicular_error_m": round(perpendicular, 6),
        "plane_difference_m": round(abs(left_xyz[2] - right_xyz[2]), 6),
        "left_position_error_m": round(left_error, 6),
        "right_position_error_m": round(right_error, 6),
        "left_orientation_error_rad": round(
            _quaternion_error_rad(config["left_xyzw"], left_quat), 6
        ),
        "right_orientation_error_rad": round(
            _quaternion_error_rad(config["right_xyzw"], right_quat), 6
        ),
    }


class CarryDemo:
    def __init__(self, args: argparse.Namespace, client: DualArmTaskClient) -> None:
        self.args = args
        self.client = client
        self.config = load_carry_config(args.config)
        self.poses = _carry_poses(self.config)
        self.motion = self.config["motion"]
        self.tolerance = self.config["validation"]

    def _seed_list(self) -> list[dict[str, float]]:
        current = self.client.positions()
        zeros = {joint: 0.0 for joint in ALL_JOINTS}
        seeds = [current, zeros]
        return seeds

    def _solve_start(self) -> tuple[dict[str, float], dict]:
        configured_left = self.config.get("start_left_rad")
        configured_right = self.config.get("start_right_rad")
        if configured_left is not None and configured_right is not None:
            combined = self.client.positions()
            combined.update(
                {
                    joint: configured_left[index]
                    for index, joint in enumerate(GROUP_JOINTS["left_arm"])
                }
            )
            combined.update(
                {
                    joint: configured_right[index]
                    for index, joint in enumerate(GROUP_JOINTS["right_arm"])
                }
            )
            valid, contacts = self.client.check_state(combined)
            if not valid:
                raise RuntimeError(
                    "configured carry start is in collision: " + str(contacts)
                )
            fk = self.client.compute_fk(combined)
            for side, pose in (
                ("left_ee_link", self.poses["left_pose_start"]),
                ("right_ee_link", self.poses["right_pose_start"]),
            ):
                actual = fk[side]
                position_error = math.sqrt(
                    (actual.position.x - pose.position.x) ** 2
                    + (actual.position.y - pose.position.y) ** 2
                    + (actual.position.z - pose.position.z) ** 2
                )
                orientation_error = _quaternion_error_rad(
                    (
                        actual.orientation.x,
                        actual.orientation.y,
                        actual.orientation.z,
                        actual.orientation.w,
                    ),
                    (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
                if position_error > 0.01 or orientation_error > 0.02:
                    raise RuntimeError(
                        "configured carry start does not match the target poses: "
                        f"{side} position={position_error:.4f} m, "
                        f"orientation={orientation_error:.4f} rad; re-run IK "
                        "after changing mounts or targets"
                    )
            return combined, {
                "method": "configured",
                "left_joints": [round(value, 6) for value in configured_left],
                "right_joints": [round(value, 6) for value in configured_right],
            }

        errors = []
        for index, seed in enumerate(self._seed_list()):
            try:
                left = self.client.compute_ik(
                    "left_arm", self.poses["left_pose_start"], [seed]
                )
                right = self.client.compute_ik(
                    "right_arm", self.poses["right_pose_start"], [seed]
                )
            except Exception as error:
                errors.append({"seed": index, "error": str(error)})
                continue
            combined = dict(seed)
            combined.update(left)
            combined.update(right)
            valid, contacts = self.client.check_state(combined)
            if valid:
                return combined, {
                    "seed_index": index,
                    "left_joints": [round(left[joint], 6) for joint in GROUP_JOINTS["left_arm"]],
                    "right_joints": [round(right[joint], 6) for joint in GROUP_JOINTS["right_arm"]],
                    "errors": errors,
                }
            errors.append({"seed": index, "contacts": contacts})
        raise RuntimeError(f"no collision-free IK pair for carry start: {errors}")

    def run(self) -> dict:
        self.client.wait_until_ready(
            timeout=self.args.timeout,
            require_actions=not self.args.plan_only,
            require_ik=True,
            require_cartesian=True,
        )
        self.client.apply_scene(
            self.args.scene_config, self.config["scene"], self.config["frame"]
        )
        start_positions, ik_metrics = self._solve_start()
        start_target = {
            joint: float(start_positions[joint]) for joint in ALL_JOINTS
        }
        planning_trajectory, planning = self.client.plan_joint(
            "both_arms",
            start_target,
            planning_time=self.motion["planning_time_s"],
            velocity_scaling=self.motion["velocity_scaling"],
            acceleration_scaling=self.motion["acceleration_scaling"],
            goal_tolerance=self.motion["goal_tolerance_rad"],
        )
        result = {
            "task": "carry",
            "scene": self.config["scene"],
            "start_center": list(self.config["start_center"]),
            "end_center": list(self.poses["end_center"]),
            "hand_separation_m": self.config["hand_separation_m"],
            "ik": ik_metrics,
            "planning": planning,
            "start_validation": None,
            "cartesian_paths": {},
            "transport": None,
            "execution": None,
            "final_geometry": None,
            "success": False,
        }
        if planning_trajectory is None:
            return result
        start_validation = self.client.validate_trajectory(
            planning_trajectory.joint_trajectory,
            samples=min(20, self.motion["samples"]),
        )
        result["start_validation"] = start_validation
        if start_validation["invalid_indices"] or start_validation["contacts"]:
            result["error"] = "start trajectory failed validation"
            return result
        if not self.args.plan_only:
            result["start_execution"] = self.client.execute_dual(
                planning_trajectory.joint_trajectory,
                goal_tolerance=self.motion["goal_tolerance_rad"],
            )
            if not result["start_execution"]["success"]:
                result["error"] = "failed to execute the carry-start alignment"
                return result

        waypoints_left = []
        waypoints_right = []
        waypoint_count = 5
        for index in range(waypoint_count):
            fraction = index / (waypoint_count - 1)
            left_position = [
                self.poses["left_start"][axis] * (1.0 - fraction)
                + self.poses["left_end"][axis] * fraction
                for axis in range(3)
            ]
            right_position = [
                self.poses["right_start"][axis] * (1.0 - fraction)
                + self.poses["right_end"][axis] * fraction
                for axis in range(3)
            ]
            waypoints_left.append(_pose(left_position, self.config["left_xyzw"]))
            waypoints_right.append(_pose(right_position, self.config["right_xyzw"]))

        left_path, left_meta = self.client.cartesian_path(
            "left_arm",
            "left_ee_link",
            start_positions,
            waypoints_left,
            max_step=self.motion["max_cartesian_step_m"],
            jump_threshold=self.motion["cartesian_jump_threshold"],
            revolute_jump_threshold=self.motion["revolute_jump_threshold_rad"],
            velocity_scaling=self.motion["velocity_scaling"],
            acceleration_scaling=self.motion["acceleration_scaling"],
        )
        right_path, right_meta = self.client.cartesian_path(
            "right_arm",
            "right_ee_link",
            start_positions,
            waypoints_right,
            max_step=self.motion["max_cartesian_step_m"],
            jump_threshold=self.motion["cartesian_jump_threshold"],
            revolute_jump_threshold=self.motion["revolute_jump_threshold_rad"],
            velocity_scaling=self.motion["velocity_scaling"],
            acceleration_scaling=self.motion["acceleration_scaling"],
        )
        result["cartesian_paths"] = {"left": left_meta, "right": right_meta}

        count = self.motion["samples"]
        axis_index = max(
            range(3), key=lambda index: abs(self.config["direction"][index])
        )
        left_axis_samples = _path_axis_samples(
            self.client,
            left_path,
            start_positions,
            "left_ee_link",
            axis_index,
        )
        right_axis_samples = _path_axis_samples(
            self.client,
            right_path,
            start_positions,
            "right_ee_link",
            axis_index,
        )
        left_samples = []
        right_samples = []
        states = []
        invalid = []
        contacts = set()
        geometry = []
        for index in range(count):
            fraction = index / (count - 1)
            offset = self.config["distance_m"] * fraction
            left_target = (
                self.poses["left_start"][axis_index]
                + self.config["direction"][axis_index] * offset
            )
            right_target = (
                self.poses["right_start"][axis_index]
                + self.config["direction"][axis_index] * offset
            )
            left = _interpolate_axis(left_axis_samples, left_target)
            right = _interpolate_axis(right_axis_samples, right_target)
            combined = dict(start_positions)
            combined.update(
                {joint: left[position] for position, joint in enumerate(GROUP_JOINTS["left_arm"])}
            )
            combined.update(
                {joint: right[position] for position, joint in enumerate(GROUP_JOINTS["right_arm"])}
            )
            valid, state_contacts = self.client.check_state(combined)
            if not valid:
                invalid.append(index)
            contacts.update(state_contacts)
            fk = self.client.compute_fk(combined)
            geometry.append(_geometry(fk, self.config, fraction))
            left_samples.append(left)
            right_samples.append(right)
            states.append(combined)

        max_separation_error = max(
            abs(item["separation_m"] - self.config["hand_separation_m"])
            for item in geometry
        )
        max_plane_error = max(item["plane_difference_m"] for item in geometry)
        max_position_error = max(
            max(item["left_position_error_m"], item["right_position_error_m"])
            for item in geometry
        )
        max_orientation_error = max(
            max(item["left_orientation_error_rad"], item["right_orientation_error_rad"])
            for item in geometry
        )
        transport = {
            "checked": count,
            "invalid_indices": invalid,
            "contacts": sorted(contacts),
            "max_separation_error_m": round(max_separation_error, 6),
            "max_plane_error_m": round(max_plane_error, 6),
            "max_position_error_m": round(max_position_error, 6),
            "max_orientation_error_rad": round(max_orientation_error, 6),
            "passed": (
                not invalid
                and not contacts
                and max_separation_error <= self.tolerance["separation_tolerance_m"]
                and max_plane_error <= self.tolerance["plane_tolerance_m"]
                and max_position_error <= self.tolerance["position_tolerance_m"]
                and max_orientation_error <= self.tolerance["orientation_tolerance_rad"]
            ),
            "samples": geometry,
        }
        result["transport"] = transport
        if not transport["passed"]:
            result["error"] = "synchronized carry path failed validation"
            return result

        synchronized = _build_synchronized_trajectory(
            left_samples, right_samples, self.motion["path_duration_s"]
        )
        if not self.args.plan_only:
            result["execution"] = self.client.execute_dual(
                synchronized,
                goal_tolerance=self.motion["goal_tolerance_rad"] * 2.0,
            )
            if not result["execution"]["success"]:
                result["error"] = "synchronized carry execution failed"
                return result
            final_fk = self.client.compute_fk(self.client.positions())
            final_geometry = _geometry(final_fk, self.config, 1.0)
            result["final_geometry"] = final_geometry
            final_ok = (
                abs(final_geometry["separation_m"] - self.config["hand_separation_m"])
                <= self.tolerance["separation_tolerance_m"]
                and final_geometry["plane_difference_m"]
                <= self.tolerance["plane_tolerance_m"]
                and max(
                    final_geometry["left_position_error_m"],
                    final_geometry["right_position_error_m"],
                )
                <= self.tolerance["position_tolerance_m"]
                and max(
                    final_geometry["left_orientation_error_rad"],
                    final_geometry["right_orientation_error_rad"],
                )
                <= self.tolerance["orientation_tolerance_rad"]
            )
            result["success"] = bool(result["execution"]["success"] and final_ok)
        else:
            result["success"] = bool(transport["passed"])
        return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="dual_carry.yaml path")
    parser.add_argument("--scene-config", default="", help="dual_scenes.yaml path")
    parser.add_argument(
        "--output", default="/tmp/litearm_dual_carry_metrics.json"
    )
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args(argv)


def default_path(filename: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("litearm_dual_arm"), "config", filename
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(remove_ros_args(argv or sys.argv)[1:])
    if not args.config:
        args.config = default_path("dual_carry.yaml")
    if not args.scene_config:
        args.scene_config = default_path("dual_scenes.yaml")
    rclpy.init()
    client = None
    try:
        client = DualArmTaskClient("litearm_dual_carry_demo")
        result = CarryDemo(args, client).run()
    except Exception as error:
        result = {
            "task": "carry",
            "success": False,
            "error": str(error),
        }
        print(f"dual_carry_demo: {error}", file=sys.stderr)
    finally:
        if client is not None:
            client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    write_json(args.output, result)
    print(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
