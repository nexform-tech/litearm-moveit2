#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_alternating_demo.py — 共享工作空间里的双臂交替作业（无夹爪）。

按 dual_alternating.yaml 的阶段顺序执行：

    both_arms 回 ready
      → left_arm 进入共享区 → left_arm 回撤
      → right_arm 进入共享区 → right_arm 回撤
      → both_arms 回 ready

单臂阶段只给 active 侧下发轨迹；另一侧保持当前状态并继续参与
/check_state_validity 的 full-robot 碰撞检查。执行后检查空闲臂偏差。

    ros2 run litearm_dual_arm dual_alternating_demo.py --plan-only
    ros2 run litearm_dual_arm dual_alternating_demo.py
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Mapping, Sequence

import rclpy
import yaml
from rclpy.utilities import remove_ros_args

from dual_arm_task_client import (
    GROUP_JOINTS,
    DualArmTaskClient,
    write_json,
)


def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def load_targets(path: str) -> dict:
    config = _load_yaml(path)
    targets = config.get("targets")
    if not isinstance(targets, dict) or not targets:
        raise ValueError(f"goals file has no targets mapping: {path}")
    return targets


def load_target(targets: Mapping, name: str) -> tuple[str, dict[str, float]]:
    entry = targets.get(name)
    if not isinstance(entry, dict):
        raise ValueError(f"unknown target {name!r}")
    group = str(entry.get("group", "")).strip()
    if group not in GROUP_JOINTS:
        raise ValueError(f"target {name!r} has unknown group {group!r}")
    positions = entry.get("positions")
    if not isinstance(positions, dict):
        raise ValueError(f"target {name!r} has no positions mapping")
    parsed = {}
    for joint, value in positions.items():
        if joint not in GROUP_JOINTS[group]:
            raise ValueError(f"target {name!r} joint {joint!r} is not in {group!r}")
        parsed[joint] = float(value)
    if set(parsed) != set(GROUP_JOINTS[group]):
        raise ValueError(f"target {name!r} does not cover all joints in {group!r}")
    return group, parsed


def load_alternating_config(path: str) -> dict:
    config = _load_yaml(path)
    if not isinstance(config, dict):
        raise ValueError("alternating configuration must be a mapping")
    frame = str(config.get("frame", "world")).strip()
    if frame != "world":
        raise ValueError("alternating configuration currently supports frame=world only")
    stages = config.get("stages")
    if not isinstance(stages, list) or not stages:
        raise ValueError("alternating configuration needs a non-empty stages list")
    parsed_stages = []
    for index, stage in enumerate(stages):
        if not isinstance(stage, dict):
            raise ValueError(f"stage {index} must be a mapping")
        name = str(stage.get("name", f"stage_{index}")).strip()
        active = str(stage.get("active", "")).strip()
        target = str(stage.get("target", "")).strip()
        if active not in GROUP_JOINTS:
            raise ValueError(f"stage {name!r} has unknown active group {active!r}")
        if not target:
            raise ValueError(f"stage {name!r} has no target")
        parsed_stages.append({"name": name, "active": active, "target": target})
    motion = config.get("motion") or {}
    validation = config.get("validation") or {}
    result = {
        "frame": frame,
        "scene": str(config.get("scene", "none")).strip() or "none",
        "stages": parsed_stages,
        "motion": {
            "planning_time_s": float(motion.get("planning_time_s", 10.0)),
            "velocity_scaling": float(motion.get("velocity_scaling", 0.10)),
            "acceleration_scaling": float(motion.get("acceleration_scaling", 0.10)),
            "samples": int(motion.get("samples", 20)),
            "goal_tolerance_rad": float(motion.get("goal_tolerance_rad", 0.02)),
            "execute_timeout_s": float(motion.get("execute_timeout_s", 30.0)),
        },
        "validation": {
            "idle_tolerance_rad": float(validation.get("idle_tolerance_rad", 0.02)),
            "position_tolerance_m": float(validation.get("position_tolerance_m", 0.01)),
        },
    }
    if result["motion"]["samples"] < 2:
        raise ValueError("motion.samples must be at least 2")
    return result


class AlternatingDemo:
    def __init__(
        self,
        args: argparse.Namespace,
        client: DualArmTaskClient,
        targets: Mapping,
    ) -> None:
        self.args = args
        self.client = client
        self.targets = targets
        self.config = load_alternating_config(args.config)
        self.motion = self.config["motion"]
        self.tolerance = self.config["validation"]

    def _execute_stage(self, stage: Mapping) -> dict:
        group, target = load_target(self.targets, stage["target"])
        if group != stage["active"]:
            raise ValueError(
                f"stage {stage['name']!r} active={stage['active']!r} but "
                f"target {stage['target']!r} belongs to {group!r}"
            )
        trajectory, planning = self.client.plan_joint(
            group,
            target,
            planning_time=self.motion["planning_time_s"],
            velocity_scaling=self.motion["velocity_scaling"],
            acceleration_scaling=self.motion["acceleration_scaling"],
            goal_tolerance=self.motion["goal_tolerance_rad"],
        )
        result = {
            "name": stage["name"],
            "active": group,
            "target": stage["target"],
            "planning": planning,
            "validation": None,
            "execution": None,
            "success": False,
        }
        if trajectory is None:
            return result
        validation = self.client.validate_trajectory(
            trajectory.joint_trajectory,
            samples=self.motion["samples"],
        )
        result["validation"] = validation
        planning_ok = (
            planning.get("success", False)
            and validation["checked"] > 0
            and not validation["invalid_indices"]
            and not validation["contacts"]
        )
        if not planning_ok:
            result["error"] = "planning or path validation failed"
            return result
        if not self.args.plan_only:
            if group == "both_arms":
                execution = self.client.execute_dual(
                    trajectory.joint_trajectory,
                    goal_tolerance=self.motion["goal_tolerance_rad"],
                )
            else:
                execution = self.client.execute_single(
                    group,
                    trajectory.joint_trajectory,
                    goal_tolerance=self.motion["goal_tolerance_rad"],
                    idle_tolerance=self.tolerance["idle_tolerance_rad"],
                )
            result["execution"] = execution
            result["success"] = bool(execution.get("success"))
        else:
            result["success"] = True
        return result

    def run(self) -> dict:
        self.client.wait_until_ready(
            timeout=self.args.timeout,
            require_actions=not self.args.plan_only,
        )
        self.client.apply_scene(
            self.args.scene_config, self.config["scene"], self.config["frame"]
        )
        result = {
            "task": "alternating",
            "scene": self.config["scene"],
            "stages": [],
            "success": False,
        }
        for stage in self.config["stages"]:
            self.client.get_logger().info(
                f"alternating stage {stage['name']}: {stage['active']} -> {stage['target']}"
            )
            try:
                stage_result = self._execute_stage(stage)
            except Exception as error:
                stage_result = {
                    "name": stage["name"],
                    "active": stage["active"],
                    "target": stage["target"],
                    "success": False,
                    "error": str(error),
                }
            result["stages"].append(stage_result)
            if not stage_result.get("success"):
                result["error"] = f"stage {stage['name']!r} failed"
                break
        result["success"] = (
            bool(result["stages"])
            and all(stage.get("success") for stage in result["stages"])
            and len(result["stages"]) == len(self.config["stages"])
        )
        return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="dual_alternating.yaml path")
    parser.add_argument("--goals", default="", help="dual_goals.yaml path")
    parser.add_argument("--scene-config", default="", help="dual_scenes.yaml path")
    parser.add_argument(
        "--output", default="/tmp/litearm_dual_alternating_metrics.json"
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
        args.config = default_path("dual_alternating.yaml")
    if not args.goals:
        args.goals = default_path("dual_goals.yaml")
    if not args.scene_config:
        args.scene_config = default_path("dual_scenes.yaml")
    rclpy.init()
    client = None
    try:
        client = DualArmTaskClient("litearm_dual_alternating_demo")
        targets = load_targets(args.goals)
        result = AlternatingDemo(args, client, targets).run()
    except Exception as error:
        result = {
            "task": "alternating",
            "success": False,
            "error": str(error),
        }
        print(f"dual_alternating_demo: {error}", file=sys.stderr)
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
