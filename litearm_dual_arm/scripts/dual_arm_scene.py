#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_arm_scene.py — 把 dual_scenes.yaml 的静态场景写入共享 Planning Scene。

move_group 只有一个 Planning Scene，左右臂共享同一批碰撞对象。这个节点在
启动后等待 /apply_planning_scene，把选中的场景一次性作为 diff 应用，然后退出；
场景会保留在 move_group 里。

    ros2 run litearm_dual_arm dual_arm_scene.py --scene dual_static

尺寸约定：YAML 里是物理尺寸，节点按 safety.obstacle_padding_m 向外膨胀。
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Sequence

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


OBJECT_PREFIX = "litearm_dual_"


def _finite_vector(value, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{field} must be a list of {length} numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} contains a non-finite number")
    return result


def load_scene(config_path: str, scene_name: str, frame: str) -> tuple[list[CollisionObject], list[ObjectColor]]:
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError(f"scene configuration must be a mapping: {config_path}")

    objects = config.get("objects") or {}
    scenes = config.get("scenes") or {}
    if scene_name not in scenes:
        raise ValueError(
            f"unknown scene {scene_name!r}; available: {sorted(scenes)}"
        )
    safety = config.get("safety") or {}
    padding = float(safety.get("obstacle_padding_m", 0.0) or 0.0)
    if not math.isfinite(padding) or padding < 0.0:
        raise ValueError("safety.obstacle_padding_m must be non-negative")

    collision_objects: list[CollisionObject] = []
    colors: list[ObjectColor] = []
    for name in scenes[scene_name]:
        if name not in objects:
            raise ValueError(f"scene {scene_name!r} references unknown object {name!r}")
        spec = objects[name]
        if not isinstance(spec, dict):
            raise ValueError(f"object {name!r} must be a mapping")

        kind = str(spec.get("type", "box")).strip().lower()
        pose_values = _finite_vector(spec.get("pose"), 3, f"{name}.pose")
        dimensions = _finite_vector(
            spec.get("dimensions"), 2 if kind == "cylinder" else 3, f"{name}.dimensions"
        )
        if any(value <= 0.0 for value in dimensions):
            raise ValueError(f"object {name!r} dimensions must be positive")

        primitive = SolidPrimitive()
        if kind == "box":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                dimensions[0] + 2.0 * padding,
                dimensions[1] + 2.0 * padding,
                dimensions[2] + 2.0 * padding,
            ]
        elif kind == "cylinder":
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [
                dimensions[0] + 2.0 * padding,
                dimensions[1] + padding,
            ]
        else:
            raise ValueError(f"object {name!r} has unsupported type {kind!r}")

        primitive_pose = Pose()
        primitive_pose.position.x, primitive_pose.position.y, primitive_pose.position.z = pose_values
        primitive_pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = frame
        collision.id = OBJECT_PREFIX + name
        collision.operation = CollisionObject.ADD
        collision.primitives = [primitive]
        collision.primitive_poses = [primitive_pose]
        collision_objects.append(collision)

        color_values = spec.get("color")
        if color_values is not None:
            rgba = _finite_vector(color_values, 4, f"{name}.color")
            color = ObjectColor()
            color.id = collision.id
            color.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            colors.append(color)

    return collision_objects, colors


def build_scene_message(
    collision_objects: Sequence[CollisionObject],
    colors: Sequence[ObjectColor],
) -> PlanningScene:
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = list(collision_objects)
    scene.object_colors = list(colors)
    return scene


class DualArmScene(Node):
    def __init__(self, config_path: str, scene_name: str, frame: str, timeout: float) -> None:
        super().__init__("litearm_dual_arm_scene")
        self._scene = build_scene_message(
            *load_scene(config_path, scene_name, frame)
        )
        self._client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/apply_planning_scene is not available; is move_group running?")
        future = self._client.call_async(ApplyPlanningScene.Request(scene=self._scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("timed out waiting for /apply_planning_scene")
        if not future.result().success:
            raise RuntimeError("move_group rejected the planning scene")
        self.get_logger().info(
            f"applied scene {scene_name!r}: {len(self._scene.world.collision_objects)} objects "
            f"in frame {frame!r}"
        )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="dual_scenes.yaml path")
    parser.add_argument("--scene", default="dual_static")
    parser.add_argument("--frame", default="", help="override the scene frame")
    parser.add_argument("--timeout", type=float, default=90.0)
    return parser.parse_args(argv)


def default_config() -> str:
    import os

    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("litearm_dual_arm"), "config", "dual_scenes.yaml"
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(remove_ros_args(argv or sys.argv)[1:])
    config_path = args.config or default_config()
    try:
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        frame = args.frame or str((config.get("frames") or {}).get("dual", "world"))
    except (OSError, yaml.YAMLError) as error:
        print(f"dual_arm_scene: {error}", file=sys.stderr)
        return 2

    rclpy.init()
    try:
        node = DualArmScene(config_path, args.scene, frame, args.timeout)
    except Exception as error:  # startup failure must be visible in launch logs
        print(f"dual_arm_scene: {error}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        # 应用后短暂自旋，确保服务响应和日志被处理完。
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
