#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""scene_builder.py — 把 scene.yaml 描述的静态环境写进 MoveIt PlanningScene。

数据流：

    scene.yaml -> scene_builder -> /apply_planning_scene (diff) -> PlanningScene

之后 move_group 的碰撞检查、规划、RViz 的 Scene Geometry 都会看到这些
物体。物体是**世界对象**（不是机器人 part）：不会被 attach，也不会写进
URDF/SRDF。

参数
====
    scene_file           场景 YAML 路径；空 = 本包 config/scene.yaml
    service_name         /apply_planning_scene
    remove_objects       true = 发送 REMOVE（清理场景）
    refresh_period       秒；>0 时周期性重发（move_group 重启后自动补回）
    remove_on_shutdown   退出时是否移除本文件描述的对象

支持的形状：box / cylinder / sphere。mesh 需要先行三角化（本阶段不做）。

⚠ 这是测试工位。真实工作台/夹具请按实测改 scene.yaml；场景坐标必须
  和 URDF 的固定坐标系一致（默认 base_link，底座下表面在 z=0）。
"""

import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def _pose_quaternion(rpy):
    """URDF 固定轴 roll-pitch-yaw (Rz(y)Ry(p)Rx(r)) 对应的四元数。"""
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r / 2.0), math.sin(r / 2.0)
    cp, sp = math.cos(p / 2.0), math.sin(p / 2.0)
    cy, sy = math.cos(y / 2.0), math.sin(y / 2.0)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def _make_pose(cfg):
    pose = Pose()
    xyz = cfg.get("xyz", [0.0, 0.0, 0.0])
    rpy = cfg.get("rpy", [0.0, 0.0, 0.0])
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    qx, qy, qz, qw = _pose_quaternion(rpy)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _make_primitive(spec, padding):
    """构造图元。

    moveit_msgs/msg/CollisionObject 在 Humble 里**没有** padding/scale 字段
    （那是 C++ CollisionObject 的内部属性），所以 scene.yaml 的 padding 在这
    里通过放大图元尺寸实现，效果等价于碰撞时的外扩：
      box: 三边各 +2p；cylinder: 半径 +p、高 +2p；sphere: 半径 +p。
    """
    shape = spec.get("shape")
    primitive = SolidPrimitive()
    if shape == "box":
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) + 2.0 * padding for v in spec["size"]]
    elif shape == "cylinder":
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(spec["height"]) + 2.0 * padding,
                                float(spec["radius"]) + padding]
    elif shape == "sphere":
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(spec["radius"]) + padding]
    else:
        raise ValueError(f"{spec.get('id')}: 不支持的 shape={shape!r}"
                         "（支持 box/cylinder/sphere）")
    return primitive


def _make_object(spec, default_frame, default_padding, operation):
    obj = CollisionObject()
    obj.header.frame_id = spec.get("frame_id", default_frame)
    obj.id = spec["id"]
    obj.operation = operation
    if operation == CollisionObject.ADD:
        padding = float(spec.get("padding", default_padding))
        obj.primitives.append(_make_primitive(spec, padding))
        obj.primitive_poses.append(_make_pose(spec.get("pose", {})))
    return obj


class SceneBuilder(Node):
    def __init__(self):
        super().__init__("scene_builder")
        self.declare_parameter("scene_file", "")
        self.declare_parameter("service_name", "/apply_planning_scene")
        self.declare_parameter("remove_objects", False)
        self.declare_parameter("refresh_period", 0.0)
        self.declare_parameter("remove_on_shutdown", False)

        scene_file = self.get_parameter("scene_file").value or ""
        if not scene_file:
            scene_file = (
                get_package_share_directory("litearm_obstacle_avoidance")
                + "/config/scene.yaml")
        self.scene_file = scene_file
        service_name = self.get_parameter("service_name").value

        with open(self.scene_file, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        self.frame_id = data.get("frame_id", "base_link")
        self.default_padding = float(data.get("padding", 0.0))
        self.specs = data.get("objects", [])
        if not self.specs:
            raise ValueError(f"{self.scene_file}: objects 为空")
        ids = [spec["id"] for spec in self.specs]
        if len(ids) != len(set(ids)):
            raise ValueError(f"{self.scene_file}: object id 重复")

        self.client = self.create_client(ApplyPlanningScene, service_name)
        self._refresh_pending = False
        self.get_logger().info(
            f"场景 {self.scene_file}: {len(self.specs)} 个对象 "
            f"{ids}，frame={self.frame_id}")

    def _scene_message(self, remove=False):
        operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        for spec in self.specs:
            scene.world.collision_objects.append(
                _make_object(spec, self.frame_id, self.default_padding, operation))
        return scene

    def apply(self, remove=False):
        if not self.client.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(
                f"{self.client.srv_name} 不可用（move_group 未就绪？）")
            return False

        future = self.client.call_async(
            ApplyPlanningScene.Request(scene=self._scene_message(remove)))
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        response = future.result()
        if response is None:
            self.get_logger().error("apply_planning_scene 调用超时")
            return False
        if not response.success:
            self.get_logger().error("move_group 拒绝了 planning scene diff")
            return False
        action = "移除" if remove else "应用"
        self.get_logger().info(
            f"已{action} {len(self.specs)} 个场景对象（frame={self.frame_id}）")
        return True

    def refresh(self):
        """定时器回调里用：非阻塞重发，避免在回调里嵌套 spin。"""
        if self._refresh_pending or not self.client.service_is_ready():
            return
        self._refresh_pending = True
        future = self.client.call_async(
            ApplyPlanningScene.Request(scene=self._scene_message(False)))

        def _done(done_future):
            self._refresh_pending = False
            response = done_future.result()
            if response is None or not response.success:
                self.get_logger().warning("场景刷新失败（move_group 未就绪？）")

        future.add_done_callback(_done)


def main():
    rclpy.init()
    node = None
    try:
        node = SceneBuilder()
        remove = bool(node.get_parameter("remove_objects").value)
        if not node.apply(remove=remove):
            return 1
        if remove:
            return 0

        period = float(node.get_parameter("refresh_period").value)
        if period > 0.0:
            node.create_timer(period, node.refresh)
            node.get_logger().info(f"每 {period:g} s 重发一次场景 diff")

        rclpy.spin(node)
        return 0
    except (ValueError, KeyError, OSError) as exc:
        if node is not None:
            node.get_logger().error(f"加载场景失败: {exc}")
        else:
            print(f"加载场景失败: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            if bool(node.get_parameter("remove_on_shutdown").value):
                node.apply(remove=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
