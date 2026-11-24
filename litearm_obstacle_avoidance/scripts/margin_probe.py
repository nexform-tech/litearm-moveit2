#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""margin_probe.py — 阶段 2 验收：夹爪保守包络与安全裕度确实参与碰撞检查。

验证思路
========
没有夹爪状态来源时，MoveIt 按闭合位显示/计算真实手指。如果只在「闭合
手指」所在位置放障碍，无论有没有包络，状态都会是无效的——测不出包络
是否生效。所以这里把障碍放在**只有张开手指才会占到的区域**：

    包络右手（夹爪基座系）x ∈ [-0.068 - m, +...]，真实右手闭合位约
    x ∈ [-0.025, -0.001]。局部 x = -0.055 处只有张开行程才会到达。

步骤：

  1. 从 /move_group 的 robot_description 确认包络 link 存在，读出 box 尺寸
  2. 用 /compute_fk 求零位下包络 link 的位姿，把夹爪基座系的探测点变到 base_link
  3. 零位状态本身必须有效（排除其它碰撞干扰）
  4. 在张开区放 2 cm 方块 → /check_state_validity 必须失败（包络生效）
  5. 在包络外缘外 m/2 处放 4 mm 小方块 → 必须失败（margin 生效）；移除恢复有效
  6. 所有方块移除后状态恢复有效

用法：

    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py use_rviz:=false
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    python3 margin_probe.py
"""

import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (CollisionObject, PlanningScene, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionFK,
                             GetStateValidity)
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

JOINTS = [f"joint{i}" for i in range(1, 8)]
ENVELOPE_LINK = "litegrip_finger_envelope_r"
RAW_ENVELOPE_X_HALF = 0.067226 / 2.0   # 包络未加 margin 时的 x 尺寸一半
RAW_OPEN_EDGE = -0.068                  # 右手完全张开时的外缘（基座系 x）
OPEN_PROBE_LOCAL = (-0.055, 0.0, 0.066483)  # 只有张开才占到的位置


def _call(node, client, request, timeout=20.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def _zero_state():
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = JOINTS + ["gripper_opening_joint"]
    state.joint_state.position = [0.0] * (len(JOINTS) + 1)
    return state


def _validity(node, client, state):
    request = GetStateValidity.Request()
    request.group_name = ""
    request.robot_state = state
    response = _call(node, client, request)
    return None if response is None else bool(response.valid)


def _apply_box(node, client, object_id, position, size, add):
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    obj = CollisionObject()
    obj.header.frame_id = "base_link"
    obj.id = object_id
    obj.operation = CollisionObject.ADD if add else CollisionObject.REMOVE
    if add:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size)] * 3
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
    scene.world.collision_objects.append(obj)
    response = _call(node, client, ApplyPlanningScene.Request(scene=scene))
    return bool(response and response.success)


def _pose_matrix(pose):
    matrix = np.eye(4)
    qx, qy, qz, qw = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
    matrix[:3, :3] = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def main():
    rclpy.init()
    node = Node("litearm_margin_probe")
    failures = 0
    try:
        param_client = node.create_client(GetParameters, "/move_group/get_parameters")
        apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        validity_client = node.create_client(GetStateValidity, "/check_state_validity")
        fk_client = node.create_client(GetPositionFK, "/compute_fk")
        services = {
            "/move_group/get_parameters": param_client,
            "/apply_planning_scene": apply_client,
            "/check_state_validity": validity_client,
            "/compute_fk": fk_client,
        }
        for name, client in services.items():
            if not client.wait_for_service(timeout_sec=60.0):
                print(f"✗ 服务 {name} 不可用")
                return 1

        # 1) robot_description 里必须有包络 link
        params = _call(node, param_client,
                       GetParameters.Request(names=["robot_description"]))
        if params is None or not params.values or not params.values[0].string_value:
            print("✗ 读不到 /move_group 的 robot_description")
            return 1
        urdf_text = params.values[0].string_value
        root = ET.fromstring(urdf_text)
        envelope_link = next((link for link in root.findall("link")
                              if link.get("name") == ENVELOPE_LINK), None)
        if envelope_link is None:
            print(f"✗ robot_description 里没有 {ENVELOPE_LINK}"
                  "（include_gripper_envelope 关了吗？）")
            return 1
        box = envelope_link.find("collision/geometry/box")
        if box is None:
            print(f"✗ {ENVELOPE_LINK} 没有 box 碰撞体")
            return 1
        size_x = float(box.get("size").split()[0])
        margin = (size_x - 2.0 * RAW_ENVELOPE_X_HALF) / 2.0
        print(f"✓ 包络存在: box x={size_x:.4f} m（margin≈{margin * 1000:.1f} mm）")

        # 2) 零位下包络 link 的位姿 → 把局部探测点变到 base_link
        fk_request = GetPositionFK.Request()
        fk_request.header = Header()
        fk_request.header.frame_id = "base_link"
        fk_request.fk_link_names = [ENVELOPE_LINK]
        fk_request.robot_state = _zero_state()
        fk_response = _call(node, fk_client, fk_request)
        if (fk_response is None or fk_response.error_code.val != 1
                or not fk_response.pose_stamped):
            print("✗ compute_fk 失败，无法定位包络")
            return 1
        transform = _pose_matrix(fk_response.pose_stamped[0].pose)

        def to_base(local):
            point = transform @ np.array([local[0], local[1], local[2], 1.0])
            return tuple(round(float(v), 5) for v in point[:3])

        open_point = to_base(OPEN_PROBE_LOCAL)
        margin_point = to_base((RAW_OPEN_EDGE - margin / 2.0, 0.0,
                                OPEN_PROBE_LOCAL[2]))
        print(f"  张开区探测点(base_link): {open_point}")
        print(f"  margin 探测点(base_link): {margin_point}")

        # 3) 零位状态本身有效
        if not _validity(node, validity_client, _zero_state()):
            print("✗ 零位状态本身无效（其它碰撞对没配好），无法继续")
            return 1
        print("✓ 零位状态有效")

        # 4) 张开区放障碍 → 必须无效
        if not _apply_box(node, apply_client, "margin_probe_open",
                          open_point, 0.02, add=True):
            print("✗ 添加张开区方块失败")
            return 1
        open_valid = _validity(node, validity_client, _zero_state())
        _apply_box(node, apply_client, "margin_probe_open", open_point, 0.02, add=False)
        if open_valid:
            failures += 1
            print("✗ 张开区放障碍后状态仍然有效：包络没有参与碰撞检查")
        else:
            print("✓ 张开区放障碍后状态无效：包络覆盖了张开行程")

        # 5) 包络外缘外 m/2 处放小方块 → margin 必须生效
        if margin < 0.002:
            print("  提示: margin < 2 mm，跳过 margin 探测")
        else:
            if not _apply_box(node, apply_client, "margin_probe_edge",
                              margin_point, 0.004, add=True):
                print("✗ 添加 margin 探测方块失败")
                return 1
            edge_valid = _validity(node, validity_client, _zero_state())
            _apply_box(node, apply_client, "margin_probe_edge",
                       margin_point, 0.004, add=False)
            if edge_valid:
                failures += 1
                print("✗ 包络外缘外的方块未触发碰撞：margin 没有生效")
            else:
                print("✓ margin 探测方块触发碰撞：包络按 margin 外扩")

        # 6) 清理后恢复有效
        if not _validity(node, validity_client, _zero_state()):
            failures += 1
            print("✗ 探测方块移除后零位状态仍无效（场景未恢复）")
        else:
            print("✓ 探测方块已移除，零位状态恢复有效")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failures:
        print(f"\n✗ 裕度验收失败（{failures} 项）")
        return 1
    print("\n✓ 保守包络与安全裕度验收通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
