#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""scene_probe.py — 阶段 1 验收：静态环境进入 PlanningScene 并参与碰撞检查。

在已启动的 move_group（含 scene_builder）上验证：

  1. scene.yaml 里的对象确实在 PlanningScene 中（/get_planning_scene）
  2. 有环境时规划到目标成功（/plan_kinematic_path）
  3. 返回轨迹的**每个点**都通过 /check_state_validity（不穿越工作台/障碍）
  4. 反向用例：在目标位姿外放一个盒子把末端包住，规划必须失败
  5. 反向用例结束后移除盒子，场景恢复

用法：

    # 先起带场景的栈（obstacle_fake 默认加载 config/scene.yaml）
    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py use_rviz:=false
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    python3 scene_probe.py            # 或用 acceptance_fake.sh --scene

退出码 0 = 全部通过。
"""

import argparse
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint,
                             MotionPlanRequest, PlanningScene,
                             PlanningSceneComponents, RobotState,
                             WorkspaceParameters)
from moveit_msgs.srv import (ApplyPlanningScene, GetPlanningScene,
                             GetPositionFK, GetMotionPlan, GetStateValidity)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

JOINTS = [f"joint{i}" for i in range(1, 8)]
GROUP = "litearm_arm"
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]
TOLERANCE_RAD = 0.05
BLOCKER_ID = "scene_probe_goal_blocker"
BLOCKER_SIZE = 0.24  # m，包住目标位姿处的末端


def _call(node, client, request, timeout=30.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def _wait_services(node, clients, timeout=60.0):
    deadline = node.get_clock().now().nanoseconds / 1e9 + timeout
    for name, client in clients.items():
        remaining = max(0.1, deadline - node.get_clock().now().nanoseconds / 1e9)
        if not client.wait_for_service(timeout_sec=remaining):
            print(f"✗ 服务 {name} 不可用（move_group 未就绪？）")
            return False
    return True


def _state(joint_names, positions, include_gripper=True):
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = list(joint_names)
    state.joint_state.position = [float(v) for v in positions]
    if include_gripper and "gripper_opening_joint" not in state.joint_state.name:
        # 模型里夹爪没有状态来源；补 0（闭合位），让状态完整。
        state.joint_state.name.append("gripper_opening_joint")
        state.joint_state.position.append(0.0)
    return state


def _plan_request():
    request = MotionPlanRequest()
    request.group_name = GROUP
    request.pipeline_id = "ompl"
    request.planner_id = "RRTConnectkConfigDefault"
    request.num_planning_attempts = 5
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.2
    request.max_acceleration_scaling_factor = 0.2

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
    for name, value in zip(JOINTS, GOAL):
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(value)
        joint.tolerance_above = TOLERANCE_RAD
        joint.tolerance_below = TOLERANCE_RAD
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    request.goal_constraints.append(constraints)
    return request


def _scene_names(node, client):
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    response = _call(node, client, request)
    if response is None:
        return None
    return {obj.id for obj in response.scene.world.collision_objects}


def _apply_box(node, client, add):
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    if add:
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = BLOCKER_ID
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [BLOCKER_SIZE] * 3
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = BLOCKER_POSE
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
    else:
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = BLOCKER_ID
        obj.operation = CollisionObject.REMOVE
    scene.world.collision_objects.append(obj)
    response = _call(node, client, ApplyPlanningScene.Request(scene=scene))
    return bool(response and response.success)


BLOCKER_POSE = (0.0, 0.0, 0.0)  # 由 compute_fk 结果填充


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-file", default="",
                        help="用于读取期望对象 id 的 scene.yaml；空 = 本包默认")
    args = parser.parse_args()

    scene_file = args.scene_file or (
        get_package_share_directory("litearm_obstacle_avoidance")
        + "/config/scene.yaml")
    with open(scene_file, "r", encoding="utf-8") as handle:
        expected_ids = {spec["id"] for spec in yaml.safe_load(handle)["objects"]}
    print(f"期望场景对象: {sorted(expected_ids)}")

    rclpy.init()
    node = Node("litearm_scene_probe")
    failures = 0
    try:
        clients = {
            "/get_planning_scene": node.create_client(GetPlanningScene, "/get_planning_scene"),
            "/plan_kinematic_path": node.create_client(GetMotionPlan, "/plan_kinematic_path"),
            "/check_state_validity": node.create_client(GetStateValidity, "/check_state_validity"),
            "/compute_fk": node.create_client(GetPositionFK, "/compute_fk"),
            "/apply_planning_scene": node.create_client(ApplyPlanningScene, "/apply_planning_scene"),
        }
        if not _wait_services(node, clients):
            return 1

        # 1) 场景对象就位（scene_builder 可能在 move_group 之后才生效）
        deadline = node.get_clock().now().nanoseconds / 1e9 + 60.0
        names = set()
        while node.get_clock().now().nanoseconds / 1e9 < deadline:
            names = _scene_names(node, clients["/get_planning_scene"]) or set()
            if expected_ids.issubset(names):
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        if not expected_ids.issubset(names):
            print(f"✗ 场景对象缺失: 期望 {sorted(expected_ids)}，实际 {sorted(names)}")
            return 1
        print(f"✓ 场景对象已进入 PlanningScene: {sorted(expected_ids)}")

        # 2) 有环境时规划
        response = _call(node, clients["/plan_kinematic_path"],
                         GetMotionPlan.Request(motion_plan_request=_plan_request()),
                         timeout=60.0)
        if response is None:
            print("✗ /plan_kinematic_path 超时")
            return 1
        code = response.motion_plan_response.error_code.val
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        print(f"✓ 有环境时规划: error_code={code} "
              f"({'SUCCESS' if code == 1 else 'FAILURE'})，"
              f"轨迹点 {len(trajectory.points)}")
        if code != 1 or not trajectory.points:
            print("✗ 有环境时规划失败")
            return 1

        # 3) 轨迹逐点有效性
        invalid = 0
        request = GetStateValidity.Request()
        request.group_name = ""
        for point in trajectory.points:
            request.robot_state = _state(trajectory.joint_names, point.positions)
            validity = _call(node, clients["/check_state_validity"], request)
            if validity is None or not validity.valid:
                invalid += 1
        if invalid:
            failures += 1
            print(f"✗ 轨迹中有 {invalid}/{len(trajectory.points)} 个状态无效")
        else:
            print(f"✓ 轨迹全部 {len(trajectory.points)} 个状态通过碰撞检查")

        # 4) 反向用例：在目标末端包一个盒子，规划必须失败
        global BLOCKER_POSE
        fk_request = GetPositionFK.Request()
        fk_request.header = Header()
        fk_request.header.frame_id = "base_link"
        fk_request.fk_link_names = ["ee_link"]
        fk_request.robot_state = _state(JOINTS, GOAL, include_gripper=False)
        fk_response = _call(node, clients["/compute_fk"], fk_request)
        if (fk_response is None or fk_response.error_code.val != 1
                or not fk_response.pose_stamped):
            print("✗ compute_fk 失败，无法构造反向用例")
            return 1
        pose = fk_response.pose_stamped[0].pose
        BLOCKER_POSE = (pose.position.x, pose.position.y, pose.position.z)
        print(f"  目标末端位姿: {tuple(round(v, 3) for v in BLOCKER_POSE)}")

        if not _apply_box(node, clients["/apply_planning_scene"], add=True):
            print("✗ 无法添加反向用例盒子")
            return 1
        blocked = _call(node, clients["/plan_kinematic_path"],
                        GetMotionPlan.Request(motion_plan_request=_plan_request()),
                        timeout=60.0)
        blocked_code = (blocked.motion_plan_response.error_code.val
                        if blocked is not None else None)
        _apply_box(node, clients["/apply_planning_scene"], add=False)
        if blocked is None:
            failures += 1
            print("✗ 反向用例规划请求超时（无法据此判定碰撞检查生效）")
        elif blocked_code == 1:
            failures += 1
            print("✗ 目标被盒子包住时规划仍然成功（碰撞检查没有生效？）")
        else:
            print(f"✓ 反向用例: 目标被包住时规划按预期失败 (error_code={blocked_code})")

        # 场景恢复
        names = _scene_names(node, clients["/get_planning_scene"]) or set()
        if BLOCKER_ID in names:
            print("✗ 反向用例盒子未被移除")
            failures += 1
        else:
            print("✓ 反向用例盒子已移除，场景恢复")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failures:
        print(f"\n✗ 场景验收失败（{failures} 项）")
        return 1
    print("\n✓ 静态环境验收通过（对象就位、轨迹有效、反向用例生效）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
