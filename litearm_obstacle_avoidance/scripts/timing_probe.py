#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""timing_probe.py — 性能基线：输出规划时间与碰撞检查耗时。

在已启动的 move_group 上，用固定的起点与目标重复规划，统计：

  * 规划墙钟耗时：/plan_kinematic_path 服务往返时间；
  * MoveIt 内部规划时间：motion_plan_response.planning_time；
  * 碰撞检查耗时：对每次返回轨迹的每个点调用 /check_state_validity，
    统计单点往返耗时与单条轨迹总耗时。

起点固定为 ready 位形、目标固定为 GOAL，且 start_state.is_diff=False，
因此结果可重复，不受当前机器人位置与执行链影响。结果打印到终端，同时
写入 JSON（默认 ~/.ros/litearm_timing_baseline.json），纯信息、不设阈值。

用法：

    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py use_rviz:=false
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    python3 timing_probe.py --iterations 10
    # 或：acceptance_fake.sh --timing

退出码 0 = 至少完成一次有效规划，且所有碰撞检查调用都成功。
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime, timezone

import rclpy
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             RobotState, WorkspaceParameters)
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from rclpy.node import Node

JOINTS = [f"joint{i}" for i in range(1, 8)]
GROUP = "litearm_arm"
# ready 位形（supervisor 的默认撤退目标）→ scene_probe 使用的目标位形。
START = [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]
DEFAULT_OUTPUT = "~/.ros/litearm_timing_baseline.json"


def _call(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def _percentile(ordered, quantile):
    """最近秩（nearest-rank）分位数；ordered 必须已排序。"""
    index = math.ceil(quantile * len(ordered)) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def _stats(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    count = len(ordered)
    return {
        "count": count,
        "mean": round(sum(ordered) / count, 3),
        "median": round(_percentile(ordered, 0.5), 3),
        "p95": round(_percentile(ordered, 0.95), 3),
        "min": round(ordered[0], 3),
        "max": round(ordered[-1], 3),
    }


def _state(positions):
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = JOINTS + ["gripper_opening_joint"]
    state.joint_state.position = [float(v) for v in positions] + [0.0]
    return state


def _plan_request(args):
    request = MotionPlanRequest()
    request.group_name = GROUP
    request.pipeline_id = "ompl"
    request.planner_id = args.planner_id
    request.num_planning_attempts = args.attempts
    request.allowed_planning_time = args.planning_time
    request.max_velocity_scaling_factor = args.velocity_scaling
    request.max_acceleration_scaling_factor = args.velocity_scaling

    request.workspace_parameters = WorkspaceParameters()
    request.workspace_parameters.header.frame_id = "base_link"
    request.workspace_parameters.min_corner.x = -1.0
    request.workspace_parameters.min_corner.y = -1.0
    request.workspace_parameters.min_corner.z = -1.0
    request.workspace_parameters.max_corner.x = 1.0
    request.workspace_parameters.max_corner.y = 1.0
    request.workspace_parameters.max_corner.z = 1.0

    # 固定起点，避免当前机器人位置影响基线。
    request.start_state = _state(START)

    constraints = Constraints()
    for name, value in zip(JOINTS, GOAL):
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(value)
        joint.tolerance_above = 0.05
        joint.tolerance_below = 0.05
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    request.goal_constraints.append(constraints)
    return request


def _print_stats(label, stats, unit="ms"):
    if not stats.get("count"):
        print(f"  {label}: 无样本")
        return
    print(f"  {label}: mean {stats['mean']:.1f} {unit}, "
          f"median {stats['median']:.1f} {unit}, "
          f"p95 {stats['p95']:.1f} {unit}, "
          f"min/max {stats['min']:.1f}/{stats['max']:.1f} {unit} "
          f"(n={stats['count']})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=10,
                        help="规划采样次数（默认 10）")
    parser.add_argument("--planning-time", type=float, default=2.0,
                        help="每次规划的 allowed_planning_time，单位秒（默认 2.0）")
    parser.add_argument("--attempts", type=int, default=5,
                        help="num_planning_attempts（默认 5）")
    parser.add_argument("--velocity-scaling", type=float, default=0.2,
                        help="最大速度/加速度缩放（默认 0.2）")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault",
                        help="规划器 id（默认 RRTConnectkConfigDefault）")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="单次服务调用超时，单位秒（默认 30）")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help=f"JSON 基线输出路径（默认 {DEFAULT_OUTPUT}；传空字符串则不落盘）")
    args = parser.parse_args()

    if args.iterations <= 0:
        print("✗ --iterations 必须为正整数")
        return 2

    rclpy.init()
    node = Node("litearm_timing_probe")
    failures = 0
    plan_errors = 0
    check_errors = 0
    wall_samples = []
    moveit_samples = []
    point_samples = []
    per_point_samples = []
    per_trajectory_samples = []
    try:
        plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        validity_client = node.create_client(
            GetStateValidity, "/check_state_validity")
        for name, client in (("/plan_kinematic_path", plan_client),
                             ("/check_state_validity", validity_client)):
            if not client.wait_for_service(timeout_sec=60.0):
                print(f"✗ 服务 {name} 不可用（move_group 未就绪？）")
                return 1

        request = _plan_request(args)
        print(f"── 性能基线：{args.iterations} 次规划（{GROUP}: "
              f"{tuple(round(v, 3) for v in START)} → "
              f"{tuple(round(v, 3) for v in GOAL)}）──")
        for index in range(args.iterations):
            started = time.perf_counter()
            response = _call(node, plan_client,
                             GetMotionPlan.Request(motion_plan_request=request),
                             args.timeout)
            wall_ms = (time.perf_counter() - started) * 1000.0
            if response is None:
                plan_errors += 1
                print(f"  [{index + 1}/{args.iterations}] 规划请求超时"
                      f"（{args.timeout:.0f} s）")
                continue
            plan_response = response.motion_plan_response
            trajectory = plan_response.trajectory.joint_trajectory
            code = plan_response.error_code.val
            if code != 1 or not trajectory.points:
                plan_errors += 1
                print(f"  [{index + 1}/{args.iterations}] 规划失败 "
                      f"error_code={code}")
                continue

            moveit_ms = float(plan_response.planning_time) * 1000.0
            wall_samples.append(wall_ms)
            moveit_samples.append(moveit_ms)
            point_samples.append(len(trajectory.points))

            # 轨迹逐点碰撞检查：测的是 /check_state_validity 的服务往返。
            trajectory_check_ms = 0.0
            trajectory_ok = True
            validity_request = GetStateValidity.Request()
            validity_request.group_name = ""
            point_count = 0
            for point in trajectory.points:
                validity_request.robot_state = _state(point.positions)
                check_started = time.perf_counter()
                validity = _call(node, validity_client, validity_request,
                                 args.timeout)
                check_ms = (time.perf_counter() - check_started) * 1000.0
                if validity is None:
                    check_errors += 1
                    trajectory_ok = False
                    continue
                per_point_samples.append(check_ms)
                trajectory_check_ms += check_ms
                point_count += 1
            if trajectory_ok and point_count:
                per_trajectory_samples.append(trajectory_check_ms)

            print(f"  [{index + 1}/{args.iterations}] "
                  f"规划墙钟 {wall_ms:.1f} ms, MoveIt {moveit_ms:.1f} ms, "
                  f"轨迹点 {len(trajectory.points)}, "
                  f"碰撞检查 {trajectory_check_ms:.1f} ms"
                  f"{'' if trajectory_ok else '（有调用超时）'}")

        if not wall_samples:
            print("✗ 没有任何一次规划成功，无法形成基线")
            return 1
        if check_errors:
            failures += 1
            print(f"✗ {check_errors} 次碰撞检查调用超时，基线不完整")

        wall_stats = _stats(wall_samples)
        moveit_stats = _stats(moveit_samples)
        point_stats = _stats(point_samples)
        per_point_stats = _stats(per_point_samples)
        per_trajectory_stats = _stats(per_trajectory_samples)

        print("\n── 基线摘要 ──")
        _print_stats("规划墙钟", wall_stats)
        _print_stats("MoveIt 内部规划", moveit_stats)
        _print_stats("单条轨迹点数", point_stats, unit="点")
        _print_stats("单点碰撞检查", per_point_stats)
        _print_stats("单条轨迹碰撞检查总耗时", per_trajectory_stats)
        if plan_errors:
            print(f"  注意: {plan_errors} 次规划失败/超时未计入统计")

        baseline = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "scenario": {
                "group": GROUP,
                "start": START,
                "goal": GOAL,
                "planner_id": args.planner_id,
                "allowed_planning_time_s": args.planning_time,
                "num_planning_attempts": args.attempts,
                "velocity_scaling": args.velocity_scaling,
                "iterations_requested": args.iterations,
                "iterations_succeeded": len(wall_samples),
                "planning_failures": plan_errors,
            },
            "planning": {
                "wall_ms": wall_stats,
                "moveit_planning_ms": moveit_stats,
                "trajectory_points": point_stats,
            },
            "collision_check": {
                "mode": "check_state_validity_per_trajectory_point",
                "points_checked": len(per_point_samples),
                "per_point_ms": per_point_stats,
                "per_trajectory_total_ms": per_trajectory_stats,
                "call_timeouts": check_errors,
            },
        }

        if args.output:
            output = os.path.abspath(os.path.expanduser(args.output))
            os.makedirs(os.path.dirname(output), exist_ok=True)
            with open(output, "w", encoding="utf-8") as handle:
                json.dump(baseline, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
            print(f"  结果已写入: {output}")
        else:
            print("  （--output 为空，未写入 JSON）")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failures:
        print(f"\n✗ 性能基线采集失败（{failures} 项）")
        return 1
    print("\n✓ 性能基线采集完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
