#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""moveit_probe.py — acceptance probe for the MoveIt planning pipeline.

Verifies against an already running move_group:

  1. move_group is up and knows the litearm_arm planning group
  2. it can plan to the given joint goal (OMPL + KDL + the SRDF collision
     matrix all in effect)
  3. the planned trajectory has sensible joint names/point count/final point
  4. optionally: hand the plan to joint_trajectory_controller for real execution

Why /plan_kinematic_path is the default, not the MoveGroup action
------------------------------------------------------------
With planning_options.plan_only=true the MoveGroup action does not fill in the
planned_trajectory of its result (the behaviour of Humble MoveIt2), so there is
no planning result to verify. /plan_kinematic_path
(moveit_msgs/srv/GetMotionPlan), by contrast, is purely "plan and return" —
unambiguous semantics, no execution involved, which makes it the best fit for an
acceptance test.

Real execution just needs --execute; only then does it go through /move_action.

Usage:
  ros2 launch litearm_moveit_config litearm_moveit.launch.py dry_run:=true &
  python3 moveit_probe.py                # plan only (safe)
  python3 moveit_probe.py --execute      # plan and execute (drives the arm)
"""

import argparse
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             RobotState, WorkspaceParameters)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}" for i in range(1, 8)]
GROUP = "litearm_arm"
# The goal configuration matches the SRDF ready state. The all-zero pose is not
# used because it sits near a singularity, where KDL inverse kinematics and OMPL
# sampling both struggle more; the higher failure rate there would mask genuine
# problems.
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]
TOLERANCE_RAD = 0.05


def _build_request() -> MotionPlanRequest:
    request = MotionPlanRequest()
    request.group_name = GROUP
    # Specify the pipeline and planner explicitly: this does not depend on how
    # default_planning_pipeline resolves, and so sidesteps the trap where
    # "configuring only the plural planning_plugins degrades to CHOMP"
    # (an unconfigured CHOMP shows up as "succeeds but returns an empty
    # trajectory", which is very hard to pin down).
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

    # start state is_diff=True + empty content → move_group uses the state of
    # the current planning scene
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


def _check_trajectory(node, joint_trajectory) -> int:
    if len(joint_trajectory.points) == 0:
        print("✗ planning returned an empty trajectory")
        return 1
    names = list(joint_trajectory.joint_names)
    print(f"  trajectory joints: {names}")
    print(f"  trajectory points: {len(joint_trajectory.points)}")
    if sorted(names) != sorted(JOINTS):
        print(f"✗ trajectory joint set does not match the expectation (expected {JOINTS})")
        return 1
    last = list(joint_trajectory.points[-1].positions)
    order = [names.index(j) for j in JOINTS]
    last = [last[i] for i in order]
    worst = max(abs(last[i] - GOAL[i]) for i in range(7))
    print(f"  final point:       {[round(v, 4) for v in last]}")
    print(f"  goal:              {[round(v, 4) for v in GOAL]}")
    print(f"  final error:       {worst:.4e} rad (tolerance {TOLERANCE_RAD})")
    duration = joint_trajectory.points[-1].time_from_start
    print(f"  trajectory time:   {duration.sec + duration.nanosec * 1e-9:.3f}s")
    if worst > TOLERANCE_RAD:
        print("✗ planning final point is not within the goal tolerance")
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true",
                        help="actually execute the plan (drives the arm; make sure the space is clear first)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("litearm_moveit_probe")
    request = _build_request()

    if not args.execute:
        # ── plan only: go through the GetMotionPlan service ──
        client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        print("waiting for /plan_kinematic_path …")
        if not client.wait_for_service(timeout_sec=40.0):
            print("✗ /plan_kinematic_path unavailable (move_group not ready?)")
            return 1
        print("✓ move_group ready")

        future = client.call_async(GetMotionPlan.Request(motion_plan_request=request))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        response = future.result()
        if response is None:
            print("✗ planning service did not return within the timeout")
            return 1
        code = response.motion_plan_response.error_code.val
        print(f"✓ planning done: error_code={code} "
              f"({'SUCCESS' if code == 1 else 'FAILURE'}), "
              f"took {response.motion_plan_response.planning_time:.3f}s")
        if code != 1:
            print("✗ planning failed")
            return 1
        rc = _check_trajectory(
            node, response.motion_plan_response.trajectory.joint_trajectory)
        node.destroy_node()
        rclpy.shutdown()
        if rc != 0:
            return rc
        print("\n✓ MoveIt planning pipeline acceptance passed "
              "(nothing executed, the arm did not move)")
        return 0

    # ── plan and execute: go through the MoveGroup action ──
    #
    # The acceptance criterion is [/joint_states actually converging to the
    # goal], not the MoveGroup return code. The reason: move_group's MoveAction
    # spawns sub-goals while executing, and rclpy's ActionClient occasionally
    # reports "Ignoring unexpected goal response" (a goal response arriving with
    # a sequence number that is not the current one), and the result obtained
    # then may not reflect the real execution outcome. Joint angles are physical
    # facts, more trustworthy than a return code.
    live: dict = {}

    def on_joint_state(msg):
        for name, position in zip(msg.name, msg.position):
            live[name] = position

    node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(live) >= 7:
        print(f"  start position: {[round(live.get(j, float('nan')), 4) for j in JOINTS]}")
    else:
        print(f"✗ /joint_states did not provide 7 joints (got {sorted(live)})")
        return 1

    client = ActionClient(node, MoveGroup, "/move_action")
    print("waiting for /move_action …")
    if not client.wait_for_server(timeout_sec=40.0):
        print("✗ /move_action unavailable (move_group not ready?)")
        return 1
    print("✓ move_group ready")

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options.plan_only = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.replan = False

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    handle = future.result()
    if handle is None or not handle.accepted:
        print("✗ planning request was rejected")
        return 1
    print("✓ request accepted, planning and executing …")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=120)
    wrapped = result_future.result()
    if wrapped is not None:
        code = wrapped.result.error_code.val
        print(f"  MoveGroup return code: {code} "
              f"({'SUCCESS' if code == 1 else 'FAILURE/other'})")
    else:
        print("  MoveGroup did not return a result within the timeout "
              "(continuing with the joint-angle check)")

    # The trajectory is on the order of 3 s; give it a few more seconds for
    # tracking to converge
    for _ in range(120):
        rclpy.spin_once(node, timeout_sec=0.05)

    worst = 0.0
    for index, name in enumerate(JOINTS):
        worst = max(worst, abs(live.get(name, 99.0) - GOAL[index]))
    print(f"  measured position: {[round(live.get(j, float('nan')), 4) for j in JOINTS]}")
    print(f"  goal position:     {[round(v, 4) for v in GOAL]}")
    print(f"  max error:         {worst:.4e} rad (tolerance {TOLERANCE_RAD})")

    node.destroy_node()
    rclpy.shutdown()
    if worst >= TOLERANCE_RAD:
        print("✗ joint angles did not converge to the goal after execution")
        return 1
    print("\n✓ MoveIt planning + execution pipeline acceptance passed "
          "(joint angles converged to the goal)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
