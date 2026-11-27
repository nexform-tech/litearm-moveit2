#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_arm_sim.py — 双臂确定性仿真：两路状态 + 两路 FollowJointTrajectory。

仿真接口刻意与真机的两套 namespaced 控制栈同形：

    左: /left/joint_states
        /left/joint_trajectory_controller/follow_joint_trajectory
    右: /right/joint_states
        /right/joint_trajectory_controller/follow_joint_trajectory

每个 action 只接收原生 ``joint1..joint7``；双臂模型里的
``left_jointN``/``right_jointN`` 由 dual_arm_bridge 负责转换。这样离线仿真
和真机共用同一套 bridge、MoveIt 控制器映射和验收探针。

这不是动力学仿真：没有重力、接触力、跟踪误差。它只按轨迹时间线性插值关节
位置，用来验证执行链路、同步、取消和 fail-closed 行为。

    ros2 run litearm_dual_arm dual_arm_sim.py
    ros2 run litearm_dual_arm dual_arm_sim.py --publish-only
    ros2 run litearm_dual_arm dual_arm_sim.py --initial-positions "0,...,0"

重置：``/litearm_dual_sim/reset`` (std_srvs/Trigger) 把两臂恢复到启动位形，
用于验收脚本在多个目标之间回到确定状态。
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


SIDES = ("left", "right")
BASE_JOINTS = tuple(f"joint{index}" for index in range(1, 8))
ALL_JOINTS = tuple(f"{side}_{joint}" for side in SIDES for joint in BASE_JOINTS)


def parse_positions(text: str) -> list[float]:
    if not text.strip():
        return [0.0] * len(ALL_JOINTS)
    try:
        values = [float(item) for item in text.split(",")]
    except ValueError as error:
        raise ValueError("initial positions must be comma-separated numbers") from error
    if len(values) != len(ALL_JOINTS):
        raise ValueError(
            f"initial positions need {len(ALL_JOINTS)} values, got {len(values)}"
        )
    return values


def _seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class DualArmSim(Node):
    def __init__(self, args: argparse.Namespace, initial: Sequence[float]) -> None:
        super().__init__("litearm_dual_arm_sim")
        self.args = args
        self._lock = threading.RLock()
        self._initial = {
            "left": [float(value) for value in initial[:7]],
            "right": [float(value) for value in initial[7:]],
        }
        self._positions = {side: list(values) for side, values in self._initial.items()}
        self._velocities = {side: [0.0] * 7 for side in SIDES}
        self._generation = {side: 0 for side in SIDES}
        self._active = {side: None for side in SIDES}

        self._publishers = {
            side: self.create_publisher(
                JointState,
                args.left_state_topic if side == "left" else args.right_state_topic,
                20,
            )
            for side in SIDES
        }
        self._timer = self.create_timer(1.0 / args.rate, self._publish)

        self._servers = {}
        group = ReentrantCallbackGroup()
        if not args.publish_only:
            for side in SIDES:
                action_name = (
                    args.left_action if side == "left" else args.right_action
                )
                self._servers[side] = ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    execute_callback=(
                        lambda goal_handle, side=side: self._execute(side, goal_handle)
                    ),
                    goal_callback=(
                        lambda goal, side=side: self._accept_goal(side, goal)
                    ),
                    cancel_callback=self._cancel,
                    callback_group=group,
                )

        self._reset_service = self.create_service(
            Trigger, args.reset_service, self._reset, callback_group=group
        )
        self.get_logger().info(
            f"dual-arm determinism sim: state {args.left_state_topic} + "
            f"{args.right_state_topic} at {args.rate:.1f} Hz; "
            + (
                "publish-only (no action servers)"
                if args.publish_only
                else f"actions {args.left_action} + {args.right_action}"
            )
        )

    def _publish(self) -> None:
        with self._lock:
            for side in SIDES:
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name = list(BASE_JOINTS)
                message.position = list(self._positions[side])
                message.velocity = list(self._velocities[side])
                message.effort = [0.0] * len(BASE_JOINTS)
                self._publishers[side].publish(message)

    @staticmethod
    def _indices(trajectory) -> list[int]:
        names = list(trajectory.joint_names)
        if set(names) != set(BASE_JOINTS):
            raise ValueError(
                f"expected joints {list(BASE_JOINTS)}, got {names}"
            )
        return [names.index(joint) for joint in BASE_JOINTS]

    def _accept_goal(self, side: str, goal):
        trajectory = goal.trajectory
        if len(trajectory.points) < 2:
            self.get_logger().error(f"{side}: reject trajectory with <2 points")
            return GoalResponse.REJECT
        try:
            indices = self._indices(trajectory)
        except ValueError as error:
            self.get_logger().error(f"{side}: {error}")
            return GoalResponse.REJECT
        if abs(_seconds(trajectory.points[0].time_from_start)) > 1e-6:
            self.get_logger().error(f"{side}: trajectory must start at t=0")
            return GoalResponse.REJECT
        with self._lock:
            current = list(self._positions[side])
        start = [float(trajectory.points[0].positions[index]) for index in indices]
        gap = max(abs(value - current[index]) for index, value in enumerate(start))
        if gap > self.args.start_tolerance:
            self.get_logger().error(
                f"{side}: reject start gap {gap:.4f} rad "
                f"(limit {self.args.start_tolerance:.4f})"
            )
            return GoalResponse.REJECT
        previous_time = 0.0
        previous = start
        for point in trajectory.points[1:]:
            stamp = _seconds(point.time_from_start)
            if stamp <= previous_time:
                self.get_logger().error(f"{side}: non-increasing trajectory time")
                return GoalResponse.REJECT
            values = [float(point.positions[index]) for index in indices]
            if not all(math.isfinite(value) for value in values):
                self.get_logger().error(f"{side}: non-finite trajectory value")
                return GoalResponse.REJECT
            velocity = max(
                abs(a - b) / (stamp - previous_time)
                for a, b in zip(values, previous)
            )
            if velocity > self.args.max_joint_velocity:
                self.get_logger().error(
                    f"{side}: reject trajectory with {velocity:.3f} rad/s "
                    f"above {self.args.max_joint_velocity:.3f}"
                )
                return GoalResponse.REJECT
            previous = values
            previous_time = stamp
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def _interpolate(trajectory, indices: list[int], elapsed: float):
        points = trajectory.points
        times = [_seconds(point.time_from_start) for point in points]
        if elapsed <= times[0]:
            index = 0
            values = [float(points[0].positions[i]) for i in indices]
            return values, [0.0] * len(indices)
        if elapsed >= times[-1]:
            last = points[-1]
            values = [float(last.positions[i]) for i in indices]
            return values, [0.0] * len(indices)
        index = 0
        while index + 1 < len(times) and times[index + 1] < elapsed:
            index += 1
        span = times[index + 1] - times[index]
        fraction = 0.0 if span <= 0.0 else (elapsed - times[index]) / span
        current = [float(points[index].positions[i]) for i in indices]
        following = [float(points[index + 1].positions[i]) for i in indices]
        values = [
            a + (b - a) * fraction for a, b in zip(current, following)
        ]
        velocities = [
            (b - a) / span if span > 0.0 else 0.0
            for a, b in zip(current, following)
        ]
        return values, velocities

    def _execute(self, side: str, goal_handle):
        trajectory = goal_handle.request.trajectory
        try:
            indices = self._indices(trajectory)
        except ValueError as error:
            self.get_logger().error(f"{side}: {error}")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            return result
        duration = _seconds(trajectory.points[-1].time_from_start)
        with self._lock:
            generation = self._generation[side]
            self._active[side] = goal_handle
        started = time.monotonic()
        result = FollowJointTrajectory.Result()
        try:
            while True:
                elapsed = time.monotonic() - started
                with self._lock:
                    if generation != self._generation[side]:
                        self.get_logger().warning(f"{side}: goal cancelled by reset")
                        goal_handle.canceled()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = "simulation reset"
                        return result
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = (
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )
                    result.error_string = "cancelled"
                    return result
                if elapsed >= duration:
                    values = [
                        float(trajectory.points[-1].positions[index])
                        for index in indices
                    ]
                    with self._lock:
                        self._positions[side] = values
                        self._velocities[side] = [0.0] * len(BASE_JOINTS)
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                values, velocities = self._interpolate(
                    trajectory, indices, elapsed
                )
                with self._lock:
                    self._positions[side] = values
                    self._velocities[side] = velocities
                time.sleep(0.01)
        except Exception as error:  # execution failures must surface to the bridge
            self.get_logger().error(f"{side}: simulation execution failed: {error}")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            return result
        finally:
            with self._lock:
                if self._active.get(side) is goal_handle:
                    self._active[side] = None

    def _reset(self, _request, response):
        with self._lock:
            for side in SIDES:
                self._generation[side] += 1
                self._positions[side] = list(self._initial[side])
                self._velocities[side] = [0.0] * 7
        self.get_logger().info("simulation state reset to the initial pose")
        response.success = True
        response.message = "dual-arm simulation reset"
        return response


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-state-topic", default="/left/joint_states")
    parser.add_argument("--right-state-topic", default="/right/joint_states")
    parser.add_argument(
        "--left-action",
        default="/left/joint_trajectory_controller/follow_joint_trajectory",
    )
    parser.add_argument(
        "--right-action",
        default="/right/joint_trajectory_controller/follow_joint_trajectory",
    )
    parser.add_argument(
        "--initial-positions",
        default="",
        help="comma-separated 14 values in left_joint1..7,right_joint1..7 order",
    )
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--start-tolerance", type=float, default=0.08)
    parser.add_argument("--max-joint-velocity", type=float, default=0.35)
    parser.add_argument("--reset-service", default="/litearm_dual_sim/reset")
    parser.add_argument(
        "--publish-only",
        action="store_true",
        help="only publish state; do not create trajectory action servers",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(remove_ros_args(argv or sys.argv)[1:])
    if args.rate <= 0.0:
        print("dual_arm_sim: --rate must be positive", file=sys.stderr)
        return 2
    try:
        initial = parse_positions(args.initial_positions)
    except ValueError as error:
        print(f"dual_arm_sim: {error}", file=sys.stderr)
        return 2

    rclpy.init()
    node = None
    executor = None
    try:
        node = DualArmSim(args, initial)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
