#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_arm_bridge.py — 双臂 MoveIt 与两套单臂控制栈之间的执行桥。

数据流
======
    状态（每个控制周期合并）：

        /left/joint_states   (joint1..7)  ─┐
                                          ├─→ /joint_states (left_joint1..7, right_joint1..7)
        /right/joint_states  (joint1..7)  ─┘

    轨迹（MoveIt → bridge → 上游 JTC）：

        /left_arm_controller/follow_joint_trajectory
            → /left/joint_trajectory_controller/follow_joint_trajectory
        /right_arm_controller/follow_joint_trajectory
            → /right/joint_trajectory_controller/follow_joint_trajectory

安全职责
========
* 状态必须新鲜，左右时间差不能超过 ``--max-state-skew``；
* 轨迹起点必须接近当前状态，时间戳从 0 开始且单调；
* 单关节速度不能超过 ``--max-joint-velocity``；
* 任一侧失败、取消或超时，取消另一侧并进入 fail-closed（``/litearm_dual_arm/stop``）；
* ``--state-only`` 只合并状态，不创建 action server，用于只规划入口。

这个节点不负责几何碰撞检查；轨迹在 MoveIt 规划阶段和验收探针的采样复检里
完成。 bridge 负责的是名称映射、时序和失败传播。
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
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


BASE_JOINTS = tuple(f"joint{index}" for index in range(1, 8))
ARMS = {
    "left": tuple(f"left_joint{index}" for index in range(1, 8)),
    "right": tuple(f"right_joint{index}" for index in range(1, 8)),
}
ALL_JOINTS = ARMS["left"] + ARMS["right"]


def _seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class DualArmBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("litearm_dual_arm_bridge")
        self.args = args
        self._lock = threading.RLock()
        self._stop_generation = 0
        self._stopped = False
        self._active: dict[str, object] = {}
        self._arm_positions = {"left": {}, "right": {}}
        self._arm_velocities = {"left": {}, "right": {}}
        self._state_stamps: dict[str, float] = {}

        self._combined = {joint: 0.0 for joint in ALL_JOINTS}
        self._combined_velocities = {joint: 0.0 for joint in ALL_JOINTS}

        self._state_publisher = self.create_publisher(
            JointState, args.state_topic, 20
        )
        for side, topic in (
            ("left", args.left_state_topic),
            ("right", args.right_state_topic),
        ):
            self.create_subscription(
                JointState,
                topic,
                lambda message, side=side: self._on_state(side, message),
                20,
            )

        self._upstream_clients = {}
        self._servers = {}
        self._stop_service = None
        group = ReentrantCallbackGroup()
        if not args.state_only:
            self._upstream_clients = {
                side: ActionClient(
                    self,
                    FollowJointTrajectory,
                    (
                        args.left_upstream_action
                        if side == "left"
                        else args.right_upstream_action
                    ),
                    callback_group=group,
                )
                for side in ARMS
            }
            for side in ARMS:
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
            self._stop_service = self.create_service(
                Trigger,
                args.stop_service,
                self._stop,
                callback_group=group,
            )

        self._timer = self.create_timer(0.02, self._publish_combined)
        if args.state_only:
            self.get_logger().info(
                "state-only bridge active: merged joint states are published, "
                "but no trajectory action servers exist"
            )
        else:
            self.get_logger().info(
                "dual-arm execution bridge active: this process translates and "
                "forwards trajectories to two upstream controllers"
            )
            self.get_logger().info(
                f"upstream actions: {args.left_upstream_action}, "
                f"{args.right_upstream_action}"
            )
        self.get_logger().info(
            f"state input: {args.left_state_topic} + {args.right_state_topic}; "
            f"combined output: {args.state_topic}"
        )

    def _on_state(self, side: str, message: JointState) -> None:
        values = {}
        velocities = {}
        for index, name in enumerate(message.name):
            if name not in BASE_JOINTS or index >= len(message.position):
                continue
            position = float(message.position[index])
            velocity = (
                float(message.velocity[index])
                if index < len(message.velocity)
                else float("nan")
            )
            if math.isfinite(position) and math.isfinite(velocity):
                values[name] = position
                velocities[name] = velocity
        if set(values) != set(BASE_JOINTS):
            return
        with self._lock:
            self._arm_positions[side] = values
            self._arm_velocities[side] = velocities
            self._state_stamps[side] = time.monotonic()
            for local_name in BASE_JOINTS:
                self._combined[f"{side}_{local_name}"] = values[local_name]
                self._combined_velocities[f"{side}_{local_name}"] = velocities[
                    local_name
                ]

    def _state_fresh(self, side: str) -> bool:
        stamp = self._state_stamps.get(side, 0.0)
        return time.monotonic() - stamp <= self.args.max_state_age

    def _publish_combined(self) -> None:
        if not all(self._state_fresh(side) for side in ARMS):
            return
        with self._lock:
            names = [joint for side in ("left", "right") for joint in ARMS[side]]
            positions = [self._combined[name] for name in names]
            velocities = [self._combined_velocities[name] for name in names]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = names
        message.position = positions
        message.velocity = velocities
        message.effort = [0.0 for _ in names]
        self._state_publisher.publish(message)

    def _accept_goal(self, side: str, goal):
        if self._stopped:
            self.get_logger().error(
                f"{side}: reject goal because the bridge is in fail-closed stop; "
                "restart the bridge after checking the cause"
            )
            return GoalResponse.REJECT
        if not all(self._state_fresh(peer) for peer in ARMS):
            self.get_logger().error(
                f"{side}: reject goal because one or both joint states are stale"
            )
            return GoalResponse.REJECT
        stamps = [self._state_stamps[peer] for peer in ARMS]
        skew = max(stamps) - min(stamps)
        if skew > self.args.max_state_skew:
            self.get_logger().error(
                f"{side}: reject goal because left/right state skew "
                f"{skew:.3f}s exceeds {self.args.max_state_skew:.3f}s"
            )
            return GoalResponse.REJECT
        if not self._upstream_clients[side].server_is_ready():
            self.get_logger().error(
                f"{side}: reject goal because upstream controller is absent"
            )
            return GoalResponse.REJECT

        trajectory = goal.trajectory
        if len(trajectory.points) < 2:
            self.get_logger().error(f"{side}: reject trajectory with <2 points")
            return GoalResponse.REJECT
        expected = set(ARMS[side])
        if not expected.issubset(set(trajectory.joint_names)):
            self.get_logger().error(
                f"{side}: trajectory misses required joints"
            )
            return GoalResponse.REJECT
        indices = [trajectory.joint_names.index(joint) for joint in ARMS[side]]
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                self.get_logger().error(f"{side}: trajectory point width mismatch")
                return GoalResponse.REJECT

        with self._lock:
            current = self._arm_positions[side]
        start = [float(trajectory.points[0].positions[index]) for index in indices]
        gap = max(
            abs(value - current[joint.removeprefix(f"{side}_")])
            for value, joint in zip(start, ARMS[side])
        )
        if gap > self.args.start_tolerance:
            self.get_logger().error(
                f"{side}: reject start gap {gap:.4f} rad "
                f"(limit {self.args.start_tolerance:.4f})"
            )
            return GoalResponse.REJECT

        if abs(_seconds(trajectory.points[0].time_from_start)) > 1e-6:
            self.get_logger().error(f"{side}: trajectory must start at t=0")
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
    def _upstream_goal(side: str, trajectory):
        goal = FollowJointTrajectory.Goal()
        target = goal.trajectory
        target.header = trajectory.header
        target.joint_names = list(BASE_JOINTS)
        indices = [trajectory.joint_names.index(joint) for joint in ARMS[side]]
        for source in trajectory.points:
            point = type(source)()
            point.time_from_start = source.time_from_start
            point.positions = [float(source.positions[index]) for index in indices]
            if len(source.velocities) == len(trajectory.joint_names):
                point.velocities = [
                    float(source.velocities[index]) for index in indices
                ]
            else:
                point.velocities = [0.0 for _ in indices]
            if len(source.accelerations) == len(trajectory.joint_names):
                point.accelerations = [
                    float(source.accelerations[index]) for index in indices
                ]
            else:
                point.accelerations = [0.0 for _ in indices]
            target.points.append(point)
        return goal

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _execute(self, side: str, goal_handle):
        with self._lock:
            generation = self._stop_generation
            self._active[side] = goal_handle
        result = FollowJointTrajectory.Result()
        upstream_handle = None
        try:
            goal = self._upstream_goal(side, goal_handle.request.trajectory)
            send_future = self._upstream_clients[side].send_goal_async(goal)
            if not self._wait_future(send_future, 10.0):
                raise TimeoutError("controller goal acceptance timed out")
            upstream_handle = send_future.result()
            if upstream_handle is None or not upstream_handle.accepted:
                raise RuntimeError("upstream controller rejected the goal")
            duration = _seconds(goal.trajectory.points[-1].time_from_start)
            result_future = upstream_handle.get_result_async()
            deadline = time.monotonic() + duration + self.args.result_timeout
            while not result_future.done():
                if (
                    generation != self._stop_generation
                    or goal_handle.is_cancel_requested
                    or time.monotonic() >= deadline
                ):
                    upstream_handle.cancel_goal_async()
                    self._trigger_stop(f"{side}: peer/stop/cancel requested")
                    goal_handle.canceled()
                    result.error_code = (
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )
                    result.error_string = "bridge stop or cancellation"
                    return result
                time.sleep(0.01)
            wrapped = result_future.result()
            if wrapped is None:
                raise RuntimeError("controller returned no result")
            upstream_result = wrapped.result
            result.error_code = upstream_result.error_code
            result.error_string = upstream_result.error_string
            if upstream_result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self._trigger_stop(f"{side}: controller execution failed")
                goal_handle.abort()
                return result
            goal_handle.succeed()
            return result
        except Exception as error:
            self.get_logger().error(f"{side}: bridge execution failed: {error}")
            self._trigger_stop(str(error))
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                if self._active.get(side) is goal_handle:
                    self._active.pop(side, None)

    def _trigger_stop(self, reason: str) -> None:
        with self._lock:
            self._stop_generation += 1
            self._stopped = True
        self.get_logger().error(f"fail-closed stop requested: {reason}")

    def _stop(self, _request, response):
        self._trigger_stop("operator or planner requested stop")
        response.success = True
        response.message = (
            "all bridge trajectory goals cancelled; physical stop depends on the "
            "controller/hardware fault policy and the independent emergency stop"
        )
        return response


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--left-state-topic", default="/left/joint_states")
    parser.add_argument("--right-state-topic", default="/right/joint_states")
    parser.add_argument(
        "--left-action", default="/left_arm_controller/follow_joint_trajectory"
    )
    parser.add_argument(
        "--right-action", default="/right_arm_controller/follow_joint_trajectory"
    )
    parser.add_argument(
        "--left-upstream-action",
        default="/left/joint_trajectory_controller/follow_joint_trajectory",
    )
    parser.add_argument(
        "--right-upstream-action",
        default="/right/joint_trajectory_controller/follow_joint_trajectory",
    )
    parser.add_argument("--stop-service", default="/litearm_dual_arm/stop")
    parser.add_argument("--start-tolerance", type=float, default=0.08)
    parser.add_argument("--max-joint-velocity", type=float, default=0.35)
    parser.add_argument("--max-state-age", type=float, default=0.5)
    parser.add_argument("--max-state-skew", type=float, default=0.1)
    parser.add_argument("--result-timeout", type=float, default=15.0)
    parser.add_argument(
        "--state-only",
        action="store_true",
        help="publish merged joint states without creating action servers",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(remove_ros_args(argv or sys.argv)[1:])
    rclpy.init()
    node = None
    executor = None
    try:
        node = DualArmBridge(args)
        executor = MultiThreadedExecutor(num_threads=6)
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
