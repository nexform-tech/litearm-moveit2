#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dual_arm_preflight.py — 真机双臂只读预检。

检查两套 litearm_ros2_control side 实例是否具备被 bridge 接管的条件：

    * /<side>/controller_manager/list_controllers 可访问，
      joint_state_broadcaster 与 joint_trajectory_controller 均为 active；
    * /<side>/joint_states 有 7 个有限关节值、更新频率和年龄在范围内；
    * 每侧 JTC action 存在（--require-actions 时强制要求）；
    * world -> <side>_base_link 的 TF 可查询。

只读，不发轨迹、不打开 USB。输出 JSON，任何一项失败返回非零。

    ros2 run litearm_dual_arm dual_arm_preflight.py
    ros2 run litearm_dual_arm dual_arm_preflight.py --require-actions
    ros2 run litearm_dual_arm dual_arm_preflight.py --output /tmp/preflight.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Mapping, Sequence

import rclpy
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


BASE_JOINTS = tuple(f"joint{index}" for index in range(1, 8))
SIDES = ("left", "right")
EXPECTED_CONTROLLERS = ("joint_state_broadcaster", "joint_trajectory_controller")


def _wait_future(node: Node, future, timeout: float, label: str):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not future.done():
        raise TimeoutError(f"timeout waiting for {label}")
    return future.result()


def _side_error(side: str, error: Exception) -> str:
    text = str(error)
    return text if text.startswith(f"{side}:") else f"{side}: {text}"


class DualArmPreflight(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("litearm_dual_arm_preflight")
        self.args = args
        self._states: dict[str, JointState] = {}
        self._state_stamps: dict[str, float] = {}
        self._state_counts = {side: 0 for side in SIDES}
        self._namespaces = {
            "left": args.left_namespace.rstrip("/"),
            "right": args.right_namespace.rstrip("/"),
        }
        for side in SIDES:
            namespace = self._namespaces[side]
            self.create_subscription(
                JointState,
                f"{namespace}/joint_states",
                lambda message, side=side: self._on_state(side, message),
                20,
            )
        self._controller_clients = {
            side: self.create_client(
                ListControllers,
                f"{self._namespaces[side]}/controller_manager/list_controllers",
            )
            for side in SIDES
        }
        self._action_clients = {
            side: ActionClient(
                self,
                FollowJointTrajectory,
                f"{self._namespaces[side]}/joint_trajectory_controller/"
                "follow_joint_trajectory",
            )
            for side in SIDES
        }
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _on_state(self, side: str, message: JointState) -> None:
        values = {
            name: float(position)
            for name, position in zip(message.name, message.position)
            if name in BASE_JOINTS
        }
        if set(values) != set(BASE_JOINTS):
            return
        if not all(math.isfinite(value) for value in values.values()):
            return
        self._states[side] = message
        self._state_stamps[side] = time.monotonic()
        self._state_counts[side] += 1

    def _wait_state(self, side: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            stamp = self._state_stamps.get(side, 0.0)
            if stamp and time.monotonic() - stamp <= self.args.max_state_age:
                return
        raise TimeoutError(f"{side}: no fresh joint state within {timeout:.1f}s")

    def _check_state(self, side: str) -> dict:
        message = self._states[side]
        age = time.monotonic() - self._state_stamps[side]
        names = set(message.name)
        return {
            "ok": set(BASE_JOINTS) <= names and age <= self.args.max_state_age,
            "joint_count": len(BASE_JOINTS),
            "age_s": round(age, 6),
        }

    def _check_rate(self, side: str) -> float:
        window = max(0.2, float(self.args.rate_window))
        self._state_counts[side] = 0
        deadline = time.monotonic() + window
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self._state_counts[side] / window

    def _check_controllers(self, side: str, timeout: float) -> dict:
        client = self._controller_clients[side]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(
                f"{side}: {self._namespaces[side]}/controller_manager/"
                "list_controllers not available"
            )
        response = _wait_future(
            self,
            client.call_async(ListControllers.Request()),
            timeout,
            f"{side} list_controllers",
        )
        states = {controller.name: controller.state for controller in response.controller}
        return {
            name: states.get(name, "missing") for name in EXPECTED_CONTROLLERS
        }

    def _check_action(self, side: str, timeout: float) -> bool:
        client = self._action_clients[side]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if client.server_is_ready():
                return True
        return False

    def _check_tf(self, side: str, timeout: float) -> dict:
        frame = f"{side}_base_link"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self._tf_buffer.lookup_transform(
                    "world", frame, Time()
                )
            except Exception:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return {
                "ok": True,
                "translation": [
                    round(translation.x, 6),
                    round(translation.y, 6),
                    round(translation.z, 6),
                ],
                "rotation_xyzw": [
                    round(rotation.x, 6),
                    round(rotation.y, 6),
                    round(rotation.z, 6),
                    round(rotation.w, 6),
                ],
            }
        raise TimeoutError(f"world -> {frame} TF not available")

    def run(self) -> dict:
        errors = []
        result = {
            "success": False,
            "require_actions": bool(self.args.require_actions),
            "sides": {},
            "errors": errors,
        }
        for side in SIDES:
            namespace = self._namespaces[side]
            side_result = {"namespace": namespace}
            try:
                self._wait_state(side, float(self.args.timeout))
                side_result["state"] = self._check_state(side)
                if not side_result["state"]["ok"]:
                    errors.append(f"{side}: joint state is not fresh/complete")
                rate = self._check_rate(side)
                side_result["rate_hz"] = round(rate, 3)
                if rate < float(self.args.min_state_rate):
                    errors.append(
                        f"{side}: state rate {rate:.1f} Hz below "
                        f"{self.args.min_state_rate:.1f} Hz"
                    )
                controllers = self._check_controllers(side, float(self.args.timeout))
                side_result["controllers"] = controllers
                for name, state in controllers.items():
                    if state != "active":
                        errors.append(f"{side}: {name} is {state}, not active")
                if self.args.require_actions:
                    ready = self._check_action(side, float(self.args.timeout))
                    side_result["action_ready"] = ready
                    if not ready:
                        errors.append(f"{side}: JTC action is not available")
                side_result["tf"] = self._check_tf(side, float(self.args.timeout))
            except Exception as error:
                message = _side_error(side, error)
                side_result["error"] = message
                errors.append(message)
            result["sides"][side] = side_result
        result["success"] = not errors
        return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-namespace", default="/left")
    parser.add_argument("--right-namespace", default="/right")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-state-age", type=float, default=0.5)
    parser.add_argument("--rate-window", type=float, default=1.0)
    parser.add_argument("--min-state-rate", type=float, default=20.0)
    parser.add_argument(
        "--require-actions",
        action="store_true",
        help="require the two JTC FollowJointTrajectory action servers",
    )
    parser.add_argument(
        "--output", default="/tmp/litearm_dual_arm_preflight.json"
    )
    return parser.parse_args(argv)


def _write(path: str, payload: Mapping) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(remove_ros_args(argv or sys.argv)[1:])
    if args.timeout <= 0.0 or args.max_state_age <= 0.0:
        print("dual_arm_preflight: timeout values must be positive", file=sys.stderr)
        return 2
    rclpy.init()
    node = None
    try:
        node = DualArmPreflight(args)
        payload = node.run()
    except Exception as error:
        payload = {"success": False, "errors": [str(error)], "sides": {}}
        print(f"dual_arm_preflight: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    _write(args.output, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if payload.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
