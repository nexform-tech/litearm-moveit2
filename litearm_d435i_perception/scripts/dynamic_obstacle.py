#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""dynamic_obstacle.py — 在 Gazebo 里驱动一个可移动障碍，制造动态场景。

模型由 launch 里的 spawn_model.py 插入（models/dynamic_obstacle.sdf，
<static>true</static>），本节点只通过 /gazebo/set_entity_state 改它的世界
位姿。静态模型不会被重力拉走、也不会被机械臂推偏，演示可重复。

默认行为
========
  启动后停在 park_pose（工作台外，不挡路）。
  收到 /dynamic_obstacle/enable (std_srvs/SetBool, data=true) 后：
      park -> sweep_start -> 在 sweep_start/sweep_end 之间往复
  data=false 立即停住。
  /dynamic_obstacle/reset (std_srvs/Trigger) 瞬移回 park。

  mode=once 时只从 park 走到 sweep_end，然后停住。验收脚本用 once，
  避免周期运动让“重规划次数”变得不可判定。

动态障碍的“动态”体现在两个层面：
  1. Gazebo 物理实体真的在动（机械臂撞上就是实打实的接触）；
  2. D435i 不断看到它，Octomap 更新，/monitored_planning_scene 跟着变，
     d435i_supervisor 才会对正在执行的轨迹做有效性复检并重规划。
"""

import json
import math
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool, Trigger
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState


class DynamicObstacle(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle")
        self.declare_parameter("entity_name", "dynamic_obstacle")
        self.declare_parameter("mode", "sweep")  # sweep | once
        self.declare_parameter("park_pose", [0.25, -0.55, 0.25])
        self.declare_parameter("sweep_start", [0.25, -0.35, 0.25])
        self.declare_parameter("sweep_end", [0.25, 0.35, 0.25])
        self.declare_parameter("speed", 0.20)  # m/s
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("set_state_service", "/gazebo/set_entity_state")
        self.declare_parameter("pose_topic", "/dynamic_obstacle/pose")
        self.declare_parameter("status_topic", "/dynamic_obstacle/status")

        self.entity = self.get_parameter("entity_name").value
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.park = [float(v) for v in self.get_parameter("park_pose").value]
        self.start = [float(v) for v in self.get_parameter("sweep_start").value]
        self.end = [float(v) for v in self.get_parameter("sweep_end").value]
        self.speed = float(self.get_parameter("speed").value)

        self.enabled = False
        self.spawned = False
        self.direction = 1  # +1: start -> end, -1: end -> start
        self.phase = "to_start"  # 首次 enable 先走到 sweep_start
        self.pose = list(self.park)
        self.last_time = None
        self.pending = False
        self.state = "waiting_gazebo"

        self.set_client = self.create_client(
            SetEntityState, self.get_parameter("set_state_service").value)
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)

        self.create_service(SetBool, "/dynamic_obstacle/enable",
                            self._enable_cb)
        self.create_service(Trigger, "/dynamic_obstacle/reset", self._reset_cb)

        period = 1.0 / max(1.0, float(
            self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(period, self._tick)

    # ── 服务 ──
    def _enable_cb(self, request, response):
        self.enabled = bool(request.data)
        if self.enabled:
            # 从当前实际位置出发，避免假设“一定在 park”。
            self.phase = "to_start"
            self.last_time = self._now()
            self.state = "moving"
        else:
            self.last_time = None
            self.state = "paused"
        response.success = True
        response.message = "enabled" if self.enabled else "paused"
        self.get_logger().info(f"动态障碍 {'启动' if self.enabled else '暂停'}")
        return response

    def _reset_cb(self, _request, response):
        self.enabled = False
        self.phase = "to_start"
        self.last_time = None
        self.pose = list(self.park)
        self._send_pose(self.pose)
        self.state = "parked"
        response.success = True
        response.message = "reset to park"
        return response

    # ── 工具 ──
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _send_pose(self, xyz):
        if not self.set_client.service_is_ready():
            return
        if self.pending:
            return
        state = EntityState()
        state.name = self.entity
        state.reference_frame = "world"
        state.pose = Pose()
        state.pose.position.x = float(xyz[0])
        state.pose.position.y = float(xyz[1])
        state.pose.position.z = float(xyz[2])
        state.pose.orientation.w = 1.0
        self.pending = True
        future = self.set_client.call_async(
            SetEntityState.Request(state=state))
        future.add_done_callback(self._set_done)

    def _set_done(self, future):
        self.pending = False
        if future.exception() is not None:
            self.get_logger().warning(
                f"set_entity_state 失败: {future.exception()}")

    def _step_towards(self, target, distance):
        """把 self.pose 朝 target 前进 distance；返回是否到达。"""
        delta = [target[i] - self.pose[i] for i in range(3)]
        length = math.sqrt(sum(v * v for v in delta))
        if length <= 1e-9 or distance >= length:
            self.pose = list(target)
            return True
        self.pose = [self.pose[i] + delta[i] / length * distance
                     for i in range(3)]
        return False

    def _tick(self):
        now = self._now()

        # Gazebo 起来之前持续把模型摆到 park；服务就绪后发一次即可。
        if not self.spawned:
            if self.set_client.service_is_ready():
                self._send_pose(self.park)
                self.spawned = True
                self.state = "parked"
                self.get_logger().info("动态障碍已就位（park）")
            self._publish(now)
            return

        if self.enabled:
            if self.last_time is None:
                self.last_time = now
            dt = max(0.0, min(0.25, now - self.last_time))
            self.last_time = now
            step = self.speed * dt

            if self.phase == "to_start":
                if self._step_towards(self.start, step):
                    self.phase = "sweeping"
                    self.direction = 1
            elif self.phase == "sweeping":
                target = self.end if self.direction > 0 else self.start
                if self._step_towards(target, step):
                    if self.mode == "once":
                        self.enabled = False
                        self.state = "done"
                    else:
                        self.direction *= -1
            self._send_pose(self.pose)

        self._publish(now)

    def _publish(self, now):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x = float(self.pose[0])
        msg.pose.position.y = float(self.pose[1])
        msg.pose.position.z = float(self.pose[2])
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

        status = String()
        status.data = json.dumps({
            "state": self.state if self.enabled or self.state == "done"
            else ("parked" if self.spawned else "waiting_gazebo"),
            "enabled": self.enabled,
            "mode": self.mode,
            "pose": [round(v, 4) for v in self.pose],
        })
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = DynamicObstacle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
