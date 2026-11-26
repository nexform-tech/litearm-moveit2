#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""d435i_supervisor.py — 相机场景下的规划 / 执行 / 重规划监督。

和 litearm_obstacle_avoidance/avoid_supervisor.py 的区别
=======================================================
旧 supervisor 用「world.collision_objects 变化」作为触发条件；相机链路
的场景变化主要来自 **Octomap**（world.octomap），而且点云每帧都在刷新，
不能一有更新就无脑重规划。本节点因此改成：

    1. 按目标规划；规划结果先对整条轨迹做一次 /check_state_validity，
       通过后才交给 JTC 执行（pre_execution_check）；
    2. 执行中监听 /monitored_planning_scene；Octomap 或碰撞对象有更新时，
       只对**剩余轨迹**做 /check_state_validity 复检（限频、降采样）；
    3. 剩余轨迹真的被挡住 -> 取消当前 JTC goal -> 从当前状态重规划 ->
       继续执行；
    4. “被挡”和“无解”分开记账：一次被挡恢复（取消—重规划—执行）成功后
       预算归零，下一次被挡重新有 max_replans 次机会；只有规划器连续
       max_no_path_retries 次返回空轨迹，才认定真的没有路径，结束重规划；
    5. 恢复预算耗尽 / 无解 -> 低速撤退到 ready（默认关闭），撤退失败或
       未启用则原地 safe_stop。

复检的起点取手臂当前所在的轨迹点，不用“已执行时间”推算：手臂滞后或被
挡时，时间索引会跑到实际位置前面，漏掉马上要走的那段。单次复检和 IK 都
有超时（validity_timeout / ik_guard_timeout），超时按失败处理，避免一个
不返回的 future 把节点永久卡在 check_pending 或 ik 状态。

目标输入
========
    /d435i_supervisor/goal_pose    geometry_msgs/PoseStamped
        base_link 下的 ee_link 目标位姿，先经 /compute_ik 求关节解。
    /d435i_supervisor/goal_joints  sensor_msgs/JointState
        直接给七轴关节目标（探针/脚本用）。

输出
====
    /d435i_supervisor/status                 std_msgs/String (JSON)
    /d435i_supervisor/planned_trajectory     trajectory_msgs/JointTrajectory
    /d435i_supervisor/cancel                 std_srvs/Trigger

安全
====
    * /joint_states 或传感器心跳过期 -> 拒绝新目标；执行中则取消并停住。
      心跳可配置多个 CameraInfo 话题（真机默认同时看深度和彩色流），
      订阅的是 CameraInfo 而不是整份点云，避免每帧反序列化几百 KB 的
      PointCloud2。
    * require_octomap=true 时，在收到第一份非空 Octomap 前拒绝运动；
      配置 max_octomap_age 后，还会拒绝“地图已经很久没更新”的运动。
    * check_current_state=true 时，规划请求显式带上当前关节值；返回后
      比较 trajectory_start 与当前状态。FixStartStateCollision 把起点
      挪到附近无碰撞状态时，直接安全停，避免 JTC 从实际位置去追一个
      被挪走的起点。
    * max_tracking_error>0 时，执行中比较 /joint_states 与最近轨迹点；
      实际跟踪误差超限就安全停，不等 JTC/固件到 0.35 rad 才报错。
    * controller_wait_timeout / execution_timeout_* 防止 action server
      消失或控制器不返回结果时节点永久停在 busy 状态。
    * 这是「停 -> 复检 -> 重规划」，不是 MoveIt Servo 那种毫秒级实时避障。
"""

import json
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningScene, RobotState, WorkspaceParameters)
from moveit_msgs.srv import (GetMotionPlan, GetPositionIK, GetStateValidity)
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]


class D435iSupervisor(Node):
    def __init__(self):
        super().__init__("d435i_supervisor")
        self.declare_parameter("group_name", "litearm_arm")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("velocity_scaling", 0.20)
        self.declare_parameter("acceleration_scaling", 0.20)
        self.declare_parameter("planning_time", 2.0)
        self.declare_parameter("plan_attempts", 5)
        # 两本独立的预算：
        #   max_replans —— “被挡 -> 取消 -> 重规划”这条恢复路径的次数上限。
        #     一次恢复成功（重规划后的轨迹通过执行前复检并开始执行）后会归零
        #     （见 replan_budget_reset_on_success），所以它限制的是连续恢复
        #     失败，而不是整趟任务总共能重规划几次。
        #   max_no_path_retries —— 规划器连续返回空轨迹的次数上限。被挡只说
        #     明当前路径失效，不等于无解；只有连续这么多次规划都找不到路径，
        #     才结束重规划。
        self.declare_parameter("max_replans", 20)
        self.declare_parameter("max_no_path_retries", 3)
        self.declare_parameter("replan_budget_reset_on_success", True)
        self.declare_parameter("goal_tolerance", 0.06)
        self.declare_parameter("cancel_settle", 0.3)
        self.declare_parameter("ik_timeout", 0.5)

        # 数据新鲜度与“地图已建立”门槛
        self.declare_parameter("max_state_age", 1.0)
        # sensor_topic 是旧的单话题接口；sensor_topics 非空时优先生效。
        # 真机同时要求深度和彩色 CameraInfo，避免彩色流活着、深度流停了
        # 而 supervisor 仍以为传感器正常。
        self.declare_parameter("sensor_topic", "/camera/color/camera_info")
        # 默认给一个空字符串，让 rclpy 把类型推断为 STRING_ARRAY；真正的
        # 空列表会被下面的代码过滤掉，然后回退到 sensor_topic。
        self.declare_parameter("sensor_topics", [""])
        self.declare_parameter("max_sensor_age", 1.0)
        # 0 = 不检查 Octomap 新鲜度（默认，兼容仿真）；真机建议 1.5 s。
        self.declare_parameter("max_octomap_age", 0.0)
        self.declare_parameter("require_sensor", True)
        self.declare_parameter("require_octomap", True)

        # 剩余轨迹复检
        self.declare_parameter("validity_check_period", 0.5)
        self.declare_parameter("validity_stride", 3)
        # 单次 /check_state_validity 的最长等待。超时按“轨迹被挡”处理：
        # 不能让一个不返回的 future 把 check_pending 永久卡住，否则之后
        # 整个执行过程都不再复检。
        self.declare_parameter("validity_timeout", 1.0)
        # IK 除了服务内部的 timeout 之外，再加一层看门狗：服务进程卡住时
        # 回调可能永远不回来，节点会一直停在 busy=True 的 ik 状态。
        self.declare_parameter("ik_guard_timeout", 2.0)
        # 执行前把整条轨迹复检一遍。OMPL 规划期间 Octomap 还在更新，
        # 规划开始时的地图与返回时的地图可能已经不同。
        self.declare_parameter("pre_execution_check", True)
        self.declare_parameter("pre_execution_stride", 2)
        # base_link 是固定基座，和台面/安装板天然接触；规划器按规划组检查
        # 时不会把它当成可避让碰撞。复检如果检查全部 link，会每个轨迹点都
        # 判无效，导致取消—重规划—再取消的抖动。默认忽略只涉及 base_link
        # 的接触；涉及其他 link 的接触仍然照常拦截。
        self.declare_parameter("ignore_base_link_contacts", True)

        # 起始状态与执行期看门狗。默认关闭，真机的 real_avoidance launch
        # 会打开；仿真/fake 控制栈保持原来的行为。
        # check_current_state：规划请求显式使用当前关节值；返回后比较
        # trajectory_start，防止 FixStartStateCollision 挪动起点。
        self.declare_parameter("check_current_state", False)
        self.declare_parameter("max_start_deviation", 0.1)
        # 实际跟踪误差：/joint_states 与最近轨迹点的最大关节偏差。
        # 0 = 关闭；真机建议 0.1 rad（JTC/固件 0.35 rad 才报 following error）。
        self.declare_parameter("max_tracking_error", 0.0)
        self.declare_parameter("tracking_check_period", 0.2)
        # 取消后等关节真正停稳再重规划；0 = 不等待（原行为）。
        self.declare_parameter("stop_velocity_tolerance", 0.0)
        self.declare_parameter("stop_wait_timeout", 0.0)
        # action server 不可用 / 控制器不返回结果时的看门狗；0 = 关闭。
        self.declare_parameter("controller_wait_timeout", 0.0)
        self.declare_parameter("execution_timeout_scale", 0.0)
        self.declare_parameter("execution_timeout_margin", 0.0)

        # 失败后的低速撤退
        self.declare_parameter("retreat_enabled", True)
        self.declare_parameter("retreat_positions",
                               [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0])
        self.declare_parameter("retreat_velocity_scaling", 0.1)
        self.declare_parameter("clear_octomap_on_stale", True)

        # 接口
        self.declare_parameter("pose_goal_topic", "/d435i_supervisor/goal_pose")
        self.declare_parameter("joint_goal_topic",
                               "/d435i_supervisor/goal_joints")
        self.declare_parameter("status_topic", "/d435i_supervisor/status")
        self.declare_parameter("trajectory_topic",
                               "/d435i_supervisor/planned_trajectory")

        # ── 状态 ──
        self.state = "idle"
        self.reason = ""
        self.busy = False
        self.replans = 0
        self.no_path_retries = 0
        self.goal = None
        self.goal_source = ""
        self.current = {}
        self.current_velocity = {}
        self.last_joint_time = None
        self.sensor_topics = []
        self.sensor_seen = {}
        self.last_sensor_time = {}
        self.last_cloud_time = None
        self.cloud_seen = False
        self.octomap_seen = False
        self.last_octomap_time = None
        self.scene_dirty = False
        self.scene_updates = 0
        self.last_collision_sig = None
        self.validity_checks = 0
        self.path_invalid = False
        self.current_state_invalid = False
        self.check_queue = []
        self.check_pending = False
        self.current_check_index = None
        self.last_check_time = 0.0
        self.check_deadline = 0.0
        self.trajectory = None
        self.exec_start_time = None
        self.exec_deadline = 0.0
        self.last_tracking_time = 0.0
        self.last_tracking_error = 0.0
        self.pending_trajectory = None
        self.pending_state = "executing"
        self.plan_kind = ""
        self.plan_deadline = 0.0
        self.ik_deadline = 0.0
        self.controller_deadline = 0.0
        self.retreat_reason = ""
        self.cancel_requested = False
        self.result_received = False
        self.replan_due = 0.0
        self.active_handle = None
        self.exec_seq = 0
        self.last_status_time = 0.0
        self.last_log_key = None
        self.last_log_time = 0.0
        self.last_base_ignore_log = 0.0

        # ── 接口 ──
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity")
        self.clear_octomap_client = self.create_client(Empty, "/clear_octomap")
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")

        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10)

        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)
        self.create_subscription(PlanningScene, "/monitored_planning_scene",
                                 self._on_scene, 10)
        sensor_topic = str(self.get_parameter("sensor_topic").value).strip()
        sensor_topics = [
            str(topic).strip()
            for topic in (self.get_parameter("sensor_topics").value or [])
            if str(topic).strip()]
        if not sensor_topics and sensor_topic:
            # 向后兼容旧的 sensor_topic 单话题接口。
            sensor_topics = [sensor_topic]
        self.sensor_topics = sensor_topics
        for topic in sensor_topics:
            # 只用来判断传感器是否还在发；用 CameraInfo（KB 级）而不是
            # PointCloud2（几百 KB、Python 每帧反序列化很贵）。真机同时
            # 订阅深度和彩色，深度流停了不会被彩色流掩盖。
            self.create_subscription(
                CameraInfo, topic,
                lambda msg, name=topic: self._on_sensor(name, msg),
                qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.get_parameter("pose_goal_topic").value,
            self._on_pose_goal, 10)
        self.create_subscription(
            JointState, self.get_parameter("joint_goal_topic").value,
            self._on_joint_goal, 10)
        self.create_service(Trigger, "/d435i_supervisor/cancel",
                            self._cancel_cb)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"d435i_supervisor 就绪: group={self.get_parameter('group_name').value}, "
            f"sensor={', '.join(self.sensor_topics) or '无'}, "
            f"require_octomap={bool(self.get_parameter('require_octomap').value)}, "
            f"max_octomap_age={float(self.get_parameter('max_octomap_age').value):.2f}s")

    # ── 工具 ──
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp_seconds(self, stamp):
        """消息时间戳 -> 秒；零时间戳退回本地接收时间，未来时间夹到现在。"""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self._now()
        value = stamp.sec + stamp.nanosec * 1e-9
        now = self._now()
        return now if value > now else value

    def _status(self, state, reason=""):
        self.state = state
        self.reason = reason
        if state in ("succeeded", "failed", "safe_stop", "retreated"):
            self.busy = False
        self._publish_status()

    def _publish_status(self):
        now = self._now()
        if now - self.last_status_time < 0.1 and self.state not in (
                "succeeded", "failed", "safe_stop", "retreated"):
            return
        self.last_status_time = now
        payload = {
            "state": self.state,
            "busy": self.busy,
            "replans": self.replans,
            "no_path_retries": self.no_path_retries,
            "reason": self.reason,
            "goal_source": self.goal_source,
            "octomap_seen": self.octomap_seen,
            "cloud_seen": self.cloud_seen,
            "scene_updates": self.scene_updates,
            "validity_checks": self.validity_checks,
            "path_invalid": self.path_invalid,
            "tracking_error": round(self.last_tracking_error, 3),
        }
        if self.goal is not None:
            payload["goal"] = [round(v, 3) for v in self.goal]
        if self.last_joint_time is not None:
            payload["joint_age"] = round(now - self.last_joint_time, 3)
        if self.sensor_topics:
            payload["sensors"] = {
                topic: (round(now - self.last_sensor_time[topic], 3)
                        if topic in self.last_sensor_time else None)
                for topic in self.sensor_topics}
        if self.last_cloud_time is not None:
            # 最旧一路传感器的心跳年龄；任一话题停发都会让它变差。
            payload["cloud_age"] = round(now - self.last_cloud_time, 3)
        if self.last_octomap_time is not None:
            payload["octomap_age"] = round(now - self.last_octomap_time, 3)
        self.status_pub.publish(String(data=json.dumps(payload)))

        # 状态 topic 保持 10 Hz（探针要看实时状态），日志只在状态变化或
        # 每 2 s 打一条，否则执行期间会刷屏。
        key = (self.state, self.reason)
        if key != self.last_log_key or now - self.last_log_time >= 2.0:
            self.last_log_key = key
            self.last_log_time = now
            self.get_logger().info(
                f"state={self.state} replans={self.replans} "
                f"reason={self.reason or '-'}")

    def _freshness_error(self):
        now = self._now()
        if not self.current:
            return "no_joint_states"
        missing = [name for name in JOINTS if name not in self.current]
        if missing:
            return "joint_state_incomplete"
        max_age = float(self.get_parameter("max_state_age").value)
        if max_age > 0.0 and now - self.last_joint_time > max_age:
            return "stale_joint_states"

        if self.sensor_topics and bool(
                self.get_parameter("require_sensor").value):
            max_sensor_age = float(
                self.get_parameter("max_sensor_age").value)
            for topic in self.sensor_topics:
                if not self.sensor_seen.get(topic, False):
                    return "no_sensor"
                if max_sensor_age > 0.0 and \
                        now - self.last_sensor_time.get(topic, now) > \
                        max_sensor_age:
                    return "stale_sensor"

        if bool(self.get_parameter("require_octomap").value):
            if not self.octomap_seen:
                return "no_octomap"
            max_octomap_age = float(
                self.get_parameter("max_octomap_age").value)
            if max_octomap_age > 0.0 and self.last_octomap_time is not None \
                    and now - self.last_octomap_time > max_octomap_age:
                return "stale_octomap"
        return ""

    # ── 输入 ──
    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current[name] = position
        # 跟踪误差看门狗和“停稳再重规划”都需要速度；没有 velocity 字段时
        # 相关检查自动退化为“无法判断 -> 不阻塞”。
        if len(msg.velocity) == len(msg.name):
            for name, velocity in zip(msg.name, msg.velocity):
                self.current_velocity[name] = velocity
        else:
            # 没有速度字段的消息不能用旧速度判断“停稳”。
            self.current_velocity.clear()
        self.last_joint_time = self._now()

    def _on_sensor(self, topic, _msg):
        now = self._now()
        self.sensor_seen[topic] = True
        self.last_sensor_time[topic] = now
        if self.sensor_topics:
            self.cloud_seen = all(
                self.sensor_seen.get(name, False)
                for name in self.sensor_topics)
            if self.cloud_seen:
                self.last_cloud_time = min(
                    self.last_sensor_time[name]
                    for name in self.sensor_topics)

    def _on_scene(self, msg):
        octomap = msg.world.octomap.octomap
        if len(octomap.data) > 0:
            self.octomap_seen = True
            # 内层 octomap.header 是地图自己的时间戳，优先用它判断地图
            # 是否在更新；它为零时再退回外层的位姿/场景时间戳。
            stamp = octomap.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = msg.world.octomap.header.stamp
            self.last_octomap_time = self._stamp_seconds(stamp)
            self.scene_updates += 1
            if self.busy and self.state in ("executing", "waiting_controller"):
                self.scene_dirty = True

        # 碰撞对象（scene_builder / 用户手动添加）也纳入变化检测。
        signature = []
        for obj in msg.world.collision_objects:
            pose = obj.primitive_poses[0] if obj.primitive_poses else None
            shape = obj.primitives[0].type if obj.primitives else -1
            dims = tuple(obj.primitives[0].dimensions) if obj.primitives else ()
            signature.append((
                obj.id, obj.header.frame_id, shape, dims,
                None if pose is None else (
                    round(pose.position.x, 4), round(pose.position.y, 4),
                    round(pose.position.z, 4)),
            ))
        signature = tuple(signature)
        if signature != self.last_collision_sig:
            self.last_collision_sig = signature
            self.scene_updates += 1
            if self.busy and self.state in ("executing", "waiting_controller"):
                self.scene_dirty = True

    def _on_pose_goal(self, msg):
        if self.busy:
            self.get_logger().warning("收到位姿目标，但当前仍在执行；忽略")
            return
        error = self._freshness_error()
        if error:
            self.get_logger().error(f"拒绝运动：{error}")
            self._status("failed", error)
            return
        if not self.ik_client.service_is_ready():
            self._status("failed", "ik_service_unavailable")
            return
        self.goal_source = "pose"
        self.busy = True
        self.plan_kind = "ik"
        self.ik_deadline = self._now() + float(
            self.get_parameter("ik_guard_timeout").value)
        self._status("ik", "")
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.get_parameter("group_name").value
        request.ik_request.ik_link_name = self.get_parameter("ee_link").value
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = "base_link"
        pose.pose = msg.pose
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(JOINTS)
        state.joint_state.position = [
            float(self.current.get(name, 0.0)) for name in JOINTS]
        request.ik_request.robot_state = state
        timeout = float(self.get_parameter("ik_timeout").value)
        request.ik_request.timeout = Duration(
            sec=int(timeout), nanosec=int((timeout % 1.0) * 1e9))
        future = self.ik_client.call_async(request)
        future.add_done_callback(self._ik_done)

    def _ik_done(self, future):
        if self.state != "ik":
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - move_group 可能中途退出
            self._status("failed", f"ik_exception:{exc}")
            return
        if response is None or response.error_code.val != 1:
            code = None if response is None else response.error_code.val
            self._status("failed", f"ik_failed:{code}")
            return
        positions = dict(zip(response.solution.joint_state.name,
                             response.solution.joint_state.position))
        missing = [name for name in JOINTS if name not in positions]
        if missing:
            self._status("failed", f"ik_missing_joints:{missing}")
            return
        self.goal = [float(positions[name]) for name in JOINTS]
        self.replans = 0
        self.no_path_retries = 0
        self.scene_dirty = False
        self.cancel_requested = False
        self._start_plan("initial")

    def _on_joint_goal(self, msg):
        if self.busy:
            self.get_logger().warning("收到关节目标，但当前仍在执行；忽略")
            return
        error = self._freshness_error()
        if error:
            self.get_logger().error(f"拒绝运动：{error}")
            self._status("failed", error)
            return
        positions = dict(zip(msg.name, msg.position))
        missing = [name for name in JOINTS if name not in positions]
        if missing:
            self.get_logger().error(f"目标缺少关节 {missing}，忽略")
            return
        if not self.plan_client.service_is_ready():
            self._status("failed", "plan_service_unavailable")
            return
        self.goal_source = "joints"
        self.goal = [float(positions[name]) for name in JOINTS]
        self.replans = 0
        self.no_path_retries = 0
        self.scene_dirty = False
        self.cancel_requested = False
        self.busy = True
        self._start_plan("initial")

    # ── 规划 ──
    def _start_plan(self, kind, retry=False):
        """retry=True 只用于“规划返回空轨迹”后的重试：消耗 no_path 预算，
        不消耗被挡恢复预算。"""
        error = self._freshness_error()
        if error:
            self._safe_stop(error)
            return
        if not self.plan_client.service_is_ready():
            self._safe_stop("plan_service_unavailable")
            return
        if kind == "replan":
            if retry:
                # 地图和起点都没变，规划器仍然找不到路：连续多次才判定无解。
                max_retries = int(
                    self.get_parameter("max_no_path_retries").value)
                if self.no_path_retries >= max_retries:
                    self._fail("no_path")
                    return
                self.no_path_retries += 1
                self.get_logger().warning(
                    f"重规划未找到路径，重试 "
                    f"({self.no_path_retries}/{max_retries})")
            else:
                max_replans = int(self.get_parameter("max_replans").value)
                if self.replans >= max_replans:
                    self._fail("max_replans")
                    return
                self.replans += 1
        self.plan_kind = kind
        self.scene_dirty = False
        self.cancel_requested = False
        self.path_invalid = False
        self.current_state_invalid = False
        self.current_check_index = None
        self.check_queue = []
        self.check_pending = False
        state = {"initial": "planning", "replan": "replanning",
                 "retreat": "retreat_planning"}[kind]
        reason = self.retreat_reason if kind == "retreat" else (
            "no_path_retry" if retry else "")
        self._status(state, reason)
        self.plan_deadline = self._now() + float(
            self.get_parameter("planning_time").value) + 5.0
        scaling = (float(self.get_parameter("retreat_velocity_scaling").value)
                   if kind == "retreat"
                   else float(self.get_parameter("velocity_scaling").value))
        future = self.plan_client.call_async(
            GetMotionPlan.Request(
                motion_plan_request=self._plan_request(scaling)))
        future.add_done_callback(lambda done: self._plan_done(done, kind))

    def _plan_request(self, scaling):
        request = MotionPlanRequest()
        request.group_name = self.get_parameter("group_name").value
        request.pipeline_id = self.get_parameter("pipeline_id").value
        request.planner_id = self.get_parameter("planner_id").value
        request.num_planning_attempts = int(
            self.get_parameter("plan_attempts").value)
        request.allowed_planning_time = float(
            self.get_parameter("planning_time").value)
        request.max_velocity_scaling_factor = scaling
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value)

        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0

        # 显式使用 supervisor 收到的当前关节值，不用 move_group 自己缓存
        # 的状态；否则它可能比 /joint_states 旧。_freshness_error 已经保证
        # 七个关节都在。FixStartStateCollision 仍可能改起点，_plan_done 会
        # 比较 trajectory_start 来发现。
        request.start_state = RobotState()
        request.start_state.is_diff = False
        request.start_state.joint_state.name = list(JOINTS)
        request.start_state.joint_state.position = [
            float(self.current[name]) for name in JOINTS]

        constraints = Constraints()
        for name, value in zip(JOINTS, self.goal):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = value
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)
        return request

    def _start_state_deviation(self, response, trajectory):
        """返回规划起点与 supervisor 当前状态的最大关节差（rad）。

        同时看 trajectory_start 和轨迹第 0 点：适配器可能只改其中一处。
        两处都拿不到完整七轴时返回 None（无法判断，不阻挡执行）。
        """
        candidates = []
        try:
            start = response.motion_plan_response.trajectory_start
            candidates.append(dict(zip(start.joint_state.name,
                                       start.joint_state.position)))
        except Exception:  # noqa: BLE001 - 老版本/异常响应按缺省处理
            pass
        if trajectory is not None and trajectory.points:
            candidates.append(dict(zip(trajectory.joint_names,
                                       trajectory.points[0].positions)))
        deviations = []
        for positions in candidates:
            if all(name in positions for name in JOINTS):
                deviations.append(max(
                    abs(self.current[name] - positions[name])
                    for name in JOINTS))
        return max(deviations) if deviations else None

    def _plan_done(self, future, kind):
        expected = {"initial": "planning", "replan": "replanning",
                    "retreat": "retreat_planning"}[kind]
        if self.state != expected or self.plan_kind != kind:
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - move_group 可能中途退出
            self.get_logger().warning(f"规划服务调用异常: {exc}")
            response = None
        trajectory = None
        if response is not None \
                and response.motion_plan_response.error_code.val == 1:
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
        if trajectory is None or not trajectory.points:
            if kind == "retreat":
                self._safe_stop("retreat_failed")
            elif kind == "replan":
                # 返回空轨迹只说明“这一次没找到”，不立刻等于无解：Octomap
                # 在规划期间还在更新，重试往往能成功。按 no_path 预算重试，
                # 连续 max_no_path_retries 次都为空才结束重规划。
                self._start_plan("replan", retry=True)
            else:
                self._safe_stop("plan_failed")
            return

        # 规划器找到了路径。先确认返回的起点没有被 FixStartStateCollision
        # 等适配器挪走：JTC 从实际当前位置执行，如果轨迹第一点不是实际
        # 状态，它会先跳过去，这一段没有人检查过。
        if bool(self.get_parameter("check_current_state").value):
            deviation = self._start_state_deviation(response, trajectory)
            limit = float(self.get_parameter("max_start_deviation").value)
            if deviation is not None and limit > 0.0 and deviation > limit:
                self.get_logger().error(
                    f"规划起点与实际状态偏差 {deviation:.3f} rad > "
                    f"{limit:.3f}：FixStartStateCollision 可能改过起点，"
                    "拒绝执行")
                self._safe_stop(f"start_state_mismatch:{deviation:.3f}")
                return

        # 规划器找到了路径：连续无路径的计数从此清零。
        self.no_path_retries = 0
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "base_link"
        state_name = "retreating" if kind == "retreat" else "executing"
        if bool(self.get_parameter("pre_execution_check").value):
            self._begin_plan_validation(trajectory, state_name)
        else:
            self._start_execution(trajectory, state_name)

    def _begin_plan_validation(self, trajectory, state_name):
        """执行前把整条轨迹过一遍 /check_state_validity。

        规划请求用的是规划开始那一刻的地图；OMPL 跑几十到几百毫秒，期间
        点云还在写 Octomap，返回的轨迹可能已经过期。先复检再交给 JTC，
        避免跑到一半才发现被挡、再取消重规划。
        """
        self.pending_trajectory = trajectory
        self.pending_state = state_name
        self.trajectory = trajectory
        missing = [name for name in JOINTS if name not in self.current]
        if missing:
            # 用 0.0 补缺失关节会得出错误的复检结果，宁可不执行。
            self._status("failed", f"joint_state_incomplete:{missing}")
            return
        self.path_invalid = False
        self.current_state_invalid = False
        self.current_check_index = None
        self.check_queue = []
        self.check_pending = False
        names = list(trajectory.joint_names)
        stride = max(1, int(self.get_parameter("pre_execution_stride").value))
        indices = list(range(0, len(trajectory.points), stride))
        if indices[-1] != len(trajectory.points) - 1:
            indices.append(len(trajectory.points) - 1)
        for index in indices:
            point = trajectory.points[index]
            by_name = dict(zip(names, point.positions))
            self.check_queue.append((
                index,
                [float(by_name.get(name, self.current.get(name, 0.0)))
                 for name in JOINTS]))
        self.check_pending = True
        self.last_check_time = self._now()
        self._status("validating_plan",
                     self.retreat_reason if state_name == "retreating" else "")
        self._issue_next_validity()

    def _plan_invalidated(self, reason):
        """执行前复检失败：不执行，直接重规划或停住。"""
        if self.pending_state == "retreating":
            self._safe_stop(reason)
            return
        if self.current_state_invalid:
            # 轨迹第 0 点就是当前状态；它自己无效时，重规划只会得到
            # 同样的起点（或被 FixStartStateCollision 挪走），不要循环。
            self.get_logger().error(
                "当前状态本身与 Octomap 碰撞；停止并等待人工处理，不重规划")
            self._safe_stop("current_state_invalid")
            return
        max_replans = int(self.get_parameter("max_replans").value)
        if self.replans >= max_replans:
            self._fail(reason)
        else:
            self.get_logger().warning(f"执行前复检无效，重规划：{reason}")
            self._start_plan("replan")

    # ── 执行 ──
    def _start_execution(self, trajectory, state_name):
        self.pending_trajectory = trajectory
        self.pending_state = state_name
        self.trajectory = trajectory
        if state_name != "retreating" and bool(
                self.get_parameter("replan_budget_reset_on_success").value):
            # 一次被挡恢复到此结束：重规划后的轨迹已经通过执行前复检。把
            # 恢复预算还回去，下一次被挡重新有 max_replans 次机会，避免长
            # 任务里几次成功重规划就把总预算耗光。
            self.replans = 0
        # 只有通过执行前复检（或关闭该复检）的轨迹才会发布到这个话题。
        self.trajectory_pub.publish(trajectory)
        self.exec_start_time = self._now()
        self.last_tracking_time = 0.0
        self.last_tracking_error = 0.0
        self.scene_dirty = False
        self.path_invalid = False
        self.check_queue = []
        self.check_pending = False
        self._status(state_name, self.retreat_reason if state_name == "retreating" else "")
        if not self.action_client.server_is_ready():
            timeout = float(
                self.get_parameter("controller_wait_timeout").value)
            self.controller_deadline = (
                self._now() + timeout if timeout > 0.0 else float("inf"))
            self._status("waiting_controller",
                         self.retreat_reason if state_name == "retreating" else "")
            return
        self._send_trajectory()

    def _execution_deadline(self, trajectory):
        """按轨迹时长生成执行看门狗截止时间；两者都为 0 时禁用。"""
        scale = float(self.get_parameter("execution_timeout_scale").value)
        margin = float(self.get_parameter("execution_timeout_margin").value)
        if scale <= 0.0 and margin <= 0.0:
            return float("inf")
        duration = 0.0
        if trajectory is not None and trajectory.points:
            last = trajectory.points[-1].time_from_start
            duration = last.sec + last.nanosec * 1e-9
        return self._now() + max(0.0, duration) * max(0.0, scale) + \
            max(0.0, margin)

    def _send_trajectory(self):
        self.exec_seq += 1
        seq = self.exec_seq
        self.pending_trajectory.header.stamp = self.get_clock().now().to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.pending_trajectory
        self.result_received = False
        self.cancel_requested = False
        self.exec_start_time = self._now()
        self.exec_deadline = self._execution_deadline(
            self.pending_trajectory)
        try:
            future = self.action_client.send_goal_async(goal)
        except Exception as exc:  # noqa: BLE001 - action server 可能刚掉线
            self.get_logger().error(f"发送轨迹失败: {exc}")
            self._safe_stop(f"send_goal_exception:{exc}")
            return
        future.add_done_callback(lambda done: self._goal_response(done, seq))

    def _goal_response(self, future, seq):
        if seq != self.exec_seq:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - action server 中途退出
            self.get_logger().error(f"发送轨迹的 future 异常: {exc}")
            self._safe_stop(f"goal_response_exception:{exc}")
            return
        if handle is None or not handle.accepted:
            self._fail("controller_rejected_goal")
            return
        if self.cancel_requested:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"取消刚接受的 goal 失败: {exc}")
            return
        self.active_handle = handle
        try:
            handle.get_result_async().add_done_callback(
                lambda done: self._exec_result(done, seq))
        except Exception as exc:  # noqa: BLE001 - action server 中途退出
            self.get_logger().error(f"获取执行结果失败: {exc}")
            self._safe_stop(f"result_handle_exception:{exc}")

    def _exec_result(self, future, seq):
        if seq != self.exec_seq:
            return
        if self.state == "canceling":
            self.result_received = True
            return
        if self.state not in ("executing", "retreating"):
            return
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - action server 中途退出
            self.get_logger().error(f"执行结果的 future 异常: {exc}")
            self._safe_stop(f"exec_result_exception:{exc}")
            return
        if result is None or result.result is None:
            code = -999
        else:
            code = result.result.error_code
        if self.state == "retreating":
            self.active_handle = None
            if code != 0:
                self._safe_stop(f"retreat_aborted:{code}")
                return
            error = max(abs(self.current.get(name, 1e9) - value)
                        for name, value in zip(JOINTS, self.goal))
            if error > float(self.get_parameter("goal_tolerance").value):
                self._safe_stop(f"retreat_not_reached:{error:.3f}")
                return
            self._status("retreated", self.retreat_reason)
            return
        self.active_handle = None
        if code != 0:
            self._fail(f"controller_aborted:{code}")
            return
        error = max(abs(self.current.get(name, 1e9) - value)
                    for name, value in zip(JOINTS, self.goal))
        if error > float(self.get_parameter("goal_tolerance").value):
            self._fail(f"goal_not_reached:{error:.3f}")
            return
        self._status("succeeded")

    # ── 实际跟踪误差 / 停稳判断 ──
    def _closest_waypoint_index(self):
        if self.trajectory is None or not self.current:
            return 0
        best, best_error = 0, float("inf")
        for index, point in enumerate(self.trajectory.points):
            error = 0.0
            for name, position in zip(self.trajectory.joint_names,
                                      point.positions):
                error += abs(self.current.get(name, position) - position)
            if error < best_error:
                best, best_error = index, error
        return best

    def _tracking_error(self):
        """当前实际关节与最近轨迹点的最大单关节偏差（rad）。"""
        if self.trajectory is None or not self.trajectory.points \
                or not self.current:
            return 0.0
        point = self.trajectory.points[self._closest_waypoint_index()]
        by_name = dict(zip(self.trajectory.joint_names, point.positions))
        errors = [abs(self.current.get(name, by_name[name]) - by_name[name])
                  for name in JOINTS if name in by_name]
        return max(errors) if errors else 0.0

    def _maybe_check_tracking(self, now):
        """执行中看实际跟踪误差，不等 JTC/固件到 0.35 rad 才报错。"""
        if self.state not in ("executing", "retreating") or not self.busy:
            return
        max_error = float(self.get_parameter("max_tracking_error").value)
        if max_error <= 0.0:
            return
        period = max(0.05, float(
            self.get_parameter("tracking_check_period").value))
        if now - self.last_tracking_time < period:
            return
        self.last_tracking_time = now
        error = self._tracking_error()
        self.last_tracking_error = error
        if error > max_error:
            self.get_logger().error(
                f"实际跟踪误差 {error:.3f} rad 超过 {max_error:.3f}；安全停")
            self._safe_stop(f"tracking_error:{error:.3f}")

    def _is_stopped(self, tolerance):
        """所有关节速度都在 tolerance 以内；没有速度数据时不阻塞。"""
        if tolerance <= 0.0:
            return True
        missing = [name for name in JOINTS
                   if name not in self.current_velocity]
        if missing:
            return True
        return max(abs(self.current_velocity[name])
                   for name in JOINTS) <= tolerance

    # ── 剩余轨迹复检 ──
    def _maybe_check_path(self, now):
        if self.state != "executing" or not self.busy:
            return
        if not self.scene_dirty or self.check_pending:
            return
        if self.trajectory is None or not self.trajectory.points:
            return
        period = float(self.get_parameter("validity_check_period").value)
        if now - self.last_check_time < period:
            return
        if not self.validity_client.service_is_ready():
            self.scene_dirty = False
            return

        # 以手臂实际所在的轨迹点为起点，而不是按“已执行时间”推算的索引：
        # 手臂滞后或被挡住时，按时间算的索引会跑到实际位置前面，恰好跳过
        # 手臂马上要走的那一段。复检单点约 0.4 ms，整条剩余轨迹全查也只有
        # 十几毫秒，不需要再用 lookahead 缩小范围。
        start_index = self._closest_waypoint_index()
        if start_index >= len(self.trajectory.points):
            self.scene_dirty = False
            return

        stride = max(1, int(self.get_parameter("validity_stride").value))
        indices = list(range(start_index, len(self.trajectory.points), stride))
        if indices[-1] != len(self.trajectory.points) - 1:
            indices.append(len(self.trajectory.points) - 1)
        names = list(self.trajectory.joint_names)
        self.check_queue = []
        for index in indices:
            point = self.trajectory.points[index]
            by_name = dict(zip(names, point.positions))
            self.check_queue.append((
                index,
                [float(by_name.get(name, self.current.get(name, 0.0)))
                 for name in JOINTS]))
        self.check_pending = True
        self.path_invalid = False
        self.scene_dirty = False
        self.last_check_time = now
        self._issue_next_validity()
        self.get_logger().info(
            f"场景更新，复检剩余 {len(self.check_queue)} 个轨迹点")

    def _issue_next_validity(self):
        if not self.check_queue:
            self.check_pending = False
            if self.state == "validating_plan":
                # 执行前复检：通过才交给 JTC；不通过直接重规划，不要先
                # 执行再取消。
                if self.path_invalid:
                    self._plan_invalidated("plan_invalid")
                else:
                    self._start_execution(self.pending_trajectory,
                                          self.pending_state)
                return
            if self.path_invalid:
                if self.replans >= int(self.get_parameter("max_replans").value):
                    self._fail("max_replans")
                else:
                    self._begin_cancel_and_replan("path_blocked")
            return
        if not self.validity_client.service_is_ready():
            self.get_logger().warning("复检服务不可用；按轨迹无效处理")
            self.path_invalid = True
            self.check_queue = []
            self._issue_next_validity()
            return
        index, positions = self.check_queue.pop(0)
        self.current_check_index = index
        self.check_deadline = self._now() + float(
            self.get_parameter("validity_timeout").value)
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = positions[:len(JOINTS)]
        # 和规划器保持一致：只查规划组，而不是=""（全部 link，包含固定
        # 的 base_link）。后者会把基座与台面的安装接触误判为路径被挡。
        request.group_name = str(self.get_parameter("group_name").value)
        future = self.validity_client.call_async(request)
        future.add_done_callback(self._validity_done)

    def _validity_done(self, future):
        # 超时处理或取消可能已经把状态改掉；迟到的回调不要再推进队列。
        if self.state not in ("executing", "validating_plan"):
            return
        self.validity_checks += 1
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - move_group 可能中途退出
            self.get_logger().warning(f"复检调用异常: {exc}")
            response = None
        if response is None:
            self.path_invalid = True
            self.check_queue = []
            self.get_logger().warning(
                f"复检服务无响应: 轨迹点 #{self.current_check_index}")
            self._issue_next_validity()
            return
        if response.valid:
            self._issue_next_validity()
            return

        contacts = list(response.contacts)
        ignore_base = bool(
            self.get_parameter("ignore_base_link_contacts").value)
        relevant = [
            c for c in contacts
            if not (ignore_base and "base_link" in
                    (c.contact_body_1, c.contact_body_2))]
        if ignore_base and contacts and not relevant:
            # 只涉及固定 base_link 的接触：规划器按组检查时不算碰撞，复检
            # 也不能算，否则基座/台面的安装接触会让每个点都“被挡”，造成
            # 反复取消—重规划。继续检查剩余轨迹点。
            now = self._now()
            if now - self.last_base_ignore_log > 10.0:
                self.last_base_ignore_log = now
                self.get_logger().info(
                    "复检忽略 base_link 与场景的固定接触（基座/台面安装面）")
            self._issue_next_validity()
            return

        if self.state == "validating_plan" and self.current_check_index == 0:
            # 轨迹第 0 点就是当前状态；它自己无效时不要进入
            # 取消—重规划—再取消的循环，_plan_invalidated 会安全停。
            self.current_state_invalid = True
        self.path_invalid = True
        # 已经确定被挡，不用再查剩余点；尽快取消/重规划。
        self.check_queue = []
        pairs = [(c.contact_body_1, c.contact_body_2)
                 for c in contacts[:3]]
        self.get_logger().warning(
            f"复检无效: 轨迹点 #{self.current_check_index} "
            f"contacts={pairs}")
        self._issue_next_validity()

    # ── 取消 / 重规划 / 失败 ──
    def _begin_cancel_and_replan(self, reason):
        self._status("canceling", reason)
        self.cancel_requested = True
        self.result_received = False
        self.replan_due = self._now() + float(
            self.get_parameter("cancel_settle").value)
        if self.active_handle is not None:
            try:
                self.active_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - action server 掉线
                self.get_logger().warning(f"取消当前轨迹失败: {exc}")
        else:
            self._start_plan("replan")

    def _fail(self, reason):
        if (bool(self.get_parameter("retreat_enabled").value)
                and self.active_handle is None
                and not self._freshness_error()):
            self.retreat_reason = reason
            self.goal = [float(v) for v in
                         self.get_parameter("retreat_positions").value]
            self.busy = True
            self._start_plan("retreat")
        else:
            self._safe_stop(reason)

    def _clear_octomap_if_stale(self, reason):
        if not reason.startswith("stale_"):
            return
        if not bool(self.get_parameter("clear_octomap_on_stale").value):
            return
        if not self.clear_octomap_client.service_is_ready():
            return
        future = self.clear_octomap_client.call_async(Empty.Request())
        future.add_done_callback(
            lambda _done: self.get_logger().info("数据过期，已清除 Octomap"))

    def _safe_stop(self, reason):
        self.cancel_requested = True
        if self.active_handle is not None:
            try:
                self.active_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - action server 掉线
                self.get_logger().warning(f"安全停取消轨迹失败: {exc}")
            self.active_handle = None
        self.pending_trajectory = None
        self.check_pending = False
        self.check_queue = []
        self._clear_octomap_if_stale(reason)
        self._status("safe_stop", reason)

    def _cancel_cb(self, _request, response):
        self._safe_stop("user_cancel")
        response.success = True
        response.message = "cancelled"
        return response

    # ── 主循环 ──
    def _tick(self):
        now = self._now()

        if self.busy:
            error = self._freshness_error()
            if error:
                self._safe_stop(error)
                return

        # 实际跟踪误差看门狗：执行中/撤退中都看，超限立刻安全停。
        self._maybe_check_tracking(now)

        if self.state == "ik" and now > self.ik_deadline:
            self.get_logger().error("IK 超时，放弃本次目标")
            self._status("failed", "ik_timeout")
            return

        # 复检的未来如果一直不回来，check_pending 会永久为真，之后整个执行
        # 过程都不再复检。超时一律按“轨迹被挡”处理，立即进入取消/重规划。
        if self.check_pending and now > self.check_deadline:
            self.get_logger().warning(
                f"复检超时（轨迹点 #{self.current_check_index}）；按轨迹无效处理")
            self.check_pending = False
            self.check_queue = []
            self.path_invalid = True
            if self.state == "validating_plan":
                self._plan_invalidated("check_timeout")
            elif self.state == "executing":
                if self.replans >= int(self.get_parameter("max_replans").value):
                    self._fail("max_replans")
                else:
                    self._begin_cancel_and_replan("check_timeout")
            return

        if self.state in ("executing", "retreating") \
                and now > self.exec_deadline:
            self.get_logger().error("执行超时，控制器可能没有返回 result；安全停")
            self._safe_stop("execution_timeout")
            return

        if self.state in ("planning", "replanning", "retreat_planning") \
                and now > self.plan_deadline:
            if self.state == "retreat_planning":
                self._safe_stop("retreat_timeout")
            elif self.state == "replanning":
                self._fail("plan_timeout")
            else:
                self._safe_stop("plan_timeout")
            return

        if self.state == "waiting_controller":
            if self.action_client.server_is_ready():
                self._status(self.pending_state,
                             self.retreat_reason
                             if self.pending_state == "retreating" else "")
                self._send_trajectory()
            elif now > self.controller_deadline:
                self.get_logger().error(
                    "等待 joint_trajectory_controller 超时；安全停")
                self._safe_stop("controller_unavailable")
            return

        if self.state == "executing":
            self._maybe_check_path(now)
            # _maybe_check_path 可能在服务不可用等情况下同步把状态改成
            # canceling；重新确认状态，避免对同一次无效再触发一次取消。
            if self.state == "executing" \
                    and self.path_invalid and not self.check_pending:
                if self.replans >= int(self.get_parameter("max_replans").value):
                    self._fail("max_replans")
                else:
                    self._begin_cancel_and_replan("path_blocked")
                return

        if self.state == "canceling":
            if self.result_received or now >= self.replan_due:
                # 等 JTC 真正停稳再重规划；没有速度数据或参数为 0 时
                # _is_stopped 返回 True，保持原来的行为。
                tolerance = float(
                    self.get_parameter("stop_velocity_tolerance").value)
                wait = float(self.get_parameter("stop_wait_timeout").value)
                if wait > 0.0 and now < self.replan_due + wait \
                        and not self._is_stopped(tolerance):
                    self._publish_status()
                    return
                self.active_handle = None
                self.cancel_requested = False
                self.result_received = False
                self._start_plan("replan")

        self._publish_status()


def main():
    rclpy.init()
    node = D435iSupervisor()
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
