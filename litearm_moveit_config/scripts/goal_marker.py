#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""goal_marker.py — RViz 6-DOF 目标标记，把目标交给 d435i_supervisor。

为什么不用 RViz MotionPlanning 的交互 marker
============================================
MotionPlanning 面板的 Execute 直接走 move_group，绕过 supervisor 的剩余轨迹
复检。本节点只负责“选目标”，真正的执行仍然通过：

    /d435i_supervisor/goal_pose   (geometry_msgs/PoseStamped, base_link)

这样 RViz 里同时能看到机器人、相机点云、Octomap 和目标 marker，执行仍然保留
supervisor 的运行时复检 / 重规划。

交互方式
========
    * 启动时把 marker 放在当前 ee_link 位姿（TF 查询）；
    * 在 RViz 里拖动/旋转 marker；拖动时按 preview_rate（默认 10 Hz）
      调 /compute_ik，把 IK 解作为幽灵臂发到 /display_planned_path，
      在 MotionPlanning 面板的 Planned Path 里实时预览目标位姿；
      只预览，不执行、不规划；
    * 右键 marker：
    * 右键 marker 弹出菜单：
        发送目标 (Send goal)   -> 发布到 supervisor
        回零点 (Home)          -> 发布 7 轴全 0 的 joint goal；可用
                                  home_positions 参数改成别的 home 位姿
        预览路径 (Preview)     -> IK + 规划，把轨迹发到 /display_planned_path
                                  只显示，不执行
        打印当前 marker 位姿
        回到当前末端 (Reset)
        取消执行 (Cancel)      -> 调 /d435i_supervisor/cancel

RViz 里需要添加一个 InteractiveMarkers display，Namespace 填 `/goal_marker`。
本包的 rviz/real_avoidance.rviz 已经加好。
"""

import copy
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from moveit_msgs.msg import (Constraints, DisplayTrajectory, JointConstraint,
                             MotionPlanRequest, RobotState, RobotTrajectory)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (InteractiveMarker,
                                     InteractiveMarkerControl,
                                     InteractiveMarkerFeedback, Marker)

JOINTS = [f"joint{i}" for i in range(1, 8)]


def _clamp(value, low, high):
    return max(low, min(high, value))


def _rpy_from_quat(q):
    """四元数 -> ZYX fixed-axis roll/pitch/yaw（与 URDF/相机 TF 约定一致）。"""
    x, y, z, w = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(_clamp(sinp, -1.0, 1.0))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _pose_from_transform(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w
    return pose


def _format_pose(pose):
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    rpy = [math.degrees(v) for v in _rpy_from_quat(q)]
    return (
        f"x={pose.position.x:.4f} y={pose.position.y:.4f} z={pose.position.z:.4f} m; "
        f"rpy=({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}) deg; "
        f"quat=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})")


class GoalMarker(Node):
    def __init__(self):
        super().__init__("goal_marker")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("group_name", "litearm_arm")
        self.declare_parameter("goal_topic", "/d435i_supervisor/goal_pose")
        self.declare_parameter("joint_goal_topic",
                               "/d435i_supervisor/goal_joints")
        self.declare_parameter("home_positions", [0.0] * 7)
        self.declare_parameter("cancel_service", "/d435i_supervisor/cancel")
        self.declare_parameter("marker_scale", 0.22)
        self.declare_parameter("velocity_scaling", 0.08)
        self.declare_parameter("acceleration_scaling", 0.08)
        self.declare_parameter("planning_time", 4.0)
        self.declare_parameter("plan_attempts", 8)
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("pipeline_id", "ompl")
        # 拖动时的实时 IK 幽灵预览：只做 IK，不规划/不执行。
        self.declare_parameter("preview_rate", 10.0)
        self.declare_parameter("preview_avoid_collisions", False)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.joint_goal_topic = str(
            self.get_parameter("joint_goal_topic").value)
        self.home_positions = [
            float(v) for v in self.get_parameter("home_positions").value]
        if len(self.home_positions) != len(JOINTS):
            raise RuntimeError(
                f"home_positions 必须是 {len(JOINTS)} 个关节角，"
                f"实际 {len(self.home_positions)} 个")
        self.cancel_service = str(
            self.get_parameter("cancel_service").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.velocity_scaling = float(
            self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value)
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.plan_attempts = int(self.get_parameter("plan_attempts").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.pipeline_id = str(self.get_parameter("pipeline_id").value)
        self.preview_rate = max(
            1.0, float(self.get_parameter("preview_rate").value))
        self.preview_avoid_collisions = bool(
            self.get_parameter("preview_avoid_collisions").value)

        self.marker_name = "ee_goal"
        self.pose = None
        self.marker = None
        self.current = {}
        self.last_warn = None
        self.preview_dirty = True
        self.drag_ik_pending = False

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.server = InteractiveMarkerServer(self, "goal_marker")
        self.menu = MenuHandler()
        self.menu.insert("发送目标 (Send goal)", callback=self._menu_send)
        self.menu.insert("回零点 (Home)", callback=self._menu_home)
        self.menu.insert("预览路径 (Preview)", callback=self._menu_preview)
        self.menu.insert("打印当前 marker 位姿", callback=self._menu_print)
        self.menu.insert("回到当前末端 (Reset)", callback=self._menu_reset)
        self.menu.insert("取消执行 (Cancel)", callback=self._menu_cancel)

        self.goal_pub = self.create_publisher(
            PoseStamped, self.goal_topic, 10)
        self.joint_goal_pub = self.create_publisher(
            JointState, self.joint_goal_topic, 10)
        self.display_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10)
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik")
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path")
        self.cancel_client = self.create_client(
            Trigger, self.cancel_service)

        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10)
        self.init_timer = self.create_timer(1.0, self._try_initialize)
        self.preview_timer = self.create_timer(
            1.0 / self.preview_rate, self._preview_tick)
        self.get_logger().info(
            "goal_marker 已启动：等待 TF "
            f"{self.base_frame} -> {self.ee_link}，"
            "然后在 RViz 里使用 /goal_marker InteractiveMarkers display")

    # ── 初始化 / 同步 ─────────────────────────────────────────────
    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in JOINTS:
                self.current[name] = float(position)

    def _try_initialize(self):
        if self.marker is not None:
            self.init_timer.cancel()
            return
        try:
            tf_msg = self.buffer.lookup_transform(
                self.base_frame, self.ee_link, Time())
        except Exception as exc:  # noqa: BLE001 - TF 未就绪时继续重试
            now = self.get_clock().now()
            if self.last_warn is None or \
                    (now - self.last_warn).nanoseconds / 1e9 > 5.0:
                self.last_warn = now
                self.get_logger().warning(
                    f"等待 TF {self.base_frame} -> {self.ee_link}: {exc}")
            return

        self.pose = _pose_from_transform(tf_msg.transform)
        self.preview_dirty = True
        self.marker = self._build_marker(self.pose)
        self.server.insert(self.marker, feedback_callback=self._feedback)
        self.menu.apply(self.server, self.marker_name)
        self.server.applyChanges()
        self.get_logger().info(
            f"目标 marker 已放到当前 {self.ee_link}: {_format_pose(self.pose)}")
        self.init_timer.cancel()

    def _build_marker(self, pose):
        marker = InteractiveMarker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.name = self.marker_name
        marker.description = ("拖动方块移动 / 圆环旋转；"
                              "拖动时显示目标幽灵臂；右键菜单发送")
        marker.scale = self.marker_scale
        marker.pose = pose

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = box.scale.z = 0.08
        box.color.r = 0.15
        box.color.g = 0.75
        box.color.b = 1.0
        box.color.a = 0.9

        move = InteractiveMarkerControl()
        move.name = "move_3d"
        move.interaction_mode = InteractiveMarkerControl.MOVE_3D
        move.orientation.w = 1.0
        move.always_visible = True
        move.markers.append(box)
        marker.controls.append(move)

        # 单独加 x/y/z 三个旋转环。只用一个 MOVE_ROTATE_3D 方块时，RViz 里
        # 通常只能拖动移动，旋转手柄不明显或抓不到；三个 ROTATE_AXIS 环
        # 是经典的 6-DOF 交互方式，绕各轴旋转更明确。
        for axis in ("x", "y", "z"):
            rotate = InteractiveMarkerControl()
            rotate.name = f"rotate_{axis}"
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            rotate.orientation.w = 1.0
            setattr(rotate.orientation, axis, 1.0)
            rotate.always_visible = True
            marker.controls.append(rotate)
        return marker

    # ── 交互反馈 ─────────────────────────────────────────────────
    def _feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose = feedback.pose
            self.preview_dirty = True

    def _menu_send(self, _feedback):
        if self.pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose = self.pose
        self.goal_pub.publish(msg)
        self.get_logger().info(f"发送目标到 {self.goal_topic}: "
                               f"{_format_pose(self.pose)}")
        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                f"{self.goal_topic} 没有订阅者；supervisor 是否在运行？")

    def _menu_home(self, _feedback):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in self.home_positions]
        self.joint_goal_pub.publish(msg)
        self.get_logger().info(
            f"发送回零点到 {self.joint_goal_topic}: "
            + ", ".join(f"{name}={value:.4f}"
                        for name, value in zip(JOINTS, msg.position)))
        if self.joint_goal_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                f"{self.joint_goal_topic} 没有订阅者；"
                "supervisor 是否在运行？")

    def _menu_print(self, _feedback):
        if self.pose is not None:
            self.get_logger().info(f"marker 位姿: {_format_pose(self.pose)}")

    # ── 拖动时的实时 IK 幽灵预览 ──────────────────────────────────
    def _preview_tick(self):
        if not self.preview_dirty or self.drag_ik_pending:
            return
        if self.pose is None or len(self.current) < len(JOINTS):
            return
        if not self.ik_client.service_is_ready():
            return
        self.preview_dirty = False
        self.drag_ik_pending = True
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = \
            self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = self.pose
        request.ik_request.avoid_collisions = self.preview_avoid_collisions
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(JOINTS)
        state.joint_state.position = [
            float(self.current[name]) for name in JOINTS]
        request.ik_request.robot_state = state
        request.ik_request.timeout = Duration(sec=1, nanosec=0)
        future = self.ik_client.call_async(request)
        future.add_done_callback(self._drag_ik_done)

    def _drag_ik_done(self, future):
        self.drag_ik_pending = False
        try:
            response = future.result()
        except Exception:  # noqa: BLE001 - move_group 可能中途退出
            return
        if response is None or response.error_code.val != 1:
            return
        positions = dict(zip(response.solution.joint_state.name,
                             response.solution.joint_state.position))
        missing = [name for name in JOINTS if name not in positions]
        if missing:
            return
        self._publish_drag_preview(
            [float(positions[name]) for name in JOINTS])

    def _publish_drag_preview(self, positions):
        """把 IK 解作为单点轨迹发到 /display_planned_path，RViz 的
        MotionPlanning 面板会在 Planned Path 里显示一条幽灵臂。"""
        start = RobotState()
        start.joint_state.name = list(JOINTS)
        start.joint_state.position = [
            float(self.current[name]) for name in JOINTS]

        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = list(JOINTS)
        first = JointTrajectoryPoint()
        first.positions = [float(v) for v in positions]
        first.time_from_start = Duration(sec=0, nanosec=0)
        second = JointTrajectoryPoint()
        second.positions = list(first.positions)
        second.time_from_start = Duration(sec=0, nanosec=100000000)
        joint_trajectory.points = [first, second]

        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = joint_trajectory
        display = DisplayTrajectory()
        display.model_id = "litearm"
        display.trajectory_start = start
        display.trajectory.append(robot_trajectory)
        self.display_pub.publish(display)

    def _menu_reset(self, _feedback):
        try:
            tf_msg = self.buffer.lookup_transform(
                self.base_frame, self.ee_link, Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"无法读取当前末端位姿: {exc}")
            return
        pose = _pose_from_transform(tf_msg.transform)
        self.pose = pose
        self.preview_dirty = True
        self.server.setPose(self.marker_name, pose, tf_msg.header)
        self.server.applyChanges()
        self.get_logger().info(f"marker 回到当前末端: {_format_pose(pose)}")

    def _menu_cancel(self, _feedback):
        if not self.cancel_client.service_is_ready():
            self.get_logger().warning(
                f"{self.cancel_service} 不可用；supervisor 是否在运行？")
            return
        future = self.cancel_client.call_async(Trigger.Request())
        future.add_done_callback(self._cancel_done)

    def _cancel_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"取消服务调用异常: {exc}")
            return
        if response is None:
            self.get_logger().warning("取消服务没有响应")
        else:
            self.get_logger().info(f"取消: {response.message}")

    # ── 预览：IK + 规划，只发轨迹不执行 ────────────────────────────
    def _menu_preview(self, _feedback):
        if self.pose is None:
            return
        if len(self.current) < len(JOINTS):
            self.get_logger().warning(
                "还没有完整的 /joint_states，无法预览；先确认控制栈在运行")
            return
        if not self.ik_client.service_is_ready():
            self.get_logger().warning("/compute_ik 不可用；move_group 是否在运行？")
            return

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = \
            self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = self.pose
        request.ik_request.avoid_collisions = True
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(JOINTS)
        state.joint_state.position = [
            float(self.current[name]) for name in JOINTS]
        request.ik_request.robot_state = state
        request.ik_request.timeout = Duration(sec=1, nanosec=0)

        self.get_logger().info(f"预览: 求 IK {_format_pose(self.pose)}")
        future = self.ik_client.call_async(request)
        future.add_done_callback(self._preview_ik_done)

    def _preview_ik_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - move_group 可能中途退出
            self.get_logger().warning(f"预览失败: IK 服务调用异常 {exc}")
            return
        if response is None or response.error_code.val != 1:
            code = None if response is None else response.error_code.val
            self.get_logger().warning(f"预览失败: IK error_code={code}")
            return
        positions = dict(zip(response.solution.joint_state.name,
                             response.solution.joint_state.position))
        missing = [name for name in JOINTS if name not in positions]
        if missing:
            self.get_logger().warning(f"预览失败: IK 缺少关节 {missing}")
            return
        if not self.plan_client.service_is_ready():
            self.get_logger().warning("/plan_kinematic_path 不可用")
            return
        request = GetMotionPlan.Request(
            motion_plan_request=self._plan_request(positions))
        future = self.plan_client.call_async(request)
        future.add_done_callback(self._preview_plan_done)

    def _plan_request(self, positions):
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.pipeline_id = self.pipeline_id
        request.planner_id = self.planner_id
        request.num_planning_attempts = self.plan_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.workspace_parameters.header.frame_id = self.base_frame
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        request.start_state = RobotState()
        request.start_state.is_diff = True
        constraints = Constraints()
        for name in JOINTS:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(positions[name])
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)
        return request

    def _preview_plan_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"预览失败: 规划服务调用异常 {exc}")
            return
        if response is None or \
                response.motion_plan_response.error_code.val != 1:
            code = None if response is None else \
                response.motion_plan_response.error_code.val
            self.get_logger().warning(f"预览失败: 规划 error_code={code}")
            return
        display = DisplayTrajectory()
        display.model_id = "litearm"
        display.trajectory_start = \
            response.motion_plan_response.trajectory_start
        display.trajectory.append(
            response.motion_plan_response.trajectory)
        self.display_pub.publish(display)
        self.get_logger().info(
            "预览轨迹已发到 /display_planned_path；只显示，不执行。"
            "确认后点 发送目标。")


def main():
    rclpy.init()
    try:
        node = GoalMarker()
    except Exception as exc:  # noqa: BLE001 - 配置错误转成明确的退出码
        print(f"✗ goal_marker 启动失败: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
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
