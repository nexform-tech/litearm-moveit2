#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""wrist_scan.py — 腕装 D435i 多视角扫描、Octomap 融合与覆盖率统计。

它解决什么问题
==============
固定外置相机只有一个视角，物体背面在 Octomap 里是 unknown，而 MoveIt 只
把 occupied 当障碍，于是规划器会从“没看到”的那一侧穿过去。腕装相机可以把
镜头送到多个方位，把同一张 Octomap（frame = base_link）从多视角补全。

本节点怎么动
============
不是连续扫，而是「停 -> 拍 -> 走」：

    1. 在任务体积周围生成一组候选视角（默认顶视 + 55° 环视）；
    2. 对每个视角解 IK，并用 /check_state_validity 验证；
    3. 按关节空间最近邻排序，逐点规划（OMPL）-> 执行（JTC）;
    4. 每个视角停稳 dwell 秒，让 PointCloudOctomapUpdater 把点云融进
       Octomap，同时本节点用 filtered cloud 统计覆盖率；
    5. 覆盖率达到目标或视角跑完 -> 回 ready。

关键约定
========
* 视角是**相机光学帧**的位姿：optical +Z 指向目标，+X 向右，+Y 向下。
  通过 TF ee_link -> camera_depth_optical_frame 反解出 ee_link 目标位姿，
  再交给 /compute_ik。安装位姿只在 wrist_camera.xacro 里维护一份。
* 点云话题复用 /camera/depth/color/points_filtered（移动前已做机器人自
  滤波），sensors_3d.yaml 不需要改。
* 停稳才统计覆盖率；运动中的点云会被跳过，避免拖影污染统计。

服务 / 话题
===========
    /wrist_scan/start    std_srvs/Trigger   开始一轮扫描（auto_start=false 时）
    /wrist_scan/cancel   std_srvs/Trigger   当前视角结束后停止
    /wrist_scan/status   std_msgs/String    JSON 状态与覆盖率

参数全部由 launch 传入；单独运行时可用 --ros-args -p 覆盖。
"""

import json
import math
import re
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint,
                             MotionPlanRequest, PlanningScene,
                             PositionConstraint, RobotState,
                             WorkspaceParameters)
from moveit_msgs.srv import (GetMotionPlan, GetPositionIK, GetStateValidity)
from rclpy.action import ActionClient
from rclpy.duration import Duration as TfDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory

JOINTS = [f"joint{i}" for i in range(1, 8)]
# 与 litearm_moveit_config/config/joint_limits.yaml 一致的粗略边界，只用于
# 生成 IK 种子和快速拒绝；真实限位仍由 MoveIt 检查。
JOINT_MIN = np.array([-2.81, -1.73, -2.81, -3.07, -2.81, -1.55, -1.55])
JOINT_MAX = np.array([2.81, 1.73, 2.81, 0.02, 2.81, 1.55, 1.55])
READY = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0])


# ── 数值工具 ──────────────────────────────────────────────────────────────
def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _quat_to_matrix(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


def _matrix_to_quat(m):
    """旋转矩阵 -> 四元数 (x, y, z, w)，分支写法保证数值稳定。"""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def _transform_to_matrix(tf: TransformStamped):
    matrix = np.eye(4)
    matrix[:3, :3] = _quat_to_matrix(tf.transform.rotation)
    matrix[:3, 3] = [tf.transform.translation.x,
                     tf.transform.translation.y,
                     tf.transform.translation.z]
    return matrix


def _pose_to_matrix(pose: Pose):
    matrix = np.eye(4)
    matrix[:3, :3] = _quat_to_matrix(pose.orientation)
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def _matrix_to_pose(matrix):
    pose = Pose()
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    qx, qy, qz, qw = _matrix_to_quat(matrix[:3, :3])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def _look_at_optical(position, target):
    """生成光学帧位姿：+Z 指向 target，+X 向右，+Y 向下（REP 103）。"""
    z = np.asarray(target, dtype=np.float64) - np.asarray(position, dtype=np.float64)
    norm = np.linalg.norm(z)
    if norm < 1e-9:
        raise ValueError("camera position coincides with target")
    z /= norm
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, up))) > 0.999:
        up = np.array([1.0, 0.0, 0.0])
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    rotation = np.column_stack((x, y, z))
    return _matrix_to_pose(np.block([
        [rotation, np.asarray(position, dtype=np.float64).reshape(3, 1)],
        [np.zeros((1, 3)), np.ones((1, 1))],
    ]))


def _parse_floats(text, fallback):
    try:
        values = [float(v) for v in re.split(r"[,\s]+", str(text).strip()) if v]
    except ValueError:
        return list(fallback)
    return values or list(fallback)


def _parse_points(text):
    points = []
    for chunk in str(text).replace(";", "|").split("|"):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = _parse_floats(chunk, [])
        if len(values) == 3:
            points.append(np.array(values, dtype=np.float64))
    return points


class WristScan(Node):
    def __init__(self):
        super().__init__("wrist_scan")

        # ── 接口/几何 ──
        self.declare_parameter("group_name", "litearm_arm")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")

        # ── 扫描图案 ──
        self.declare_parameter("targets", "0.10,0.0,0.10")
        self.declare_parameter("radius", 0.40)
        self.declare_parameter("elevations", "90,55")
        self.declare_parameter("azimuths", 8)
        self.declare_parameter("low_elevation_threshold", 45.0)
        self.declare_parameter("low_max_azimuth", 90.0)
        self.declare_parameter("max_views", 0)

        # ── 运动 / 采集 ──
        self.declare_parameter("velocity_scaling", 0.20)
        self.declare_parameter("acceleration_scaling", 0.20)
        self.declare_parameter("planning_time", 6.0)
        self.declare_parameter("plan_attempts", 12)
        self.declare_parameter("ik_timeout", 0.30)
        self.declare_parameter("ik_seeds", 7)
        self.declare_parameter("dwell", 0.60)
        self.declare_parameter("return_home", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("top_first", True)
        self.declare_parameter("safe_transfer", True)
        self.declare_parameter("safe_transfer_z", 0.30)
        self.declare_parameter("approach_height", 0.08)
        self.declare_parameter("require_scene_objects", False)
        self.declare_parameter("runtime_check", True)
        self.declare_parameter("check_period", 0.50)
        self.declare_parameter("check_lookahead", 0.40)
        self.declare_parameter("check_stride", 2)
        self.declare_parameter("max_exec_replans", 3)
        self.declare_parameter("cancel_settle", 0.50)

        # ── 地图 / 覆盖率 ──
        self.declare_parameter("clear_octomap", False)
        self.declare_parameter("clear_wait", 1.0)
        self.declare_parameter("require_octomap", True)
        self.declare_parameter("coverage_target", 0.90)
        self.declare_parameter("coverage_resolution", 0.05)
        self.declare_parameter("coverage_period", 0.25)
        self.declare_parameter("coverage_stride", 5)
        self.declare_parameter("coverage_rays", 800)
        self.declare_parameter("roi_min", "-0.40,-0.50,-0.05")
        self.declare_parameter("roi_max", "0.60,0.50,0.45")
        self.declare_parameter("cloud_topic",
                               "/camera/depth/color/points_filtered")
        self.declare_parameter("sensor_topic", "/camera/color/camera_info")

        # ── 启动 ──
        self.declare_parameter("auto_start", True)
        self.declare_parameter("start_delay", 8.0)
        self.declare_parameter("startup_timeout", 90.0)
        self.declare_parameter("status_topic", "/wrist_scan/status")

        self.targets = _parse_points(self.get_parameter("targets").value)
        if not self.targets:
            self.targets = [np.array([0.10, 0.0, 0.10])]
        self.radius = float(self.get_parameter("radius").value)
        self.elevations = _parse_floats(
            self.get_parameter("elevations").value, [90.0, 55.0])
        self.azimuths = max(1, int(self.get_parameter("azimuths").value))
        self.low_elevation_threshold = float(
            self.get_parameter("low_elevation_threshold").value)
        self.low_max_azimuth = float(
            self.get_parameter("low_max_azimuth").value)
        self.max_views = max(0, int(self.get_parameter("max_views").value))

        self.group_name = self.get_parameter("group_name").value
        self.ee_link = self.get_parameter("ee_link").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.velocity_scaling = float(
            self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value)
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.plan_attempts = int(self.get_parameter("plan_attempts").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.ik_seeds = max(1, int(self.get_parameter("ik_seeds").value))
        self.dwell = max(0.0, float(self.get_parameter("dwell").value))
        self.return_home = bool(self.get_parameter("return_home").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.top_first = bool(self.get_parameter("top_first").value)
        self.safe_transfer = bool(self.get_parameter("safe_transfer").value)
        self.safe_transfer_z = float(
            self.get_parameter("safe_transfer_z").value)
        self.approach_height = max(0.0, float(
            self.get_parameter("approach_height").value))
        self.require_scene_objects = bool(
            self.get_parameter("require_scene_objects").value)
        self.runtime_check = bool(
            self.get_parameter("runtime_check").value)
        self.check_period = max(0.0, float(
            self.get_parameter("check_period").value))
        self.check_lookahead = max(0.0, float(
            self.get_parameter("check_lookahead").value))
        self.check_stride = max(1, int(
            self.get_parameter("check_stride").value))
        self.max_exec_replans = max(0, int(
            self.get_parameter("max_exec_replans").value))
        self.cancel_settle = max(0.0, float(
            self.get_parameter("cancel_settle").value))

        self.clear_octomap = bool(self.get_parameter("clear_octomap").value)
        self.clear_wait = float(self.get_parameter("clear_wait").value)
        self.require_octomap = bool(self.get_parameter("require_octomap").value)
        self.coverage_target = float(
            self.get_parameter("coverage_target").value)
        self.coverage_period = float(
            self.get_parameter("coverage_period").value)
        self.coverage_stride = max(1, int(
            self.get_parameter("coverage_stride").value))
        self.coverage_rays = max(50, int(
            self.get_parameter("coverage_rays").value))
        self.roi_min = np.array(_parse_floats(
            self.get_parameter("roi_min").value, [-0.40, -0.50, -0.05]))
        self.roi_max = np.array(_parse_floats(
            self.get_parameter("roi_max").value, [0.60, 0.50, 0.45]))
        self.voxel = max(0.01, float(
            self.get_parameter("coverage_resolution").value))
        self.cloud_topic = self.get_parameter("cloud_topic").value

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.startup_timeout = float(
            self.get_parameter("startup_timeout").value)

        # ── 状态 ──
        self.current = {}
        self.last_joint_time = None
        self.sensor_seen = False
        self.last_sensor_time = None
        self.octomap_seen = False
        self.scene_objects_seen = False
        self.executing = False
        self.scene_dirty = False
        self.exec_trajectory = None
        self.exec_start = 0.0
        self.last_validity_check = 0.0
        self.active_handle = None
        self.runtime_replans = 0
        self.validity_checks = 0
        self.last_solution = None
        self.start_requested = False
        self.cancel_requested = False
        self.scan_state = "idle"
        self.reason = ""
        self.current_view = ""
        self.views_total = 0
        self.views_done = 0
        self.views_failed = 0
        self.coverage = 0.0
        self.cloud_frames = 0
        self.capture_ready = False
        self.cloud_busy = False
        self.last_coverage_time = 0.0
        self.observed = set()
        self.occupied = set()
        self.total_voxels = 1
        self._seed = 20261125

        # ── 覆盖率网格 ──
        self._init_coverage()

        # ── 服务/动作 ──
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_client = self.create_client(GetMotionPlan,
                                              "/plan_kinematic_path")
        self.validity_client = self.create_client(GetStateValidity,
                                                  "/check_state_validity")
        self.clear_client = self.create_client(Empty, "/clear_octomap")
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_ee_camera = None

        # ── 订阅/发布 ──
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)
        self.create_subscription(PlanningScene, "/monitored_planning_scene",
                                 self._on_scene, 10)
        sensor_topic = self.get_parameter("sensor_topic").value
        self.create_subscription(CameraInfo, sensor_topic,
                                 self._on_sensor, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic,
                                 self._on_cloud, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)
        # 当前执行/将要执行的轨迹；验收脚本用它把障碍注入到轨迹中段
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/wrist_scan/planned_trajectory", 10)
        self.create_service(Trigger, "/wrist_scan/start", self._start_cb)
        self.create_service(Trigger, "/wrist_scan/cancel", self._cancel_cb)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f"wrist_scan 就绪: targets={[t.tolist() for t in self.targets]} "
            f"radius={self.radius:.2f} elevations={self.elevations} "
            f"azimuths={self.azimuths} cloud={self.cloud_topic}")

    # ── 时间 / 回调 ──
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current[name] = position
        self.last_joint_time = self._now()

    def _on_sensor(self, _msg):
        self.sensor_seen = True
        self.last_sensor_time = self._now()

    def _on_scene(self, msg):
        if len(msg.world.octomap.octomap.data) > 0:
            self.octomap_seen = True
        if len(msg.world.collision_objects) > 0:
            self.scene_objects_seen = True
        if self.executing:
            # 执行期间地图/碰撞对象更新 -> 触发剩余轨迹复检（带限频）
            self.scene_dirty = True

    def _current_positions(self):
        return [float(self.current.get(name, 0.0)) for name in JOINTS]

    def _start_cb(self, _request, response):
        if self.scan_state == "scanning":
            response.success = False
            response.message = "scan already running"
            return response
        self.start_requested = True
        response.success = True
        response.message = "scan queued"
        return response

    def _cancel_cb(self, _request, response):
        self.cancel_requested = True
        response.success = True
        response.message = "cancel requested"
        return response

    # ── 服务调用 ──
    def _call(self, client, request, timeout):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001 - 服务端异常不应中断扫描
            self.get_logger().warning(f"服务调用异常: {exc}")
            return None

    def _sleep(self, seconds):
        deadline = self._now() + max(0.0, seconds)
        while rclpy.ok() and self._now() < deadline:
            rclpy.spin_once(
                self, timeout_sec=max(0.0, min(0.05, deadline - self._now())))

    def _tf_matrix(self, target, source, stamp=None):
        if self.T_ee_camera is not None \
                and target == self.ee_link and source == self.camera_frame:
            return self.T_ee_camera
        try:
            when = Time.from_msg(stamp) if stamp is not None else Time()
            tf = self.tf_buffer.lookup_transform(
                target, source, when, TfDuration(seconds=0.2))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target, source, Time(), TfDuration(seconds=0.2))
            except Exception:
                return None
        matrix = _transform_to_matrix(tf)
        if target == self.ee_link and source == self.camera_frame:
            self.T_ee_camera = matrix
        return matrix

    # ── 启动门槛 ──
    def _services_ready(self):
        return (self.ik_client.service_is_ready()
                and self.plan_client.service_is_ready()
                and self.validity_client.service_is_ready()
                and self.action_client.server_is_ready())

    def _sensor_ready(self):
        if not self.sensor_seen or self.last_sensor_time is None:
            return False
        return self._now() - self.last_sensor_time < 5.0

    def wait_for_stack(self):
        deadline = self._now() + self.startup_timeout
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = (self._services_ready() and bool(self.current)
                     and self._sensor_ready())
            if ready and (not self.require_octomap or self.octomap_seen) \
                    and (not self.require_scene_objects
                         or self.scene_objects_seen):
                return True
            now = self._now()
            if now - last_log > 5.0:
                last_log = now
                self.get_logger().info(
                    "等待扫描栈: services="
                    f"{self._services_ready()} joints={bool(self.current)} "
                    f"sensor={self._sensor_ready()} "
                    f"octomap={self.octomap_seen} "
                    f"scene_objects={self.scene_objects_seen}")
            if now > deadline:
                self.get_logger().error("等待扫描栈超时，扫描未启动")
                return False
        return False

    # ── 覆盖率网格 ──
    def _init_coverage(self):
        self.roi_min = np.minimum(self.roi_min, self.roi_max - self.voxel)
        self.dims = np.maximum(
            1, np.ceil((self.roi_max - self.roi_min) / self.voxel)
        ).astype(int)
        self.total_voxels = int(np.prod(self.dims))
        self.observed = set()
        self.occupied = set()
        self.coverage = 0.0

    def _voxel_indices(self, points):
        if points.size == 0:
            return np.zeros((0, 3), dtype=int)
        indices = np.floor((points - self.roi_min) / self.voxel).astype(int)
        inside = np.all((indices >= 0) & (indices < self.dims), axis=1)
        return indices[inside]

    def _update_coverage(self, points_world, origin_world):
        if points_world.size == 0:
            return
        points = points_world[:: self.coverage_stride]
        occupied = self._voxel_indices(points)
        self.occupied.update(map(tuple, occupied))

        if len(points) > self.coverage_rays:
            selection = np.linspace(0, len(points) - 1,
                                    self.coverage_rays).astype(int)
            ray_targets = points[selection]
        else:
            ray_targets = points
        if len(ray_targets) == 0:
            return

        steps = 12
        fractions = np.linspace(0.0, 1.0, steps)[None, :, None]
        samples = (origin_world[None, None, :]
                   + fractions * (ray_targets[:, None, :]
                                  - origin_world[None, None, :]))
        observed = self._voxel_indices(samples.reshape(-1, 3))
        self.observed.update(map(tuple, observed))
        self.coverage = min(1.0, len(self.observed) / float(self.total_voxels))

    def _on_cloud(self, msg):
        if not self.capture_ready or self.cloud_busy:
            return
        now = self._now()
        if now - self.last_coverage_time < self.coverage_period:
            return
        self.cloud_busy = True
        try:
            stamp = Time.from_msg(msg.header.stamp) \
                if (msg.header.stamp.sec or msg.header.stamp.nanosec) else None
            transform = self._tf_matrix(
                "base_link", msg.header.frame_id or self.camera_frame, stamp)
            if transform is None:
                return
            try:
                points = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"点云解析失败: {exc}")
                return
            points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
            finite = np.all(np.isfinite(points), axis=1)
            points = points[finite]
            if points.size == 0:
                return
            points_world = points @ transform[:3, :3].T + transform[:3, 3]
            self._update_coverage(points_world, transform[:3, 3])
            self.cloud_frames += 1
            # 只有真正处理成功才进入下一个节流窗口；TF/解析失败时让下一帧
            # 立即重试，否则停稳 0.6 s 的窗口可能一帧都统计不到。
            self.last_coverage_time = now
        finally:
            self.cloud_busy = False

    # ── IK / 规划 / 执行 ──
    def _state_valid(self, positions):
        request = GetStateValidity.Request()
        request.group_name = ""  # 整机检查，相机 link 也会被覆盖
        request.robot_state = RobotState()
        request.robot_state.is_diff = False
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = [
            float(v) for v in positions]
        response = self._call(self.validity_client, request, timeout=5.0)
        return bool(response is not None and response.valid)

    def _ik_seeds(self):
        current = np.array(self._current_positions())
        seeds = [current]
        if self.last_solution is not None:
            seeds.append(np.array(self.last_solution))
        seeds.append(READY.copy())
        seeds.append(np.zeros(7))
        seeds.append((JOINT_MIN + JOINT_MAX) * 0.5)
        rng = np.random.default_rng(self._seed)
        while len(seeds) < self.ik_seeds:
            seeds.append(np.clip(current + rng.normal(0.0, 0.7, 7),
                                 JOINT_MIN, JOINT_MAX))
        return seeds[: max(self.ik_seeds, 1)]

    def _solve_ik(self, pose):
        for seed in self._ik_seeds():
            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name = self.group_name
            ik.ik_link_name = self.ee_link
            ik.avoid_collisions = True
            ik.timeout = Duration(sec=0, nanosec=int(self.ik_timeout * 1e9))
            stamped = PoseStamped()
            stamped.header.frame_id = "base_link"
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.pose = pose
            ik.pose_stamped = stamped
            ik.robot_state = RobotState()
            ik.robot_state.is_diff = True
            ik.robot_state.joint_state.name = list(JOINTS)
            ik.robot_state.joint_state.position = [
                float(v) for v in seed]
            response = self._call(self.ik_client, request,
                                  timeout=self.ik_timeout + 2.0)
            if response is None or response.error_code.val != 1:
                continue
            by_name = dict(zip(response.solution.joint_state.name,
                               response.solution.joint_state.position))
            if any(name not in by_name for name in JOINTS):
                continue
            solution = np.array([float(by_name[name]) for name in JOINTS])
            if np.any(solution < JOINT_MIN - 1e-6) \
                    or np.any(solution > JOINT_MAX + 1e-6):
                continue
            if not self._state_valid(solution):
                continue
            self.last_solution = solution
            return solution
        return None

    def _solve_view(self, view):
        if self.T_ee_camera is None:
            self.T_ee_camera = self._tf_matrix(self.ee_link, self.camera_frame)
        if self.T_ee_camera is None:
            self.get_logger().error(
                f"找不到 TF {self.ee_link} -> {self.camera_frame}")
            return None, None
        target = view["target"]
        # 视角不可达时，沿同一方向稍微收/放，给 KDL 多一次机会
        for scale in (1.0, 0.92, 1.08, 0.85, 1.15):
            position = target + view["direction"] * (self.radius * scale)
            optical = _look_at_optical(position, target)
            base_optical = _pose_to_matrix(optical)
            base_ee = base_optical @ np.linalg.inv(self.T_ee_camera)
            solution = self._solve_ik(_matrix_to_pose(base_ee))
            if solution is not None:
                return solution, position
        return None, None

    def _solve_preview(self, view):
        """在已求解的视角正上方求预视角，保证预视角确实更高。

        不能再用 _solve_view 的 scale 搜索：换 scale 会改变仰角高度，
        可能把“预视角”算到比目标视角更低的位置。
        """
        if self.T_ee_camera is None:
            self.T_ee_camera = self._tf_matrix(self.ee_link, self.camera_frame)
        if self.T_ee_camera is None:
            return None, None
        target = view["target"]
        base = np.asarray(view["position"], dtype=np.float64)
        for height in (self.approach_height,
                       self.approach_height * 1.5,
                       self.approach_height * 0.5):
            position = base + np.array([0.0, 0.0, height])
            optical = _look_at_optical(position, target)
            base_optical = _pose_to_matrix(optical)
            base_ee = base_optical @ np.linalg.inv(self.T_ee_camera)
            solution = self._solve_ik(_matrix_to_pose(base_ee))
            if solution is not None:
                return solution, position
        return None, None

    def _plan(self, goal, path_z_min=None):
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.pipeline_id = self.get_parameter("pipeline_id").value
        request.planner_id = self.get_parameter("planner_id").value
        request.num_planning_attempts = self.plan_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling

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
        for name, value in zip(JOINTS, goal):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)

        if path_z_min is not None:
            # 跨工位转移时把 ee_link 限制在 z >= path_z_min 的盒子里，
            # 避免为了走近路而贴着桌面/物体横向扫过去。终点（预视角）
            # 和起点都必须在这个盒子里；调用方负责先升到安全高度。
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            lower = float(path_z_min)
            upper = 1.0
            box.dimensions = [2.0, 2.0, upper - lower]
            region = BoundingVolume()
            region.primitives.append(box)
            region_pose = Pose()
            region_pose.position.z = 0.5 * (lower + upper)
            region_pose.orientation.w = 1.0
            region.primitive_poses.append(region_pose)
            constraint = PositionConstraint()
            constraint.header.frame_id = "base_link"
            constraint.link_name = self.ee_link
            constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            constraint.constraint_region = region
            constraint.weight = 1.0
            request.path_constraints.position_constraints.append(constraint)

        response = self._call(
            self.plan_client,
            GetMotionPlan.Request(motion_plan_request=request),
            timeout=self.planning_time + 10.0)
        if response is None:
            return None
        plan = response.motion_plan_response
        if plan.error_code.val != 1:
            self.get_logger().warning(
                f"规划失败 code={plan.error_code.val}: {self.current_view}")
            return None
        trajectory = plan.trajectory.joint_trajectory
        return trajectory if trajectory.points else None

    def _remaining_path_invalid(self, trajectory, exec_start):
        """复检剩余轨迹：从“当前时间 + lookahead”之后采样状态做碰撞检查。

        返回 True 表示剩余路径已被新障碍挡住，需要取消并重规划。
        """
        if trajectory is None or not trajectory.points:
            return False
        elapsed = max(0.0, self._now() - exec_start)
        start = len(trajectory.points) - 1
        for index, point in enumerate(trajectory.points):
            stamp = (point.time_from_start.sec
                     + point.time_from_start.nanosec * 1e-9)
            if stamp >= elapsed + self.check_lookahead:
                start = index
                break
        indices = list(range(start, len(trajectory.points),
                             self.check_stride))
        if indices[-1] != len(trajectory.points) - 1:
            indices.append(len(trajectory.points) - 1)

        # 限采样次数，但必须包含末尾目标点；否则靠近终点的障碍可能被漏掉。
        max_checks = 12
        if len(indices) > max_checks:
            picks = np.linspace(0, len(indices) - 1,
                                max_checks).astype(int)
            indices = [indices[i] for i in picks]

        names = list(trajectory.joint_names)
        for index in indices:
            point = trajectory.points[index]
            by_name = dict(zip(names, point.positions))
            positions = [float(by_name.get(name, self.current.get(name, 0.0)))
                         for name in JOINTS]
            self.validity_checks += 1
            if not self._state_valid(positions):
                self.get_logger().warning(
                    "运行时复检无效: 轨迹点 "
                    f"#{index}/{len(trajectory.points) - 1}")
                return True
        return False

    def _execute(self, trajectory):
        """执行轨迹；返回 'ok' / 'blocked' / 'failed'。

        'blocked' 表示执行期间地图更新并复检发现剩余轨迹被挡住；调用方应
        从当前状态重新规划。这里先取消 JTC goal 并等待其停止。
        """
        if not self.action_client.server_is_ready():
            self.get_logger().error("JTC action server 未就绪")
            return "failed"
        trajectory.header.stamp = self.get_clock().now().to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            return "failed"
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning("控制器拒绝了轨迹")
            return "failed"

        last = trajectory.points[-1].time_from_start
        duration = last.sec + last.nanosec * 1e-9
        result_future = handle.get_result_async()

        self.active_handle = handle
        self.exec_trajectory = trajectory
        self.exec_start = self._now()
        self.executing = True
        self.scene_dirty = False
        self.last_validity_check = 0.0
        deadline = self._now() + duration + 15.0
        blocked = False
        try:
            while rclpy.ok() and not result_future.done() \
                    and self._now() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.runtime_check or not self.scene_dirty:
                    continue
                now = self._now()
                if now - self.last_validity_check < self.check_period:
                    continue
                self.scene_dirty = False
                self.last_validity_check = now
                if self._remaining_path_invalid(trajectory, self.exec_start):
                    blocked = True
                    self.get_logger().warning(
                        "剩余轨迹被新障碍挡住，取消当前目标并重规划")
                    handle.cancel_goal_async()
                    cancel_deadline = (self._now() + self.cancel_settle
                                       + 2.0)
                    while rclpy.ok() and not result_future.done() \
                            and self._now() < cancel_deadline:
                        rclpy.spin_once(self, timeout_sec=0.05)
                    break
        finally:
            self.executing = False
            self.active_handle = None
            self.exec_trajectory = None
            self.scene_dirty = False

        if blocked:
            # 给控制器一点时间真正停住，再让调用方用当前关节状态重规划
            self._sleep(self.cancel_settle)
            return "blocked"
        if not result_future.done():
            self.get_logger().warning("执行超时，取消当前目标")
            handle.cancel_goal_async()
            cancel_deadline = self._now() + self.cancel_settle + 2.0
            while rclpy.ok() and not result_future.done() \
                    and self._now() < cancel_deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            self._sleep(self.cancel_settle)
            return "failed"
        result = result_future.result()
        if result is None or result.result.error_code != 0:
            code = None if result is None else result.result.error_code
            self.get_logger().warning(f"执行失败 code={code}")
            return "failed"
        return "ok"

    def _plan_execute(self, goal, label, path_z_min=None):
        for _attempt in range(self.max_exec_replans + 1):
            trajectory = self._plan(goal, path_z_min)
            if trajectory is None and path_z_min is not None:
                self.get_logger().warning(
                    f"带高度约束的规划失败，回退无约束: {label}")
                trajectory = self._plan(goal, None)
            if trajectory is None:
                return False
            trajectory.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_pub.publish(trajectory)
            status = self._execute(trajectory)
            if status == "ok":
                return True
            if status == "blocked":
                self.runtime_replans += 1
                self.get_logger().warning(
                    f"运行时重规划 {self.runtime_replans}: {label}")
                continue
            return False
        return False

    # ── 视角生成 / 排序 ──
    def _generate_views(self):
        views = []
        for target_index, target in enumerate(self.targets):
            for elevation in self.elevations:
                elevation_rad = math.radians(elevation)
                if abs(math.cos(elevation_rad)) < 1e-6:
                    # 顶视：所有方位角得到同一相机位置，只留一个
                    azimuth_values = [0.0]
                elif elevation < self.low_elevation_threshold:
                    azimuth_values = [
                        a for a in self._azimuth_values()
                        if abs(_wrap_deg(a)) <= self.low_max_azimuth]
                else:
                    azimuth_values = self._azimuth_values()
                for azimuth in azimuth_values:
                    azimuth_rad = math.radians(azimuth)
                    direction = np.array([
                        math.cos(elevation_rad) * math.cos(azimuth_rad),
                        math.cos(elevation_rad) * math.sin(azimuth_rad),
                        math.sin(elevation_rad),
                    ])
                    views.append({
                        "target": target,
                        "direction": direction,
                        "elevation": elevation,
                        "azimuth": azimuth,
                        "label": (f"target{target_index} "
                                  f"elev{elevation:g} azim{azimuth:g}"),
                    })
        return views

    def _azimuth_values(self):
        step = 360.0 / float(self.azimuths)
        return [((i * step + 180.0) % 360.0) - 180.0
                for i in range(self.azimuths)]

    @staticmethod
    def _is_top_view(view):
        return abs(math.cos(math.radians(view["elevation"]))) < 1e-6

    def _order_views(self, views):
        if not self.top_first:
            return self._order_greedy(views)
        # 顶视优先，然后按仰角从高到低分组；低环最后。组内再用关节空间
        # 最近邻，减少来回摆臂。
        groups = {}
        for view in views:
            groups.setdefault(round(view["elevation"], 3), []).append(view)
        current = np.array(self._current_positions())
        ordered = []
        for elevation in sorted(groups, reverse=True):
            remaining = list(groups[elevation])
            while remaining:
                costs = [float(np.sum((np.array(v["q"]) - current) ** 2))
                         for v in remaining]
                index = int(np.argmin(costs))
                view = remaining.pop(index)
                ordered.append(view)
                current = np.array(view["q"])
        return ordered

    def _order_greedy(self, views):
        remaining = list(views)
        current = np.array(self._current_positions())
        ordered = []
        while remaining:
            costs = [float(np.sum((np.array(v["q"]) - current) ** 2))
                     for v in remaining]
            index = int(np.argmin(costs))
            view = remaining.pop(index)
            ordered.append(view)
            current = np.array(view["q"])
        return ordered

    # ── 扫描主流程 ──
    def _clear_map(self):
        if not self.clear_octomap:
            return
        if not self.clear_client.service_is_ready():
            return
        if self._call(self.clear_client, Empty.Request(),
                      timeout=5.0) is not None:
            self.get_logger().info("已清除 Octomap，等待重新建图")
            self.octomap_seen = False
            self._sleep(self.clear_wait)

    def _run_scan(self):
        self.cancel_requested = False
        self.scan_state = "preparing"
        self.reason = ""
        self.views_total = self.views_done = self.views_failed = 0
        self.current_view = ""
        self.cloud_frames = 0
        self._init_coverage()
        self._publish_status()

        if not self.dry_run:
            self._clear_map()
            if not self._plan_execute(list(READY), "ready"):
                self.get_logger().warning("回 ready 失败，继续扫描")

        views = self._generate_views()
        self.views_total = len(views)
        if not views:
            self.scan_state = "failed"
            self.reason = "no_views"
            self._publish_status()
            return

        feasible = []
        for view in views:
            if self.cancel_requested:
                break
            self.current_view = view["label"]
            self._publish_status()
            solution, position = self._solve_view(view)
            if solution is None:
                self.views_failed += 1
                self.get_logger().warning(f"IK 不可达，跳过: {view['label']}")
                continue
            view["q"] = solution
            view["position"] = position
            if self.safe_transfer and not self._is_top_view(view):
                pre, pre_position = self._solve_preview(view)
                if pre is None:
                    self.get_logger().warning(
                        f"预视角 IK 不可达，直接转移: {view['label']}")
                else:
                    view["pre_q"] = pre
                    view["pre_position"] = pre_position
            feasible.append(view)
            self.get_logger().info(f"IK 可行: {view['label']}")

        if self.max_views > 0:
            feasible = feasible[: self.max_views]
        self.views_total = len(feasible) + self.views_failed

        if not feasible:
            self.current_view = ""
            if self.cancel_requested:
                self.scan_state = "cancelled"
                self.reason = "user_cancel"
                self._publish_status()
                if self.return_home and not self.dry_run:
                    self._plan_execute(list(READY), "home")
                return
            self.scan_state = "failed"
            self.reason = "no_feasible_views"
            self._publish_status()
            return

        order = self._order_views(feasible)
        self.scan_state = "scanning"
        self._publish_status()

        high = True
        last_pre_q = None
        for index, view in enumerate(order):
            if self.cancel_requested:
                break
            self.current_view = view["label"]
            self._publish_status()
            if self.dry_run:
                pre = view.get("pre_position")
                self.get_logger().info(
                    f"[dry-run] {view['label']} camera="
                    f"{np.round(view['position'], 3).tolist()} q="
                    f"{np.round(view['q'], 3).tolist()} pre="
                    f"{None if pre is None else np.round(pre, 3).tolist()}")
                self.views_done += 1
                continue

            reached = False
            pre_q = view.get("pre_q")
            if self.safe_transfer and pre_q is not None:
                # 若上一步在低处（上一个视角后没有升起来），先回到上一个
                # 预视角；否则带高度约束的转移会因起点在盒子外而失败。
                if not high and last_pre_q is not None:
                    if self._plan_execute(last_pre_q,
                                          view["label"] + " ascend"):
                        high = True
                    else:
                        self.get_logger().warning(
                            "上升失败，直接尝试目标视角")
                if high and self._plan_execute(
                        pre_q, view["label"] + " pre",
                        path_z_min=self.safe_transfer_z):
                    reached = self._plan_execute(view["q"], view["label"])
            if not reached:
                reached = self._plan_execute(view["q"], view["label"])
            if not reached:
                # 扫描期间 Octomap 还在长；IK 时合法的目标位形可能在规划时
                # 已经贴上新观测到的障碍。换一个 IK 解再试一次。
                retry, retry_position = self._solve_view(view)
                if retry is None or not self._plan_execute(
                        retry, view["label"] + " retry"):
                    self.views_failed += 1
                    self._publish_status()
                    continue
                view["q"] = retry
                view["position"] = retry_position
                if self.safe_transfer and not self._is_top_view(view):
                    pre, pre_position = self._solve_preview(view)
                    view["pre_q"] = pre
                    view["pre_position"] = pre_position

            self.views_done += 1
            self.capture_ready = True
            # 每个视角重新开一个采集窗口，别让上一视角的节流把第一帧吃掉
            self.last_coverage_time = 0.0
            self._sleep(self.dwell)
            self.capture_ready = False
            self._publish_status()
            self.get_logger().info(
                f"视角完成 {self.views_done}/{len(order)}: {view['label']} "
                f"coverage={self.coverage:.2%} "
                f"cloud_frames={self.cloud_frames}")

            # 采集后先回到该视角的预视角，下一次跨工位转移从高处开始
            last_pre_q = view.get("pre_q")
            if self.safe_transfer and last_pre_q is not None \
                    and index + 1 < len(order):
                if self._plan_execute(last_pre_q,
                                      view["label"] + " ascend"):
                    high = True
                else:
                    high = False
                    self.get_logger().warning(
                        "视角后上升失败，下一次转移不使用高度约束")
            else:
                high = self._is_top_view(view)

            if self.coverage >= self.coverage_target:
                self.reason = "coverage_target_reached"
                break

        self.current_view = ""
        if self.cancel_requested:
            self.scan_state = "cancelled"
            self.reason = self.reason or "user_cancel"
        else:
            self.scan_state = "done"
            if not self.reason:
                self.reason = ("coverage_target_reached"
                               if self.coverage >= self.coverage_target
                               else "views_exhausted")

        if self.return_home and not self.dry_run:
            if not high and last_pre_q is not None:
                self._plan_execute(last_pre_q, "ascend before home")
            self._plan_execute(list(READY), "home")
        # 执行阶段的失败也会累加 views_failed；保证状态里
        # views_done + views_failed 不会超过 views_total。
        self.views_total = max(self.views_total,
                               self.views_done + self.views_failed)
        self._publish_status()

    # ── 状态 ──
    def _publish_status(self):
        payload = {
            "state": self.scan_state,
            "reason": self.reason,
            "current_view": self.current_view,
            "views_total": max(self.views_total,
                               self.views_done + self.views_failed),
            "views_done": self.views_done,
            "views_failed": self.views_failed,
            "coverage": round(self.coverage, 4),
            "observed_voxels": len(self.observed),
            "occupied_voxels": len(self.occupied),
            "total_voxels": self.total_voxels,
            "cloud_frames": self.cloud_frames,
            "octomap_seen": self.octomap_seen,
            "scene_objects_seen": self.scene_objects_seen,
            "safe_transfer": self.safe_transfer,
            "runtime_check": self.runtime_check,
            "runtime_replans": self.runtime_replans,
            "validity_checks": self.validity_checks,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def run(self):
        if not self.wait_for_stack():
            self.scan_state = "failed"
            self.reason = "stack_not_ready"
            self._publish_status()
            return
        if self.auto_start:
            self.get_logger().info(f"{self.start_delay:g} s 后自动开始扫描")
            self._sleep(self.start_delay)
            self.start_requested = True
        while rclpy.ok():
            if not self.start_requested:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            self.start_requested = False
            self._run_scan()
            self._publish_status()
            if self.scan_state in ("failed", "cancelled"):
                self.get_logger().warning(
                    f"扫描结束 state={self.scan_state} reason={self.reason}")
            else:
                self.get_logger().info(
                    f"扫描结束 coverage={self.coverage:.2%} "
                    f"views={self.views_done}/{self.views_total} "
                    f"failed={self.views_failed}")


def main():
    rclpy.init()
    node = None
    try:
        node = WristScan()
        node.run()
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
