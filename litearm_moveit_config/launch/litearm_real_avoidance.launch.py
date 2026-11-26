#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_real_avoidance.launch.py — 真机 D435i 感知 + MoveIt 避障栈。

启动内容
========
    1. 真机控制栈（可选）   litearm_ros2_control/litearm_control.launch.py
    2. move_group + RViz    litearm_moveit.launch.py（arm-only 模型）
    3. RealSense D435i      realsense2_camera/rs_launch.py（可选）
    4. 静态 TF              base_link -> camera_link（读手眼标定 JSON）
    5. d435i_supervisor     规划 / 执行 / 剩余轨迹复检 / 取消 / 重规划
    6. 已知静态场景（可选）  scene_builder.py + scene_real.yaml

不启动
======
    Gazebo、仿真相机、movable_block / dynamic_obstacle、夹爪模型。
    这是真机栈：use_sim_time=false，机器人模型是 litearm_moveit_config 的
    arm-only URDF/SRDF（当前未安装夹爪）。

标定
====
默认使用 config/handeye/ 下按 D435i 实际彩色内参重算过的 TSAI 结果。
原始 handeye_eye_to_hand_TSAI_20261126_102300.json 用的是 fx=615.2，
与 1280x720 彩色流的实际 fx=910.37 不一致；不要直接用原始文件做真机运动。
标定帧默认 camera_color_optical_frame；点云在 camera_depth_optical_frame，
两者通过驱动发布的 camera_link -> camera_*_optical_frame 静态 TF 连接。

    # 正常启动（真机、相机、MoveIt、supervisor）
    ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py

    # 只做感知/规划，不开真机控制栈（控制栈已在别处运行）
    ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
        start_control:=false

    # 第一次上真机：先只开相机 + MoveIt + RViz，手动在 RViz 里 Plan，
    # 确认点云、Octomap、自滤波都干净后再启动 supervisor 执行
    ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
        start_supervisor:=false

    # 换标定文件：
    ros2 launch litearm_moveit_config litearm_real_avoidance.launch.py \
        calibration_file:=/path/to/handeye.json

安全
====
    * 首次运行把 velocity_scaling / acceleration_scaling 保持在 0.05~0.1；
    * 物理急停必须在手边，工作区清空；
    * supervisor 在 /joint_states 或 Octomap 未就绪时拒绝运动；
    * 相机避障不是安全认证功能，不能替代急停和固件保护。
"""

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_DOMAIN_ID = "42"


def _parse_bool(value, fallback=False):
    if value is None:
        return fallback
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return fallback


def _isolation_env(domain_id, localhost_only):
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _isolation_hint(domain_id, localhost_only):
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    if _parse_bool(localhost_only, True):
        return f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY=1"
    return (f"export ROS_DOMAIN_ID={domain}    "
            "(localhost-only is off: expect cross-machine traffic)")


def _float(value, fallback):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return fallback
    return out if math.isfinite(out) else fallback


def _calibration_summary(path):
    """从标定 JSON 摘出质量指标，用于启动日志；读不到就返回 None。"""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            doc = json.load(handle)
    except (OSError, ValueError):
        return None
    metrics = doc.get("metrics", {}) or {}
    camera = doc.get("camera", {}) or {}
    k = camera.get("intrinsic_matrix")
    fx = None
    if isinstance(k, list) and k and isinstance(k[0], list) and k[0]:
        try:
            fx = float(k[0][0])
        except (TypeError, ValueError):
            fx = None
    return {
        "quality": doc.get("quality", "未知"),
        "mode": doc.get("mode", ""),
        "median_mm": metrics.get("median_translation_mm"),
        "max_mm": metrics.get("max_translation_mm"),
        "spread_mm": metrics.get("constant_frame_spread_mm"),
        "fx": fx,
    }


def _declare_arguments():
    return [
        # ── 真机控制 / MoveIt ──
        DeclareLaunchArgument("start_control", default_value="true",
                              description="Also start litearm_ros2_control "
                                          "(direct USB); false if it already runs"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Start RViz2 with the MotionPlanning panel"),
        DeclareLaunchArgument("port", default_value="",
                              description="USB CDC device path; empty = auto-discovery"),
        DeclareLaunchArgument("clear_faults", default_value="true",
                              description="Passed to the control stack: clear a latched "
                                          "fault at configure"),
        # ── 相机 ──
        DeclareLaunchArgument("start_camera", default_value="true",
                              description="Start realsense2_camera; false if the driver "
                                          "runs elsewhere"),
        DeclareLaunchArgument("camera_namespace", default_value="/",
                              description="Camera namespace; '/' + camera_name=camera "
                                          "gives /camera/... topics"),
        DeclareLaunchArgument("camera_name", default_value="camera",
                              description="Camera node name"),
        DeclareLaunchArgument("camera_serial", default_value="",
                              description="RealSense serial number; empty = first device "
                                          "(the connected D435i is 243222074636)"),
        DeclareLaunchArgument("depth_profile", default_value="848x480x30",
                              description="Depth stream profile"),
        DeclareLaunchArgument("color_profile", default_value="848x480x30",
                              description="Color stream profile; align_depth needs color"),
        # ── 标定 / TF ──
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="Hand-eye JSON; empty = the package's "
                                          "config/handeye/..._fixed.json"),
        DeclareLaunchArgument("calibration_frame",
                              default_value="camera_color_optical_frame",
                              description="Frame the calibration T_base_cam refers to; "
                                          "use camera_depth_optical_frame if the board was "
                                          "observed through the depth/IR stream"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Robot base frame"),
        DeclareLaunchArgument("camera_link_frame", default_value="camera_link",
                              description="RealSense body frame"),
        DeclareLaunchArgument("tf_timeout", default_value="10.0",
                              description="Seconds before logging a missing driver TF"),
        # ── Octomap / 自滤波 ──
        DeclareLaunchArgument("sensors_file", default_value="",
                              description="sensors_3d YAML; empty = the package's "
                                          "config/sensors_3d_real.yaml"),
        DeclareLaunchArgument("shape_tf_cache_wait", default_value="0.5",
                              description="ShapeMask TF lookup wait (s); raise it if the "
                                          "arm leaks into the Octomap"),
        # ── supervisor ──
        DeclareLaunchArgument("start_supervisor", default_value="true",
                              description="Start d435i_supervisor; false = RViz Plan only"),
        DeclareLaunchArgument("start_goal_marker", default_value="true",
                              description="Start goal_marker.py: an RViz 6-DOF goal "
                                          "marker that publishes to the supervisor"),
        DeclareLaunchArgument("retreat_enabled", default_value="false",
                              description="After max_replans/replan_failed: false = "
                                          "safe_stop where the arm is; true = low-speed "
                                          "retreat to retreat_positions (ready)"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.08",
                              description="Supervisor velocity scaling; keep 0.05~0.1 "
                                          "for the first real runs"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.08",
                              description="Supervisor acceleration scaling"),
        DeclareLaunchArgument("max_state_age", default_value="0.5",
                              description="Refuse motion when /joint_states is older (s)"),
        DeclareLaunchArgument("max_sensor_age", default_value="1.0",
                              description="Refuse motion when CameraInfo is older (s)"),
        DeclareLaunchArgument("validity_check_period", default_value="0.5",
                              description="Remaining-trajectory recheck period (s)"),
        # ── 其他 ──
        DeclareLaunchArgument("scene_file", default_value="",
                              description="Optional known static scene YAML; empty = "
                                          "Octomap only"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config; empty = the perception package's "
                                          "d435i.rviz if installed"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group log level"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
                              description="ROS domain of this stack (deliberately not 0)"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = localhost discovery only"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    start_control = _parse_bool(resolve("start_control"), True)
    use_rviz = _parse_bool(resolve("use_rviz"), True)
    start_camera = _parse_bool(resolve("start_camera"), True)
    start_supervisor = _parse_bool(resolve("start_supervisor"), True)
    start_goal_marker = _parse_bool(resolve("start_goal_marker"), True)
    retreat_enabled = _parse_bool(resolve("retreat_enabled"), False)
    port = resolve("port").strip()
    clear_faults = _parse_bool(resolve("clear_faults"), True)
    camera_namespace = resolve("camera_namespace").strip() or "/"
    camera_name = resolve("camera_name").strip() or "camera"
    camera_serial = resolve("camera_serial").strip()
    depth_profile = resolve("depth_profile").strip() or "848x480x30"
    color_profile = resolve("color_profile").strip() or "848x480x30"
    calibration_frame = (resolve("calibration_frame").strip()
                         or "camera_color_optical_frame")
    base_frame = resolve("base_frame").strip() or "base_link"
    camera_link_frame = resolve("camera_link_frame").strip() or "camera_link"
    tf_timeout = _float(resolve("tf_timeout"), 10.0)
    shape_wait = _float(resolve("shape_tf_cache_wait"), 0.5)
    velocity_scaling = _float(resolve("velocity_scaling"), 0.08)
    acceleration_scaling = _float(resolve("acceleration_scaling"), 0.08)
    max_state_age = _float(resolve("max_state_age"), 0.5)
    max_sensor_age = _float(resolve("max_sensor_age"), 1.0)
    validity_period = _float(resolve("validity_check_period"), 0.5)
    scene_file = resolve("scene_file").strip()
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    log_level = resolve("log_level").strip() or "info"
    env = _isolation_env(domain_id, localhost_only)

    moveit_share = get_package_share_directory("litearm_moveit_config")
    perception_share = get_package_share_directory("litearm_d435i_perception")

    calibration_file = resolve("calibration_file").strip() or os.path.join(
        moveit_share, "config", "handeye",
        "handeye_eye_to_hand_TSAI_20261126_102300_fixed.json")
    calibration_file = os.path.abspath(os.path.expanduser(calibration_file))
    sensors_file = resolve("sensors_file").strip() or os.path.join(
        moveit_share, "config", "sensors_3d_real.yaml")
    sensors_file = os.path.abspath(os.path.expanduser(sensors_file))
    rviz_config = resolve("rviz_config").strip()
    if not rviz_config:
        candidate = os.path.join(
            moveit_share, "rviz", "real_avoidance.rviz")
        if not os.path.isfile(candidate):
            candidate = os.path.join(perception_share, "rviz", "d435i.rviz")
        rviz_config = candidate if os.path.isfile(candidate) else os.path.join(
            moveit_share, "rviz", "moveit.rviz")
    rviz_config = os.path.abspath(os.path.expanduser(rviz_config))

    calibration = _calibration_summary(calibration_file)
    calibration_text = "无法读取标定文件"
    if calibration is not None:
        parts = [f"质量 {calibration['quality']}"]
        if calibration["median_mm"] is not None:
            parts.append(f"中位平移 {calibration['median_mm']:.2f} mm")
        if calibration["max_mm"] is not None:
            parts.append(f"最大 {calibration['max_mm']:.2f} mm")
        if calibration["spread_mm"] is not None:
            parts.append(f"常值帧散布 {calibration['spread_mm']:.2f} mm")
        if calibration["fx"] is not None:
            parts.append(f"fx={calibration['fx']:.2f}")
        calibration_text = ", ".join(parts)

    actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", env["ROS_DOMAIN_ID"]),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", env["ROS_LOCALHOST_ONLY"]),
    ]

    # 1) 真机控制栈 + move_group + RViz；传感器配置通过参数注入 move_group。
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "litearm_moveit.launch.py")),
        launch_arguments={
            "start_control": "true" if start_control else "false",
            "use_rviz": "true" if use_rviz else "false",
            "port": port,
            "clear_faults": "true" if clear_faults else "false",
            "sensors_3d_file": sensors_file,
            "shape_tf_cache_wait": f"{shape_wait:.3f}",
            "rviz_config": rviz_config,
            "log_level": log_level,
            "ros_domain_id": domain_id,
            "ros_localhost_only": localhost_only,
        }.items(),
    ))

    # 2) RealSense 驱动。直接起 realsense2_camera_node，而不是 include
    #    rs_launch.py：后者的 parse_launch_arguments 会把父 launch 的全部
    #    参数当成未知参数刷一屏警告，直接起节点则只接收本节点需要的参数。
    #    namespace='/' + name='camera' 得到 /camera/... 话题和 camera_* 帧；
    #    align_depth 让点云出现在 /camera/depth/color/points。
    if start_camera:
        camera_parameters = {
            "camera_name": camera_name,
            "pointcloud.enable": True,
            "align_depth.enable": True,
            "enable_color": True,
            "enable_sync": True,
            "depth_module.depth_profile": depth_profile,
            "rgb_camera.color_profile": color_profile,
            "publish_tf": True,
            "tf_publish_rate": 0.0,
            "decimation_filter.enable": True,
            "decimation_filter.filter_magnitude": 2,
            "spatial_filter.enable": True,
            "temporal_filter.enable": False,
            "hole_filling_filter.enable": False,
            "clip_distance": 2.5,
            "diagnostics_period": 0.0,
            "initial_reset": False,
        }
        if camera_serial:
            camera_parameters["serial_no"] = camera_serial
        actions.append(Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=camera_namespace,
            name=camera_name,
            output="screen",
            additional_env=env,
            parameters=[camera_parameters],
        ))

    # 3) 标定外参 -> base_link -> camera_link 静态 TF。
    actions.append(Node(
        package="litearm_moveit_config",
        executable="camera_calibration_tf.py",
        name="camera_calibration_tf",
        output="screen",
        additional_env=env,
        parameters=[{
            "calibration_file": calibration_file,
            "calibration_frame": calibration_frame,
            "base_frame": base_frame,
            "camera_link_frame": camera_link_frame,
            "tf_timeout": tf_timeout,
            "use_sim_time": False,
        }],
    ))

    # 4) 规划 / 执行 / 复检 / 重规划。默认低速，数据过期时拒绝运动。
    if start_supervisor:
        actions.append(Node(
            package="litearm_d435i_perception",
            executable="d435i_supervisor.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "use_sim_time": False,
                "group_name": "litearm_arm",
                "ee_link": "ee_link",
                "sensor_topic": "/camera/color/camera_info",
                "require_sensor": True,
                "require_octomap": True,
                "max_state_age": max_state_age,
                "max_sensor_age": max_sensor_age,
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
                "planning_time": 4.0,
                "plan_attempts": 8,
                "max_replans": 5,
                "validity_check_period": validity_period,
                "validity_stride": 2,
                "retreat_enabled": retreat_enabled,
                "retreat_positions": [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0],
                "retreat_velocity_scaling": 0.05,
                "clear_octomap_on_stale": False,
                # base_link 和台面/安装板的固定接触不算“可避让障碍”；
                # 复检忽略它，否则会反复取消—重规划，机械臂一顿一顿。
                "ignore_base_link_contacts": True,
            }],
        ))

    # 5) RViz 6-DOF 目标 marker：只负责选目标，执行仍交给 supervisor。
    if start_goal_marker:
        actions.append(Node(
            package="litearm_moveit_config",
            executable="goal_marker.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "use_sim_time": False,
                "base_frame": base_frame,
                "ee_link": "ee_link",
                "group_name": "litearm_arm",
                "goal_topic": "/d435i_supervisor/goal_pose",
                "joint_goal_topic": "/d435i_supervisor/goal_joints",
                # 回零点 = 7 轴全 0；如果想回 ready，改成
                # [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]。
                "home_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "cancel_service": "/d435i_supervisor/cancel",
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
                "planning_time": 4.0,
                "plan_attempts": 8,
                # 拖动时实时 IK 幽灵预览（只预览，不执行）
                "preview_rate": 10.0,
                "preview_avoid_collisions": False,
            }],
        ))

    # 6) 可选：已知静态场景（台面/夹具），补相机遮挡看不到的部分。
    if scene_file:
        actions.append(Node(
            package="litearm_obstacle_avoidance",
            executable="scene_builder.py",
            output="screen",
            additional_env=env,
            parameters=[{"scene_file": os.path.abspath(
                os.path.expanduser(scene_file))}],
        ))

    actions.insert(0, LogInfo(msg=(
        "──────── litearm 真机避障 ────────\n"
        f"  控制栈: {'本 launch 启动' if start_control else '假设已在别处运行'}\n"
        f"  相机: {'启动' if start_camera else '未启动'} "
        f"({camera_namespace.rstrip('/')}/{camera_name}), "
        f"点云 /camera/depth/color/points\n"
        f"  标定: {calibration_file}\n"
        f"       {calibration_text}\n"
        f"  标定帧: {calibration_frame} -> 发布 {base_frame} -> {camera_link_frame}\n"
        f"  Octomap: {sensors_file} (ShapeMask 等待 {shape_wait:.2f} s)\n"
        f"  Supervisor: {'启动' if start_supervisor else '未启动'} "
        f"(velocity={velocity_scaling:.2f}, accel={acceleration_scaling:.2f})\n"
        f"  Goal marker: {'启动' if start_goal_marker else '未启动'} "
        f"(RViz InteractiveMarkers, namespace /goal_marker)\n"
        f"  失败后: {'低速撤退到 ready' if retreat_enabled else '原地 safe_stop'}\n"
        f"  RViz: {rviz_config}\n"
        f"  执行: RViz 里拖动 marker -> 右键“发送目标”；"
        f"或 ros2 run litearm_moveit_config goal_tool.py --pose ...\n"
        f"  ⚠ 首次真机运行：急停在手，工作区清空；先在 RViz 里 Plan，"
        f"确认没有幽灵障碍后再执行。\n"
        f"  ⚠ 终端里使用 ros2 命令前先执行:\n"
        f"        {_isolation_hint(domain_id, localhost_only)}\n"
        "────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
