#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_moveit.launch.py — the MoveIt planning stack for the litearm arm.

Starts, in any combination:

    1. the control stack   litearm_ros2_control/litearm_control.launch.py
                           (ros2_control_node + the direct-USB hardware component
                           + controller_manager + JSB + JTC)
    2. move_group          MoveIt planning (OMPL + KDL)
    3. RViz                the MoveIt MotionPlanning panel

All three start by default. When the control stack already runs somewhere else
(for example a bring-up shell you started by hand), pass start_control:=false so
that two controller_managers do not fight over the same command interfaces.

    # real arm (board powered, USB connected, licence activated)
    ros2 launch litearm_moveit_config litearm_moveit.launch.py

    # a different USB device
    ros2 launch litearm_moveit_config litearm_moveit.launch.py port:=/dev/ttyACM1

    # let a latched fault stop the launch instead of clearing it
    ros2 launch litearm_moveit_config litearm_moveit.launch.py clear_faults:=false

The control stack clears a latched fault on the arm while configuring it (clear_faults,
on by default). The firmware latches an EMERGENCY or a joint_fault and then refuses
ENABLE until a reset, which would otherwise make every bring-up after a fault a two-step
operation. Turn it off when you want the fault to stop the launch so you can look at the
arm first.

    # MoveIt only, control stack already running
    ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false

Feed-forward is the firmware's business: the PD gains and the gravity, friction,
integral and kd_extra terms live in the litearm-stm32 firmware, whose model is
generated from the URDF and compiled in. The MOVE_JS channel this stack uses has
no M·q̈ or C·q̇ term, because MOVE_JS has no acceleration source.

Unlike the shared-memory deployment, this one has no dry-run mode: the direct-USB
component talks to the real board, so a launch either has an arm on the other end
of the USB cable or its hardware component fails to configure.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_DOMAIN_ID = "42"
"""Deliberately not 0: domain 0 is the one other equipment on the network uses."""


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
    """The environment every node of this stack gets: fixed domain, localhost only."""
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _isolation_hint(domain_id, localhost_only):
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    if _parse_bool(localhost_only, True):
        return f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY=1"
    return f"export ROS_DOMAIN_ID={domain}    (localhost-only is off: expect cross-machine traffic)"


def _is_true(value):
    return _parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument("start_control", default_value="true",
                              description="Also start the ros2_control stack"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Start RViz2 with the MoveIt MotionPlanning panel"),
        DeclareLaunchArgument("port", default_value="",
                              description="Passed to the control stack: USB CDC device "
                                          "path; empty = auto-discovery (1d50:606f)"),
        DeclareLaunchArgument("clear_faults", default_value="true",
                              description="Passed to the control stack: clear a latched "
                                          "arm fault (EMERGENCY / joint_fault) before the "
                                          "hardware is enabled, so a bring-up after a fault "
                                          "stays a single command; false lets the fault stop "
                                          "the launch instead"),
        DeclareLaunchArgument("sensors_3d_file", default_value="",
                              description="Optional sensors_3d YAML for move_group "
                                          "(point cloud -> Octomap); empty = no 3D sensor"),
        DeclareLaunchArgument("shape_tf_cache_wait", default_value="0.05",
                              description="ShapeMask TF lookup wait in seconds; raise it "
                                          "on a loaded real machine so the arm does not "
                                          "leak into the Octomap"),
        DeclareLaunchArgument("allow_move_group_execution", default_value="true",
                              description="Let move_group execute trajectories "
                                          "(/move_action, /execute_trajectory, RViz "
                                          "Execute). The real-avoidance launch sets this "
                                          "false so execution must go through the "
                                          "supervisor; planning services stay available"),
        DeclareLaunchArgument("start_cloud_envelope_filter", default_value="false",
                              description="Start cloud_envelope_filter: an extra robot "
                                          "collision-model envelope on the point cloud "
                                          "(3 cm total = the 2 cm MoveIt self-filter plus "
                                          "1 cm), toggled from an RViz interactive marker"),
        DeclareLaunchArgument("cloud_envelope_padding", default_value="0.03",
                              description="ShapeMask padding used by cloud_envelope_filter "
                                          "(m); 0.03 = 2 cm MoveIt self-filter + 1 cm extra"),
        DeclareLaunchArgument("cloud_envelope_enabled", default_value="false",
                              description="Initial state of the cloud envelope filter; the "
                                          "RViz marker toggles it at runtime"),
        DeclareLaunchArgument("cloud_envelope_input_topic",
                              default_value="/camera/depth/color/points",
                              description="Raw point cloud into cloud_envelope_filter"),
        DeclareLaunchArgument("cloud_envelope_output_topic",
                              default_value="/camera/depth/color/points_envelope",
                              description="Filtered point cloud for sensors_3d"),
        DeclareLaunchArgument("cloud_envelope_frame", default_value="base_link",
                              description="Robot model / marker frame for cloud_envelope_filter"),
        DeclareLaunchArgument("cloud_flying_pixel_filter", default_value="true",
                              description="Reject depth-discontinuity flying pixels in "
                                          "cloud_envelope_filter: project each point back "
                                          "to a pixel with cloud_camera_info_topic and "
                                          "drop neighbourhoods with a large depth spread"),
        DeclareLaunchArgument("cloud_flying_pixel_depth_jump", default_value="0.03",
                              description="Max depth spread (m) inside the neighbourhood "
                                          "of a pixel before it is dropped as a flying "
                                          "pixel; 0.03 keeps real surfaces, removes the "
                                          "arm/table edge fan"),
        DeclareLaunchArgument("cloud_flying_pixel_radius", default_value="1",
                              description="Pixel neighbourhood radius used by the flying "
                                          "pixel filter (px); larger = wider edge band"),
        DeclareLaunchArgument("cloud_camera_info_topic",
                              default_value="/camera/depth/camera_info",
                              description="CameraInfo used to project the point cloud "
                                          "back to pixels for the flying-pixel filter; "
                                          "must match the point cloud stream"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config file; empty = the package's "
                                          "rviz/moveit.rviz"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group log level"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
                              description="ROS domain of this stack (deliberately not 0); "
                                          "set 0 to integrate with another system"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = localhost discovery only: a second robot "
                                          "on the network otherwise shows up in RViz and "
                                          "as a second /move_action server"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    start_control = _is_true(resolve("start_control"))
    use_rviz = _is_true(resolve("use_rviz"))
    port = resolve("port").strip()
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    moveit_share = get_package_share_directory("litearm_moveit_config")
    control_share = get_package_share_directory("litearm_ros2_control")

    # The MoveIt side expands the same URDF the control stack does, so the port
    # argument has to reach both; a mismatch would not error, it would just leave
    # move_group describing a robot the controller_manager is not running.
    builder = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(mappings={"litearm_port": port})
    )
    sensors_3d_file = resolve("sensors_3d_file").strip()
    if sensors_3d_file:
        builder = builder.sensors_3d(file_path=sensors_3d_file)
    moveit_config = builder.planning_pipelines(
        pipelines=["ompl"]).to_moveit_configs()

    # ShapeMask 自滤波按点云时间戳查 TF；真机负载下 0.05 s 默认值可能不够，
    # 查不到的 link 会被静默跳过，整帧手臂点云进入 Octomap 变成幽灵障碍。
    # 0 = 保留 MoveIt 默认，不写这个参数。
    try:
        shape_wait = float(resolve("shape_tf_cache_wait"))
    except ValueError:
        shape_wait = 0.05
    if not math.isfinite(shape_wait) or shape_wait < 0.0:
        shape_wait = 0.05
    if shape_wait > 0.0:
        moveit_config.joint_limits.setdefault(
            "robot_description_planning", {})[
                "shape_transform_cache_lookup_wait_time"] = shape_wait

    allow_move_group_execution = _is_true(
        resolve("allow_move_group_execution"))
    move_group_parameters = moveit_config.to_dict()
    # move_group 的 execute 能力开关：false 时 /move_action 和 RViz Execute
    # 不能直接驱动机器人，但 /plan_kinematic_path、/check_state_validity
    # 和 supervisor 直接发给 JTC 的 action 不受影响。
    move_group_parameters["allow_trajectory_execution"] = \
        allow_move_group_execution

    actions = []

    if start_control:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_share, "launch", "litearm_control.launch.py")),
            launch_arguments={
                "port": port,
                "clear_faults": "true" if _is_true(resolve("clear_faults")) else "false",
                # The isolation arguments have to be forwarded: the included nodes
                # land in their own environment, and move_group only sees the
                # controller_manager actions when both sides share a domain.
                "ros_domain_id": domain_id,
                "ros_localhost_only": localhost_only,
            }.items(),
        ))

    actions.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=env,
        parameters=[move_group_parameters],
        arguments=["--ros-args", "--log-level", resolve("log_level")],
    ))

    start_cloud_filter = _is_true(resolve("start_cloud_envelope_filter"))
    cloud_enabled = _is_true(resolve("cloud_envelope_enabled"))
    try:
        cloud_padding = float(resolve("cloud_envelope_padding"))
    except ValueError:
        cloud_padding = 0.03
    if not math.isfinite(cloud_padding) or cloud_padding <= 0.0:
        cloud_padding = 0.03
    cloud_input = resolve("cloud_envelope_input_topic").strip() or \
        "/camera/depth/color/points"
    cloud_output = resolve("cloud_envelope_output_topic").strip() or \
        "/camera/depth/color/points_envelope"
    cloud_frame = resolve("cloud_envelope_frame").strip() or "base_link"
    cloud_flying = _is_true(resolve("cloud_flying_pixel_filter"))
    try:
        cloud_flying_jump = float(resolve("cloud_flying_pixel_depth_jump"))
    except ValueError:
        cloud_flying_jump = 0.03
    if not math.isfinite(cloud_flying_jump) or cloud_flying_jump <= 0.0:
        cloud_flying_jump = 0.03
    try:
        cloud_flying_radius = int(float(resolve("cloud_flying_pixel_radius")))
    except ValueError:
        cloud_flying_radius = 1
    cloud_flying_radius = min(8, max(0, cloud_flying_radius))
    cloud_camera_info = resolve("cloud_camera_info_topic").strip() or \
        "/camera/depth/camera_info"
    if start_cloud_filter:
        actions.append(Node(
            package="litearm_d435i_perception",
            executable="cloud_envelope_filter",
            name="cloud_envelope_filter",
            output="screen",
            additional_env=env,
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                {
                    "input_topic": cloud_input,
                    "output_topic": cloud_output,
                    "base_frame": cloud_frame,
                    "marker_frame": cloud_frame,
                    "padding": cloud_padding,
                    "enabled": cloud_enabled,
                    "flying_pixel_filter": cloud_flying,
                    "flying_pixel_depth_jump": cloud_flying_jump,
                    "flying_pixel_radius": cloud_flying_radius,
                    "camera_info_topic": cloud_camera_info,
                },
            ],
        ))

    if use_rviz:
        rviz_config = resolve("rviz_config").strip() or os.path.join(
            moveit_share, "rviz", "moveit.rviz")
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="log",
            additional_env=env,
            # RViz's MoveIt plugin needs these parameter groups itself; move_group's
            # copy does not reach it.
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ))

    if start_cloud_filter:
        cloud_filter_text = (
            f"on ({cloud_padding:.3f} m, "
            f"{'enabled' if cloud_enabled else 'disabled until toggled in RViz'}), "
            f"flying-pixel filter "
            f"{f'>{cloud_flying_jump * 1000:.0f} mm / {cloud_flying_radius} px' if cloud_flying else 'off'}")
    else:
        cloud_filter_text = "off"
    actions.insert(0, LogInfo(msg=(
        "──────── litearm MoveIt ────────\n"
        f"  control stack: {'started here' if start_control else 'expected to be running'}\n"
        f"  port: {port or '(auto-discovery, VID:PID 1d50:606f)'}\n"
        f"  3D sensor: {sensors_3d_file or 'none'}\n"
        f"  ShapeMask TF wait: {shape_wait:.2f} s\n"
        f"  move_group execution: "
        f"{'allowed (RViz Execute can drive the arm)' if allow_move_group_execution else 'disabled (use the supervisor)'}\n"
        f"  cloud envelope filter: {cloud_filter_text}\n"
        "  ⚠ This launch fixes the ROS domain; to use ros2 commands or the RViz\n"
        "    GUI from your own terminal, run:\n"
        f"        {_isolation_hint(domain_id, localhost_only)}\n"
        "────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
