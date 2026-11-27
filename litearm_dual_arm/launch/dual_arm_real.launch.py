#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dual_arm_real.launch.py — 双臂真机入口（两套 litearm_ros2_control side 栈）。

启动内容：

    robot_state_publisher   双臂 URDF（同一个 world，左右两套链）
    move_group              left_arm / right_arm / both_arms
    dual_arm_bridge.py      状态合并 + 14→7+7 轨迹转发
    dual_arm_preflight.py   只读预检（可选）
    rviz2                   双臂 MotionPlanning
    可选：两套 litearm_ros2_control side 实例（start_control:=true）

默认 execute:=false：bridge 只合并状态，MoveIt 不允许执行。要动真机必须
显式给 execute:=true，并同时满足 authorization / estop_verified /
workspace_verified / calibration.measured。

    # 外部先启动左右控制栈，再启动本 launch
    ros2 launch litearm_dual_arm dual_arm_real.launch.py

    # 由本 launch 一并启动两套 side 控制栈（两个 USB 口必须显式指定）
    ros2 launch litearm_dual_arm dual_arm_real.launch.py \
      start_control:=true \
      left_port:=/dev/serial/by-id/<left-arm> \
      right_port:=/dev/serial/by-id/<right-arm>

    # 门控执行
    ros2 launch litearm_dual_arm dual_arm_real.launch.py \
      start_control:=true \
      left_port:=/dev/serial/by-id/<left-arm> \
      right_port:=/dev/serial/by-id/<right-arm> \
      execute:=true \
      authorization:=I_UNDERSTAND_DUAL_ARM_MOTION \
      estop_verified:=true \
      workspace_verified:=true

本 launch 不替代物理急停；bridge 的 /litearm_dual_arm/stop 只是软件停止请求。
"""

from __future__ import annotations

import math
import os
import shlex
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_DOMAIN_ID = "42"
MOTION_TOKEN = "I_UNDERSTAND_DUAL_ARM_MOTION"


def _parse_bool(value, fallback=False):
    if value is None:
        return fallback
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return fallback


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _prefixed_joint_limits(path):
    limits = _load_yaml(path)["joint_limits"]
    result = {}
    for side in ("left_", "right_"):
        for joint_name, values in limits.items():
            result[f"{side}{joint_name}"] = values
    return {"joint_limits": result}


def _isolation_env(domain_id, localhost_only):
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _isolation_hint(domain_id, localhost_only):
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    localhost = "1" if _parse_bool(localhost_only, True) else "0"
    return f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY={localhost}"


def _validate_execute_gate(
    execute: bool,
    authorization: str,
    estop_verified: bool,
    workspace_verified: bool,
    calibration_measured: bool,
) -> None:
    """execute:=true 的安全门；纯函数，便于单测。"""
    if not execute:
        return
    if authorization != MOTION_TOKEN:
        raise RuntimeError(
            "execute:=true requires authorization:=" + MOTION_TOKEN
        )
    if not estop_verified:
        raise RuntimeError("execute:=true requires estop_verified:=true")
    if not workspace_verified:
        raise RuntimeError("execute:=true requires workspace_verified:=true")
    if not calibration_measured:
        raise RuntimeError(
            "execute:=true requires calibration.measured:=true in the mounts file"
        )


def _declare_arguments():
    return [
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="启动 RViz2 MotionPlanning 面板"),
        DeclareLaunchArgument(
            "mounts_file", default_value="",
            description="左右基座挂载 YAML；空 = 包内 config/dual_arm_mounts.yaml"),
        DeclareLaunchArgument(
            "scene", default_value="none",
            description="可选静态场景名；真机默认 none，dual_static 只是台架参考"),
        DeclareLaunchArgument(
            "robot_padding_m", default_value="0.02",
            description="MoveIt 机器人碰撞体统一膨胀（m）"),
        DeclareLaunchArgument(
            "allowed_start_tolerance", default_value="0.05",
            description="MoveIt 允许的轨迹起点关节差（rad）"),
        DeclareLaunchArgument(
            "execute", default_value="false",
            description="true = 允许 bridge 执行轨迹；默认只规划"),
        DeclareLaunchArgument(
            "authorization", default_value="",
            description="execute:=true 时必须等于 " + MOTION_TOKEN),
        DeclareLaunchArgument(
            "estop_verified", default_value="false",
            description="确认独立物理急停在岗"),
        DeclareLaunchArgument(
            "workspace_verified", default_value="false",
            description="确认工作空间、工具和安装已复核"),
        DeclareLaunchArgument(
            "start_control", default_value="false",
            description="true = 本 launch 一并启动两套 side 控制栈；"
                        "false = 期望左右栈已在外部运行"),
        DeclareLaunchArgument(
            "left_port", default_value="",
            description="start_control:=true 时左臂 USB 路径"),
        DeclareLaunchArgument(
            "right_port", default_value="",
            description="start_control:=true 时右臂 USB 路径"),
        DeclareLaunchArgument(
            "clear_faults", default_value="true",
            description="转发给两套 side 控制栈：配置时清除锁存故障"),
        DeclareLaunchArgument(
            "bridge_start_tolerance", default_value="0.08",
            description="bridge 允许的轨迹起点关节差（rad）"),
        DeclareLaunchArgument(
            "bridge_max_joint_velocity", default_value="0.35",
            description="bridge 允许的最大单关节速度（rad/s）"),
        DeclareLaunchArgument(
            "bridge_max_state_age", default_value="0.5",
            description="bridge 允许的关节状态最大年龄（s）"),
        DeclareLaunchArgument(
            "bridge_max_state_skew", default_value="0.1",
            description="bridge 允许的左右状态时间差（s）"),
        DeclareLaunchArgument(
            "preflight", default_value="true",
            description="启动只读预检节点"),
        DeclareLaunchArgument(
            "preflight_timeout", default_value="30.0",
            description="预检等待控制器/状态/action/TF 的超时（s）"),
        DeclareLaunchArgument(
            "preflight_output", default_value="/tmp/litearm_dual_arm_preflight.json",
            description="预检 JSON 输出路径"),
        DeclareLaunchArgument(
            "rviz_config", default_value="",
            description="RViz 配置；空 = 包内 rviz/dual_arm.rviz"),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="move_group 日志级别"),
        DeclareLaunchArgument(
            "ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
            description="本栈 ROS domain（刻意不是 0）"),
        DeclareLaunchArgument(
            "ros_localhost_only", default_value="true",
            description="true = 仅本机发现"),
    ]


def _positive_float(resolve, name: str) -> float:
    try:
        value = float(resolve(name))
    except ValueError as error:
        raise RuntimeError(f"{name} must be a number") from error
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"{name} must be positive and finite")
    return value


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    use_rviz = _parse_bool(resolve("use_rviz"), True)
    scene = resolve("scene").strip()
    mounts_file = resolve("mounts_file").strip()
    execute = _parse_bool(resolve("execute"), False)
    start_control = _parse_bool(resolve("start_control"), False)
    left_port = resolve("left_port").strip()
    right_port = resolve("right_port").strip()
    clear_faults = "true" if _parse_bool(resolve("clear_faults"), True) else "false"
    preflight = _parse_bool(resolve("preflight"), True)

    try:
        robot_padding = float(resolve("robot_padding_m"))
    except ValueError as error:
        raise RuntimeError("robot_padding_m must be a number") from error
    if not math.isfinite(robot_padding) or robot_padding < 0.0:
        raise RuntimeError("robot_padding_m must be non-negative and finite")
    allowed_start_tolerance = _positive_float(resolve, "allowed_start_tolerance")
    bridge_start_tolerance = _positive_float(resolve, "bridge_start_tolerance")
    bridge_max_joint_velocity = _positive_float(resolve, "bridge_max_joint_velocity")
    bridge_max_state_age = _positive_float(resolve, "bridge_max_state_age")
    bridge_max_state_skew = _positive_float(resolve, "bridge_max_state_skew")
    preflight_timeout = _positive_float(resolve, "preflight_timeout")

    dual_share = get_package_share_directory("litearm_dual_arm")
    moveit_share = get_package_share_directory("litearm_moveit_config")
    description_share = get_package_share_directory("litearm_description")
    control_share = get_package_share_directory("litearm_ros2_control")
    prefix = get_package_prefix("litearm_dual_arm")
    executable_dir = os.path.join(prefix, "lib", "litearm_dual_arm")

    generator = os.path.join(executable_dir, "generate_dual_arm_model.py")
    source_urdf = os.path.join(description_share, "urdf", "litearm_description.urdf")
    source_srdf = os.path.join(moveit_share, "config", "litearm.srdf")
    mounts = mounts_file or os.path.join(dual_share, "config", "dual_arm_mounts.yaml")
    scene_config = os.path.join(dual_share, "config", "dual_scenes.yaml")
    mounts_config = _load_yaml(mounts)
    calibration_measured = bool(
        (mounts_config.get("calibration") or {}).get("measured", False)
    )

    _validate_execute_gate(
        execute,
        resolve("authorization").strip(),
        _parse_bool(resolve("estop_verified"), False),
        _parse_bool(resolve("workspace_verified"), False),
        calibration_measured,
    )
    if start_control and (not left_port or not right_port):
        raise RuntimeError(
            "start_control:=true requires left_port and right_port; use "
            "/dev/serial/by-id/ paths so the two boards cannot be swapped"
        )

    def description_command(mode):
        return Command(
            [
                sys.executable,
                " ",
                shlex.quote(generator),
                " --mode ",
                mode,
                " --source-urdf ",
                shlex.quote(source_urdf),
                " --source-srdf ",
                shlex.quote(source_srdf),
                " --mounts ",
                shlex.quote(mounts),
            ]
        )

    robot_description = ParameterValue(description_command("urdf"), value_type=str)
    robot_description_semantic = ParameterValue(
        description_command("srdf"), value_type=str
    )
    robot_description_kinematics = _load_yaml(
        os.path.join(dual_share, "config", "dual_kinematics.yaml")
    )
    robot_description_planning = _prefixed_joint_limits(
        os.path.join(moveit_share, "config", "joint_limits.yaml")
    )
    ompl = _load_yaml(os.path.join(dual_share, "config", "dual_ompl.yaml"))

    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": robot_description_kinematics,
        "robot_description_planning": robot_description_planning,
        "planning": {"default_robot_padding": robot_padding},
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
        "planning_scene_monitor": {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
        "trajectory_execution": {
            "execution_duration_monitoring": True,
            "allowed_execution_duration_scaling": 2.0,
            "allowed_goal_duration_margin": 1.0,
            "allowed_start_tolerance": allowed_start_tolerance,
        },
        "moveit_manage_controllers": False,
        "allow_trajectory_execution": execute,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    parameters.update(
        _load_yaml(os.path.join(dual_share, "config", "dual_controllers.yaml"))
    )

    actions = []

    if start_control:
        control_launch = os.path.join(
            control_share, "launch", "litearm_control.launch.py"
        )
        for side, port in (("left", left_port), ("right", right_port)):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(control_launch),
                    launch_arguments={
                        "side": side,
                        "port": port,
                        "start_robot_state_publisher": "false",
                        "clear_faults": clear_faults,
                        "ros_domain_id": domain_id,
                        "ros_localhost_only": localhost_only,
                    }.items(),
                )
            )

    actions.extend(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                additional_env=env,
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                additional_env=env,
                parameters=[parameters],
                arguments=["--ros-args", "--log-level", resolve("log_level")],
            ),
            Node(
                executable=os.path.join(executable_dir, "dual_arm_bridge.py"),
                name="litearm_dual_arm_bridge",
                output="screen",
                additional_env=env,
                arguments=[
                    "--start-tolerance",
                    str(bridge_start_tolerance),
                    "--max-joint-velocity",
                    str(bridge_max_joint_velocity),
                    "--max-state-age",
                    str(bridge_max_state_age),
                    "--max-state-skew",
                    str(bridge_max_state_skew),
                ]
                + ([] if execute else ["--state-only"]),
            ),
        ]
    )

    if preflight:
        preflight_output = (
            resolve("preflight_output").strip()
            or "/tmp/litearm_dual_arm_preflight.json"
        )
        preflight_arguments = [
            "--left-namespace",
            "/left",
            "--right-namespace",
            "/right",
            "--timeout",
            str(preflight_timeout),
            "--max-state-age",
            str(bridge_max_state_age),
            "--output",
            preflight_output,
        ]
        if execute:
            preflight_arguments.append("--require-actions")
        actions.append(
            Node(
                executable=os.path.join(executable_dir, "dual_arm_preflight.py"),
                name="litearm_dual_arm_preflight",
                output="screen",
                additional_env=env,
                arguments=preflight_arguments,
            )
        )

    if scene and scene.lower() not in ("none", "off", "false"):
        actions.append(
            Node(
                executable=os.path.join(executable_dir, "dual_arm_scene.py"),
                name="litearm_dual_arm_scene",
                output="screen",
                additional_env=env,
                arguments=[
                    "--config",
                    scene_config,
                    "--scene",
                    scene,
                    "--timeout",
                    "90.0",
                ],
            )
        )

    if use_rviz:
        rviz_config = resolve("rviz_config").strip() or os.path.join(
            dual_share, "rviz", "dual_arm.rviz"
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="log",
                additional_env=env,
                parameters=[
                    {"robot_description": robot_description},
                    {"robot_description_semantic": robot_description_semantic},
                    {"robot_description_kinematics": robot_description_kinematics},
                    {"robot_description_planning": robot_description_planning},
                    {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl"},
                    {"ompl": ompl},
                ],
            )
        )

    control_note = (
        "started here (side:=left/right)" if start_control
        else "expected externally on /left and /right"
    )
    actions.insert(
        0,
        LogInfo(
            msg=(
                "──────── litearm dual arm (real) ────────\n"
                "  groups: left_arm, right_arm, both_arms\n"
                f"  mounts: {mounts}\n"
                f"  calibration.measured: {calibration_measured}\n"
                f"  control stacks: {control_note}\n"
                f"  execute: {'ENABLED' if execute else 'disabled (plan only)'}\n"
                f"  bridge: {'execution' if execute else 'state-only'}\n"
                f"  preflight: {'on' if preflight else 'off'}\n"
                "  ⚠ Physical emergency stop remains the independent safety "
                "measure.\n"
                "  ⚠ This launch fixes the ROS domain. In your own terminal run:\n"
                f"        {_isolation_hint(domain_id, localhost_only)}\n"
                "──────────────────────────────────────────"
            )
        ),
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        _declare_arguments() + [OpaqueFunction(function=_launch_setup)]
    )
