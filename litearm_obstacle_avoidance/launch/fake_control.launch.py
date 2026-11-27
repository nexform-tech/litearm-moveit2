#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_control.launch.py — 干跑控制栈（mock_components，不接机械臂）。

起一套与真机同形的 ros2_control 栈，但硬件插件换成
mock_components/GenericSystem：它把下发的 position / velocity 指令直接
回读为 state。JTC、joint_state_broadcaster、controller_manager 走的都是
真实流程，所以 MoveIt 的 /move_action -> follow_joint_trajectory ->
轨迹推进可以完整跑通，不需要 USB、不会动机械臂。

模型用 litearm_litegrip_fake.urdf.xacro（与真机版本共用几何与安装变换），
控制器参数默认复用 litearm_ros2_control/config/litearm_controllers.yaml，
因此 JTC 的关节名、接口组合、跟踪容差与真机完全一致。

    # 只起控制栈（不含 MoveIt）
    ros2 launch litearm_obstacle_avoidance fake_control.launch.py

    # 完整干跑：控制栈 + MoveIt + 规划（推荐）
    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py

⚠ 这不是动力学仿真：没有惯性、重力、跟踪误差，状态永远等于指令。
  执行链路可以验证，控制品质不能。真机栈与它同域同时运行会抢
  /controller_manager 与控制器 action，不要同时起。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_DOMAIN_ID = "42"
"""与真机栈一致：刻意不用 domain 0。"""


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


def _declare_arguments():
    return [
        DeclareLaunchArgument(
            "controllers_file", default_value="",
            description="控制器参数文件；空 = litearm_ros2_control 的 "
                        "config/litearm_controllers.yaml"),
        DeclareLaunchArgument(
            "ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
            description="本栈的 ROS domain（刻意不是 0）"),
        DeclareLaunchArgument(
            "ros_localhost_only", default_value="true",
            description="true = 仅本机发现"),
    ]


def _spawner(name, env):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
        output="screen",
        additional_env=env,
        emulate_tty=True,
    )


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    controllers_file = resolve("controllers_file").strip()
    if not controllers_file:
        controllers_file = os.path.join(
            get_package_share_directory("litearm_ros2_control"),
            "config", "litearm_controllers.yaml")
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    xacro_file = os.path.join(
        get_package_share_directory("litearm_obstacle_avoidance"),
        "config", "litearm_litegrip_fake.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            additional_env=env,
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            additional_env=env,
            parameters=[{"robot_description": robot_description}, controllers_file],
        ),
    ]

    broadcaster = _spawner("joint_state_broadcaster", env)
    trajectory = _spawner("joint_trajectory_controller", env)
    # JTC 配置时要读关节状态，所以等 broadcaster 激活退出后再起它。
    actions.append(broadcaster)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=broadcaster, on_exit=[trajectory])))

    actions.insert(0, LogInfo(msg=(
        "──────── litearm 干跑控制栈（mock_components）────────\n"
        "  硬件:   mock_components/GenericSystem（指令直接回读为状态）\n"
        f"  控制器: {controllers_file}\n"
        "  ⚠ 不会动机械臂；但同域同时起真机栈会抢 /controller_manager。\n"
        "  ⚠ 本栈固定 ROS domain；在自己的终端里用 ros2 命令前先执行：\n"
        f"        export ROS_DOMAIN_ID={domain_id}; export ROS_LOCALHOST_ONLY="
        f"{'1' if _parse_bool(localhost_only, True) else '0'}\n"
        "──────────────────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
