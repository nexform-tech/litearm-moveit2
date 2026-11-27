#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_demo.launch.py — 带夹爪的组合模型规划演示（不接控制栈）。

阶段 0 的验收入口：move_group + RViz，机器人描述换成
「机械臂 + LiteGrip 夹爪」（config/litearm_litegrip.urdf.xacro），
SRDF 换成同一个组合模型（config/litearm_litegrip.srdf）。

本文件只起 move_group（和 RViz），不包含控制栈：执行需要控制栈，
真机栈直连 USB，干跑栈见 fake_control.launch.py / obstacle_fake.launch.py。

    # 组合模型 + RViz（仅规划）
    ros2 launch litearm_obstacle_avoidance obstacle_demo.launch.py

    # 无显示环境（配合 litearm_moveit_config 的规划探针）
    ros2 launch litearm_obstacle_avoidance obstacle_demo.launch.py use_rviz:=false

robot_description_file 默认是连接真机 ros2_control 的组合模型；
obstacle_fake.launch.py 用同一个 launch、把它换成假硬件版本，
保证两边的运动学、SRDF、规划参数完全一致。

scene_file 非空时会启动 scene_builder.py，把 scene.yaml 里的静态环境
（工作台/夹具）通过 /apply_planning_scene 写进 PlanningScene；默认为空，
即只加载机器人。obstacle_fake.launch.py 默认传入本包的 scene.yaml。

规划参数（kinematics / joint_limits / ompl）全部来自
litearm_moveit_config，本 launch 只覆盖 robot_description 与 SRDF，
所以规划行为与既有 arm-only 配置保持一致，唯一差别是模型多了夹爪。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_DOMAIN_ID = "42"
"""与 litearm_moveit_config 的 launch 一致：刻意不用 domain 0。"""


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


def _declare_arguments(default_robot_description):
    return [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="启动 RViz2（MoveIt MotionPlanning 面板）"),
        DeclareLaunchArgument("robot_description_file", default_value=default_robot_description,
                              description="组合模型的 xacro 入口；默认接真机 ros2_control，"
                                          "obstacle_fake.launch.py 会覆盖成假硬件版本"),
        DeclareLaunchArgument("sensors_3d_file", default_value="",
                              description="Octomap 传感器配置（sensors_3d.yaml）；空 = 不启用 "
                                          "3D 传感器。obstacle_fake.launch.py 用 "
                                          "use_synthetic_cloud:=true 时传入本包的合成点云配置"),
        DeclareLaunchArgument("scene_file", default_value="",
                              description="静态环境（scene.yaml）；空 = 不加载世界物体。"
                                          "非空时启动 scene_builder 把对象写进 PlanningScene"),
        DeclareLaunchArgument("gripper_envelope_margin", default_value="0.01",
                              description="夹爪手指全行程保守包络在每个方向的外扩量（m）；"
                                          "0 表示只覆盖手指扫掠体积本身"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group 日志级别"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
                              description="本栈的 ROS domain（刻意不是 0）"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = 仅本机发现"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    use_rviz = _parse_bool(resolve("use_rviz"), True)
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    own_share = get_package_share_directory("litearm_obstacle_avoidance")
    litearm_share = get_package_share_directory("litearm_moveit_config")

    robot_description_file = resolve("robot_description_file").strip()
    if not robot_description_file:
        robot_description_file = os.path.join(
            own_share, "config", "litearm_litegrip.urdf.xacro")

    # 只覆盖机器人描述与语义描述；其余规划参数沿用 litearm_moveit_config。
    builder = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(
            file_path=robot_description_file,
            mappings={"gripper_envelope_margin": resolve("gripper_envelope_margin")})
        .robot_description_semantic(
            file_path=os.path.join(own_share, "config", "litearm_litegrip.srdf"))
        .planning_pipelines(pipelines=["ompl"])
    )
    # 只有显式给了传感器配置才启用 Octomap；真机默认不订阅任何点云话题。
    sensors_3d_file = resolve("sensors_3d_file").strip()
    if sensors_3d_file:
        builder = builder.sensors_3d(file_path=sensors_3d_file)
    moveit_config = builder.to_moveit_configs()

    actions = [Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=env,
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", resolve("log_level")],
    )]

    scene_file = resolve("scene_file").strip()
    if scene_file:
        actions.append(Node(
            package="litearm_obstacle_avoidance",
            executable="scene_builder.py",
            output="screen",
            additional_env=env,
            parameters=[{"scene_file": os.path.abspath(scene_file)}],
        ))

    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(litearm_share, "rviz", "moveit.rviz")],
            output="log",
            additional_env=env,
            # RViz 的 MoveIt 插件需要自己的一份参数，move_group 的不会传过去。
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ))

    actions.insert(0, LogInfo(msg=(
        "──────── litearm + LiteGrip MoveIt 栈 ────────\n"
        "  move_group: 已启用，规划可用\n"
        "  执行: 取决于是否另有控制栈（真机 / obstacle_fake 干跑）\n"
        "  规划组: litearm_arm（base_link -> ee_link）、gripper\n"
        "  模型:   机械臂 + 夹爪（夹爪作为碰撞体参与规划）\n"
        "  夹爪安装: ee_link + 0.0538 m（拆 10.3 mm 端盖后；端盖几何保留并被包住）\n"
        f"  夹爪包络: 全行程 + {resolve('gripper_envelope_margin')} m 外扩\n"
        f"  静态环境: {scene_file or '未加载'}\n"
        "  ⚠ 本 launch 固定 ROS domain；在自己的终端里用 ros2 命令前先执行：\n"
        f"        export ROS_DOMAIN_ID={domain_id}; export ROS_LOCALHOST_ONLY="
        f"{'1' if _parse_bool(localhost_only, True) else '0'}\n"
        "─────────────────────────────────────────────")))
    return actions


def generate_launch_description():
    own_share = get_package_share_directory("litearm_obstacle_avoidance")
    default_robot_description = os.path.join(
        own_share, "config", "litearm_litegrip.urdf.xacro")
    return LaunchDescription(_declare_arguments(default_robot_description) +
                             [OpaqueFunction(function=_launch_setup)])
