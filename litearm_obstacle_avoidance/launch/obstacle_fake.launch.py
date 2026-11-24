#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_fake.launch.py — 完整干跑：假硬件控制栈 + MoveIt（+ RViz）。

阶段 0.5 的验收入口：没有机械臂也能走完

    /move_action  ->  move_group  ->  follow_joint_trajectory  ->  JTC  ->  mock 硬件

控制栈用 mock_components/GenericSystem（下发什么状态就是什么），
MoveIt 用同一套组合模型、SRDF 与规划参数，只是 robot_description 换成
假硬件版本。执行、重规划、失败停靠都可以在这套栈上验证。

    # 只有执行链
    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py

    # 加上合成点云与 Octomap（没有相机时验证感知链路）
    ros2 launch litearm_obstacle_avoidance obstacle_fake.launch.py \
        use_synthetic_cloud:=true

    # 无显示环境，用探针做「规划 + 执行」验收（不会动机械臂）
    export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
    python3 <litearm_moveit_config>/scripts/moveit_probe.py --execute

use_synthetic_cloud:=true 会做两件事：给 move_group 传入本包
config/sensors_3d.yaml（PointCloudOctomapUpdater，话题 /synthetic_cloud），
并启动 scripts/synthetic_cloud.py 周期发布一个长方体表面点云。于是
「点云 -> Octomap -> PlanningScene」整条链路在无相机时也能验证。

默认加载本包 config/scene.yaml 的静态环境（工作台 + 两个夹具），由
scene_builder.py 写进 PlanningScene；`scene_file:=''` 可以关闭。

start_supervisor:=true 会启动 scripts/avoid_supervisor.py：订阅
/avoid_supervisor/goal，执行中检测到场景变化就取消当前轨迹、以当前状态
重规划；重规划失败则按安全策略撤退到 ready（低速），撤退也失败才停在
原地。数据过期（默认 /joint_states 超过 1 s）时拒绝新的运动。它不做
Servo 式的实时避障，语义是「停 → 重规划 → 继续」。阶段 3/4 验收
（acceptance_fake.sh --replan / --safety）会自动打开它。

⚠ JTC 的 action 要等 spawner 激活后才出现，launch 刚起来时立刻 Execute
  可能报找不到控制器；等 `ros2 control list_controllers` 显示 active 再操作。

⚠ 启用 Octomap 时，启动瞬间 move_group 可能打一条
  `shape_mask: Missing transform for shape mesh`：第一条点云比自滤波用的
  TF 先到。只出现一次，随后不再复现，属预期。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
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


def _declare_arguments(default_scene_file):
    return [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="启动 RViz2（MoveIt MotionPlanning 面板）"),
        DeclareLaunchArgument("use_synthetic_cloud", default_value="false",
                              description="true = 启动合成点云发布并给 move_group 启用 Octomap"),
        DeclareLaunchArgument("scene_file", default_value=default_scene_file,
                              description="静态环境 scene.yaml；默认本包的工位。"
                                          "传空字符串可关闭世界物体"),
        DeclareLaunchArgument("gripper_envelope_margin", default_value="0.01",
                              description="夹爪全行程保守包络的外扩量（m），透传给模型"),
        DeclareLaunchArgument("start_supervisor", default_value="false",
                              description="true = 启动 avoid_supervisor（场景变化时"
                                          "取消/重规划；失败安全停靠）。验收 --replan 用"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
                              description="本栈的 ROS domain（刻意不是 0）"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = 仅本机发现"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    use_cloud = _parse_bool(resolve("use_synthetic_cloud"), False)
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    share = get_package_share_directory("litearm_obstacle_avoidance")

    fake_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "fake_control.launch.py")),
        launch_arguments={
            "ros_domain_id": domain_id,
            "ros_localhost_only": localhost_only,
        }.items())

    moveit_arguments = {
        # 唯一区别：MoveIt 也看到假硬件版本，控制块与真正运行的栈一致。
        "robot_description_file": os.path.join(
            share, "config", "litearm_litegrip_fake.urdf.xacro"),
        "use_rviz": resolve("use_rviz"),
        "scene_file": resolve("scene_file"),
        "gripper_envelope_margin": resolve("gripper_envelope_margin"),
        "ros_domain_id": domain_id,
        "ros_localhost_only": localhost_only,
    }
    if use_cloud:
        moveit_arguments["sensors_3d_file"] = os.path.join(share, "config", "sensors_3d.yaml")
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "obstacle_demo.launch.py")),
        launch_arguments=moveit_arguments.items())

    actions = [
        LogInfo(msg=(
            "──────── litearm 避障干跑栈（mock_components）────────\n"
            "  规划: move_group（组合模型 + 夹爪碰撞体）\n"
            "  执行: JTC -> mock 硬件（状态 = 指令，不会动机械臂）\n"
            f"  环境: {resolve('scene_file') or '未加载'}\n"
            f"  Octomap: {'合成点云 /synthetic_cloud' if use_cloud else '关闭'}\n"
            f"  Supervisor: {'已启动' if _parse_bool(resolve('start_supervisor'), False) else '未启动'}\n"
            "  验收: moveit_probe.py --execute（规划并执行到目标）\n"
            "  ⚠ 等 JTC 变为 active 后再下发目标\n"
            "────────────────────────────────────────────────")),
        fake_control,
        moveit,
    ]

    if _parse_bool(resolve("start_supervisor"), False):
        actions.append(Node(
            package="litearm_obstacle_avoidance",
            executable="avoid_supervisor.py",
            output="screen",
            additional_env=env,
        ))

    if use_cloud:
        actions.append(Node(
            package="litearm_obstacle_avoidance",
            executable="synthetic_cloud.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "point_cloud_topic": "/synthetic_cloud",
                "frame_id": "base_link",
                "rate_hz": 2.0,
            }],
        ))
    return actions


def generate_launch_description():
    default_scene_file = os.path.join(
        get_package_share_directory("litearm_obstacle_avoidance"),
        "config", "scene.yaml")
    return LaunchDescription(_declare_arguments(default_scene_file) +
                             [OpaqueFunction(function=_launch_setup)])
