#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_gazebo.launch.py — Gazebo Classic 物理仿真版避障栈。

    gazebo (gzserver [+gzclient])
      + robot_state_publisher（组合模型，含夹爪与保守包络）
      + spawn_entity（base_link 在原点，基座 kinematic）
      + gazebo_ros2_control -> controller_manager -> JSB + JTC
      + move_group（同一个组合模型 + SRDF + scene.yaml）
      + 可选 avoid_supervisor / 合成点云 / RViz

与干跑栈（obstacle_fake.launch.py）的区别只有一个：执行后端从
mock_components 换成 Gazebo 物理引擎。JTC action 名、节拍、探针全部一致，
所以 acceptance_fake.sh 的探针可以直接复用（--gazebo 模式）。

    # 有显示：带 Gazebo GUI + RViz
    ros2 launch litearm_obstacle_avoidance obstacle_gazebo.launch.py

    # 无显示：仅 gzserver（验收脚本用这个）
    ros2 launch litearm_obstacle_avoidance obstacle_gazebo.launch.py \
        gui:=false use_rviz:=false start_supervisor:=true

⚠ 世界几何必须与 config/scene.yaml 一致（工作台/夹具位姿），否则会出现
  “Gazebo 里有、MoveIt 规划里没有”的错位。改一处要同步另一处。
⚠ Gazebo Classic 不支持 URDF mimic：夹爪两指在 Gazebo 里是被动关节，
  避障依赖 MoveIt 侧的保守包络；本仿真验证机械臂，不验证夹爪夹取。
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

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


def _declare_arguments(default_scene_file, default_world):
    return [
        DeclareLaunchArgument("gui", default_value="true",
                              description="启动 Gazebo 客户端（无显示环境用 false）"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="启动 RViz2"),
        DeclareLaunchArgument("use_synthetic_cloud", default_value="false",
                              description="true = 合成点云 + Octomap"),
        DeclareLaunchArgument("scene_file", default_value=default_scene_file,
                              description="静态环境 scene.yaml；空 = 不加载"),
        DeclareLaunchArgument("start_supervisor", default_value="false",
                              description="启动 avoid_supervisor"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Gazebo world 文件"),
        DeclareLaunchArgument("gripper_envelope_margin", default_value="0.01",
                              description="夹爪包络外扩量（m）"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group 日志级别"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID),
        DeclareLaunchArgument("ros_localhost_only", default_value="true"),
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
    gui = _parse_bool(resolve("gui"), True)
    use_rviz = _parse_bool(resolve("use_rviz"), True)
    use_cloud = _parse_bool(resolve("use_synthetic_cloud"), False)
    start_supervisor = _parse_bool(resolve("start_supervisor"), False)
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    share = get_package_share_directory("litearm_obstacle_avoidance")
    litearm_share = get_package_share_directory("litearm_moveit_config")
    gazebo_share = get_package_share_directory("gazebo_ros")

    scene_file = resolve("scene_file").strip()
    world = resolve("world").strip() or os.path.join(
        share, "worlds", "obstacle_avoidance.world")
    xacro_file = os.path.join(share, "config", "litearm_litegrip_gazebo.urdf.xacro")
    # gazebo_ros2_control 把 robot_description 作为 `--param` 传给内部
    # controller_manager；URDF 注释里形如 "来源: ..." 的 "冒号+空格" 会让
    # rcl 的 YAML 参数覆盖解析器报 "Couldn't parse parameter override rule"。
    # 给 Gazebo 一侧（RSP -> spawn -> 插件）用去掉注释、去掉 XML 声明的
    # 单行版本；MoveIt 仍用完整的 xacro。
    raw = subprocess.check_output(["xacro", xacro_file], text=True)
    raw = re.sub(r"<!--.*?-->", "", raw, flags=re.S)
    raw = re.sub(r"<\?xml[^>]*\?>", "", raw)
    robot_description = " ".join(raw.split())

    # 先给整个 launch（包括 gzserver 里的 ROS 插件）设好发现配置
    environment = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", env["ROS_DOMAIN_ID"]),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", env["ROS_LOCALHOST_ONLY"]),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world, "gui": "true" if gui else "false"}.items())

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        additional_env=env,
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # 不用 gazebo_ros/spawn_entity.py：它的 env python 在 conda 环境下会缺 lxml。
    spawn = Node(
        package="litearm_obstacle_avoidance",
        executable="spawn_robot.py",
        output="screen",
        additional_env=env,
        parameters=[{"entity_name": "litearm", "z": 0.0}],
    )

    broadcaster = _spawner("joint_state_broadcaster", env)
    trajectory = _spawner("joint_trajectory_controller", env)

    # ── MoveIt：同一个组合模型 + SRDF + 规划参数 ──
    # MoveIt 用不带 world link 的组合模型（本文件会给 Gazebo 加固定基座的
    # world root，和 SRDF 的 world→base_link 虚拟关节重复）。
    moveit_xacro = os.path.join(share, "config", "litearm_litegrip.urdf.xacro")
    builder = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(
            file_path=moveit_xacro,
            mappings={"gripper_envelope_margin": resolve("gripper_envelope_margin")})
        .robot_description_semantic(
            file_path=os.path.join(share, "config", "litearm_litegrip.srdf"))
        .planning_pipelines(pipelines=["ompl"])
    )
    if use_cloud:
        builder = builder.sensors_3d(
            file_path=os.path.join(share, "config", "sensors_3d.yaml"))
    moveit_config = builder.to_moveit_configs()

    actions = list(environment)
    actions.append(LogInfo(msg=(
        "──────── litearm Gazebo 避障仿真 ────────\n"
        f"  world:      {world}\n"
        f"  scene:      {scene_file or '未加载'}\n"
        f"  Gazebo GUI: {gui}    RViz: {use_rviz}\n"
        f"  Octomap:    {'合成点云' if use_cloud else '关闭'}\n"
        f"  Supervisor: {start_supervisor}\n"
        "  ⚠ Gazebo 世界几何必须与 scene.yaml 一致\n"
        f"  ⚠ 终端里用 ros2 命令前先执行: export ROS_DOMAIN_ID={domain_id}; "
        f"export ROS_LOCALHOST_ONLY={'1' if _parse_bool(localhost_only, True) else '0'}\n"
        "────────────────────────────────────────")))
    actions += [gazebo, robot_state_publisher, spawn]
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=spawn, on_exit=[broadcaster])))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=broadcaster, on_exit=[trajectory])))

    actions.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=env,
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", resolve("log_level")],
    ))

    if scene_file:
        actions.append(Node(
            package="litearm_obstacle_avoidance",
            executable="scene_builder.py",
            output="screen",
            additional_env=env,
            parameters=[{"scene_file": os.path.abspath(scene_file)}],
        ))

    if start_supervisor:
        actions.append(Node(
            package="litearm_obstacle_avoidance",
            executable="avoid_supervisor.py",
            output="screen",
            additional_env=env,
            parameters=[{"use_sim_time": True}],
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
                "use_sim_time": True,
            }],
        ))

    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(litearm_share, "rviz", "moveit.rviz")],
            output="log",
            additional_env=env,
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": True},
            ],
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("litearm_obstacle_avoidance")
    return LaunchDescription(
        _declare_arguments(
            os.path.join(share, "config", "scene.yaml"),
            os.path.join(share, "worlds", "obstacle_avoidance.world"))
        + [OpaqueFunction(function=_launch_setup)])
