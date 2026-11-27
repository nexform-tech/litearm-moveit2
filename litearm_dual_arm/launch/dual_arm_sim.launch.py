#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dual_arm_sim.launch.py — 双臂离线规划/确定性执行入口。

启动内容：

    robot_state_publisher   双臂 URDF（world + left_*/right_* 两套链）
    move_group              left_arm / right_arm / both_arms 三个组
    dual_arm_sim.py         两路 /left、/right 状态 + 两路 JTC action server
    dual_arm_bridge.py      状态合并、14→7+7 轨迹拆分、失败取消
    dual_arm_scene.py       共享静态工位
    rviz2                   双臂 MotionPlanning 面板
    可选 task               carry = 同步搬运；它自己管理场景并输出指标 JSON

``execute:=true``（默认）时，MoveIt 的轨迹经 bridge 发到确定性仿真的两路
action server；``execute:=false`` 退化为只规划。仿真没有物理引擎，只验证
执行链路、时序、同步和 fail-closed，不验证动力学。

    ros2 launch litearm_dual_arm dual_arm_sim.launch.py
    ros2 launch litearm_dual_arm dual_arm_sim.launch.py use_rviz:=false
    ros2 launch litearm_dual_arm dual_arm_sim.launch.py execute:=false
    ros2 launch litearm_dual_arm dual_arm_sim.launch.py scene:=none
    ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=carry
    ros2 launch litearm_dual_arm dual_arm_sim.launch.py task:=carry execute:=false
"""

from __future__ import annotations

import math
import os
import shlex
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_DOMAIN_ID = "42"
"""与单臂栈一致：刻意不用 domain 0。"""


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


def _declare_arguments():
    return [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="启动 RViz2 MotionPlanning 面板"),
        DeclareLaunchArgument("mounts_file", default_value="",
                              description="左右基座挂载 YAML；空 = 包内 "
                                          "config/dual_arm_mounts.yaml"),
        DeclareLaunchArgument("scene", default_value="dual_static",
                              description="要应用的场景名；none 表示不加载静态场景"),
        DeclareLaunchArgument("robot_padding_m", default_value="0.02",
                              description="MoveIt 机器人碰撞体统一膨胀（m）"),
        DeclareLaunchArgument("allowed_start_tolerance", default_value="0.05",
                              description="MoveIt 允许的轨迹起点关节差（rad）"),
        DeclareLaunchArgument("execute", default_value="true",
                              description="true = 启用确定性仿真执行（bridge + 两路 "
                                          "action server）；false = 只规划"),
        DeclareLaunchArgument("task", default_value="none",
                              choices=["none", "carry", "alternating"],
                              description="启动后运行的内置任务；carry = 同步搬运，"
                                          "alternating = 共享空间交替作业"),
        DeclareLaunchArgument("task_plan_only", default_value="false",
                              description="true = 任务只规划；false 时跟随 execute"),
        DeclareLaunchArgument(
            "task_output", default_value="",
            description="任务指标 JSON 输出路径；空 = 按 task 使用 /tmp 默认名"),
        DeclareLaunchArgument("task_timeout", default_value="60.0",
                              description="任务等待 MoveIt/bridge 的超时（s）"),
        DeclareLaunchArgument("bridge_start_tolerance", default_value="0.08",
                              description="bridge 允许的轨迹起点关节差（rad）"),
        DeclareLaunchArgument("bridge_max_joint_velocity", default_value="0.35",
                              description="bridge 允许的最大单关节速度（rad/s）"),
        DeclareLaunchArgument("bridge_max_state_age", default_value="0.5",
                              description="bridge 允许的关节状态最大年龄（s）"),
        DeclareLaunchArgument("bridge_max_state_skew", default_value="0.1",
                              description="bridge 允许的左右状态时间差（s）"),
        DeclareLaunchArgument("publish_rate", default_value="100.0",
                              description="仿真状态发布频率（Hz）"),
        DeclareLaunchArgument("initial_positions", default_value="",
                              description="14 个初始关节角，逗号分隔；空 = 全零"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz 配置；空 = 包内 rviz/dual_arm.rviz"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group 日志级别"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
                              description="本栈 ROS domain（刻意不是 0）"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = 仅本机发现"),
    ]


def _isolation_env(domain_id, localhost_only):
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _isolation_hint(domain_id, localhost_only):
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    localhost = "1" if _parse_bool(localhost_only, True) else "0"
    return f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY={localhost}"


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    use_rviz = _parse_bool(resolve("use_rviz"), True)
    scene = resolve("scene").strip()
    mounts_file = resolve("mounts_file").strip()
    try:
        robot_padding = float(resolve("robot_padding_m"))
    except ValueError as error:
        raise RuntimeError("robot_padding_m must be a number") from error
    if not math.isfinite(robot_padding) or robot_padding < 0.0:
        raise RuntimeError("robot_padding_m must be non-negative and finite")
    try:
        allowed_start_tolerance = float(resolve("allowed_start_tolerance"))
    except ValueError as error:
        raise RuntimeError("allowed_start_tolerance must be a number") from error
    if not math.isfinite(allowed_start_tolerance) or allowed_start_tolerance <= 0.0:
        raise RuntimeError("allowed_start_tolerance must be positive and finite")
    try:
        publish_rate = float(resolve("publish_rate"))
    except ValueError as error:
        raise RuntimeError("publish_rate must be a number") from error
    if not math.isfinite(publish_rate) or publish_rate <= 0.0:
        raise RuntimeError("publish_rate must be positive and finite")
    initial_positions = resolve("initial_positions").strip()
    execute = _parse_bool(resolve("execute"), True)
    task = resolve("task").strip().lower() or "none"
    task_plan_only = _parse_bool(resolve("task_plan_only"), False)
    task_output = resolve("task_output").strip()
    if not task_output:
        if task == "carry":
            task_output = "/tmp/litearm_dual_carry_metrics.json"
        elif task == "alternating":
            task_output = "/tmp/litearm_dual_alternating_metrics.json"
        else:
            task_output = ""
    try:
        task_timeout = float(resolve("task_timeout"))
    except ValueError as error:
        raise RuntimeError("task_timeout must be a number") from error
    if not math.isfinite(task_timeout) or task_timeout <= 0.0:
        raise RuntimeError("task_timeout must be positive and finite")
    try:
        bridge_start_tolerance = float(resolve("bridge_start_tolerance"))
    except ValueError as error:
        raise RuntimeError("bridge_start_tolerance must be a number") from error
    if not math.isfinite(bridge_start_tolerance) or bridge_start_tolerance <= 0.0:
        raise RuntimeError("bridge_start_tolerance must be positive and finite")
    try:
        bridge_max_joint_velocity = float(resolve("bridge_max_joint_velocity"))
    except ValueError as error:
        raise RuntimeError("bridge_max_joint_velocity must be a number") from error
    if (
        not math.isfinite(bridge_max_joint_velocity)
        or bridge_max_joint_velocity <= 0.0
    ):
        raise RuntimeError("bridge_max_joint_velocity must be positive and finite")
    try:
        bridge_max_state_age = float(resolve("bridge_max_state_age"))
    except ValueError as error:
        raise RuntimeError("bridge_max_state_age must be a number") from error
    if not math.isfinite(bridge_max_state_age) or bridge_max_state_age <= 0.0:
        raise RuntimeError("bridge_max_state_age must be positive and finite")
    try:
        bridge_max_state_skew = float(resolve("bridge_max_state_skew"))
    except ValueError as error:
        raise RuntimeError("bridge_max_state_skew must be a number") from error
    if not math.isfinite(bridge_max_state_skew) or bridge_max_state_skew <= 0.0:
        raise RuntimeError("bridge_max_state_skew must be positive and finite")

    dual_share = get_package_share_directory("litearm_dual_arm")
    moveit_share = get_package_share_directory("litearm_moveit_config")
    description_share = get_package_share_directory("litearm_description")
    prefix = get_package_prefix("litearm_dual_arm")
    executable_dir = os.path.join(prefix, "lib", "litearm_dual_arm")

    generator = os.path.join(executable_dir, "generate_dual_arm_model.py")
    source_urdf = os.path.join(description_share, "urdf", "litearm_description.urdf")
    source_srdf = os.path.join(moveit_share, "config", "litearm.srdf")
    mounts = mounts_file or os.path.join(dual_share, "config", "dual_arm_mounts.yaml")
    scene_config = os.path.join(dual_share, "config", "dual_scenes.yaml")
    carry_config = os.path.join(dual_share, "config", "dual_carry.yaml")
    alternating_config = os.path.join(
        dual_share, "config", "dual_alternating.yaml"
    )
    goals_config = os.path.join(dual_share, "config", "dual_goals.yaml")

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
        # execute:=false 时保留只规划行为；true 时由 bridge 接管执行。
        "allow_trajectory_execution": execute,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    # 控制器映射始终加载，避免 simple controller manager 在启动时报
    # "No controller_names specified"；是否真正下发轨迹由
    # allow_trajectory_execution 和 bridge 是否带 --state-only 共同决定。
    parameters.update(
        _load_yaml(os.path.join(dual_share, "config", "dual_controllers.yaml"))
    )

    actions = [
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
            executable=os.path.join(executable_dir, "dual_arm_sim.py"),
            name="litearm_dual_arm_sim",
            output="screen",
            additional_env=env,
            arguments=[
                "--left-state-topic",
                "/left/joint_states",
                "--right-state-topic",
                "/right/joint_states",
                "--left-action",
                "/left/joint_trajectory_controller/follow_joint_trajectory",
                "--right-action",
                "/right/joint_trajectory_controller/follow_joint_trajectory",
                "--rate",
                str(publish_rate),
            ]
            + (
                ["--initial-positions", initial_positions]
                if initial_positions
                else []
            )
            + ([] if execute else ["--publish-only"]),
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

    if task == "carry":
        task_arguments = [
            "--config",
            carry_config,
            "--scene-config",
            scene_config,
            "--output",
            task_output,
            "--timeout",
            str(task_timeout),
        ]
        if not execute or task_plan_only:
            task_arguments.append("--plan-only")
        actions.append(
            Node(
                executable=os.path.join(executable_dir, "dual_carry_demo.py"),
                name="litearm_dual_carry_demo",
                output="screen",
                additional_env=env,
                arguments=task_arguments,
            )
        )

    if task == "alternating":
        task_arguments = [
            "--config",
            alternating_config,
            "--goals",
            goals_config,
            "--scene-config",
            scene_config,
            "--output",
            task_output,
            "--timeout",
            str(task_timeout),
        ]
        if not execute or task_plan_only:
            task_arguments.append("--plan-only")
        actions.append(
            Node(
                executable=os.path.join(
                    executable_dir, "dual_alternating_demo.py"
                ),
                name="litearm_dual_alternating_demo",
                output="screen",
                additional_env=env,
                arguments=task_arguments,
            )
        )

    # 内置任务自带 scene 配置（carry/alternating 都用工作台场景）；此时不要
    # 再让通用场景节点铺一套 dual_static，否则两个场景会叠在同一份 Planning Scene。
    if task == "none" and scene and scene.lower() not in ("none", "off", "false"):
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
                    # move_group 首次加载网格/KDL/OMPL 可能要 30 s 以上；
                    # 场景节点先起、服务后到，给它足够的等待窗口。
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

    actions.insert(
        0,
        LogInfo(
            msg=(
                "──────── litearm dual arm (deterministic sim) ────────\n"
                "  groups: left_arm, right_arm, both_arms\n"
                "  model: world + left_*/right_* prefixed LiteArm chains\n"
                f"  mounts: {mounts}\n"
                f"  task: {task}\n"
                f"  scene: {('task-managed' if task != 'none' else (scene or 'none'))}\n"
                f"  robot collision padding: {robot_padding:.3f} m\n"
                f"  allowed start tolerance: {allowed_start_tolerance:.3f} rad\n"
                f"  execution: {'enabled (bridge + two action servers)' if execute else 'disabled (planning only)'}\n"
                "  bridge: /left_arm_controller + /right_arm_controller → "
                "/left, /right JTC actions\n"
                "  ⚠ This launch fixes the ROS domain. In your own terminal run:\n"
                f"        {_isolation_hint(domain_id, localhost_only)}\n"
                "──────────────────────────────────────────────────"
            )
        ),
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        _declare_arguments() + [OpaqueFunction(function=_launch_setup)]
    )
