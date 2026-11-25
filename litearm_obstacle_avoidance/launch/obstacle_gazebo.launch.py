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
所以包内的探针可以手动对本栈复用（acceptance_fake.sh 本身只起干跑栈）。

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
import xml.etree.ElementTree as ET

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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


def _resolve_package_uris(urdf_text):
    """把 Gazebo 侧 URDF 的 package:// 网格解析成 file:// 绝对路径。

    Gazebo Classic 不认 ROS 的 package://；不替换时 gzserver 会去
    models.gazebosim.org 找网格，模型插入会卡住，表现为实体“已插入”但
    gazebo_ros2_control 插件和 /clock 迟迟不出现。
    """
    for package in ("litearm_description", "litegrip_description",
                    "litearm_obstacle_avoidance"):
        try:
            share = get_package_share_directory(package)
        except Exception:  # noqa: BLE001 - 缺包时保留原 URI
            continue
        urdf_text = urdf_text.replace(
            f"package://{package}/", f"file://{share}/")
    return urdf_text


def _add_missing_inertia(urdf_text):
    """给几何为空的关节子 link 补一个极小惯性。

    litegrip_gripper_opening_link 没有几何/惯性，URDF→SDF 时 sdformat 会把
    没有惯性的 link（连同挂在它上面的 gripper_opening_joint）整段丢掉，
    joint_state_broadcaster 就发不出这个关节的状态。
    """
    root = ET.fromstring(urdf_text)
    child_links = set()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            child_links.add(child.get("link"))
    for link in root.findall("link"):
        name = link.get("name")
        if name in (None, "world") or name not in child_links:
            continue
        if link.find("inertial") is not None:
            continue
        inertial = ET.SubElement(link, "inertial")
        mass = ET.SubElement(inertial, "mass")
        mass.set("value", "0.001")
        inertia = ET.SubElement(inertial, "inertia")
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            inertia.set(key, "1e-6")
    return ET.tostring(root, encoding="unicode")


def _rename_gazebo_links(urdf_text):
    """只给 Gazebo 侧 URDF 的冲突 link 加前缀。

    litearm 描述里 link 和 joint 都叫 joint1..joint7。sdformat9 从 1.7 起
    把 link/joint/frame 放在同一个命名空间里检查，名字相同就报
    "Joint ... has a name collision" 并拒绝加载模型，gazebo_ros2_control
    因而不会启动。MoveIt 侧模型不能动（SRDF、碰撞矩阵、控制器都用这些
    名字），所以只在送进 Gazebo 的那份 URDF 上把冲突 link 改成 gz_<name>，
    同步更新 joint 的 parent/child 和 <gazebo reference>。

    gazebo_ros2_control 还需要一个持有 Gazebo 模型的参数节点；这里把
    plugin 的 <robot_param_node> 指到 /gazebo_robot_description，TF 由
    robot_state_publisher 用 MoveIt 模型（原始 link 名）发布。
    """
    root = ET.fromstring(urdf_text)
    link_names = {elem.get("name") for elem in root.findall("link")}
    joint_names = {elem.get("name") for elem in root.findall("joint")}
    rename = {name: f"gz_{name}"
              for name in link_names & joint_names if name}
    if not rename:
        return urdf_text

    for elem in root.findall("link"):
        name = elem.get("name")
        if name in rename:
            elem.set("name", rename[name])
    for elem in root.findall("joint"):
        for tag in ("parent", "child"):
            ref = elem.find(tag)
            if ref is not None and ref.get("link") in rename:
                ref.set("link", rename[ref.get("link")])
    for elem in root.iter("gazebo"):
        ref = elem.get("reference")
        if ref in rename:
            elem.set("reference", rename[ref])

    for plugin in root.findall(".//plugin"):
        if plugin.get("name") != "gazebo_ros2_control":
            continue
        param_node = plugin.find("robot_param_node")
        if param_node is None:
            param_node = ET.SubElement(plugin, "robot_param_node")
        param_node.text = "/gazebo_robot_description"
    return ET.tostring(root, encoding="unicode")


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
    # Gazebo side: resolve package:// meshes to file:// URIs, rename colliding
    # links to gz_jointN.., and keep the inertia-less gripper dummy link.
    gazebo_description = _add_missing_inertia(_rename_gazebo_links(
        _resolve_package_uris(" ".join(raw.split()))))

    # 先给整个 launch（包括 gzserver 里的 ROS 插件）设好发现配置
    environment = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", env["ROS_DOMAIN_ID"]),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", env["ROS_LOCALHOST_ONLY"]),
    ]

    # 不用 gazebo_ros/gazebo.launch.py：它会把 GAZEBO_PLUGIN_PATH 重设成
    # “包导出的路径 + 当前 shell 的 GAZEBO_PLUGIN_PATH”，而 ROS 的 setup.bash
    # 并不设置这个变量。结果 libgazebo_ros2_control.so 虽然就在
    # /opt/ros/humble/lib，却不在搜索路径里，插件静默不加载，
    # controller_manager 永远不会出现。这里直接起 gzserver，显式给全路径。
    gazebo_dirs = []
    try:
        gazebo_dirs.append(os.path.join(
            get_package_prefix("gazebo_ros"), "lib"))
    except Exception:  # noqa: BLE001
        pass
    gazebo_dirs += [
        "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
        "/usr/lib/aarch64-linux-gnu/gazebo-11/plugins",
        "/usr/lib/gazebo-11/plugins",
    ]

    def _merged_path(name, extras):
        parts = []
        for path in list(extras) + os.environ.get(name, "").split(os.pathsep):
            path = os.path.normpath(path) if path else ""
            if path and os.path.isdir(path) and path not in parts:
                parts.append(path)
        return os.pathsep.join(parts)

    gazebo_env = {
        "GAZEBO_PLUGIN_PATH": _merged_path("GAZEBO_PLUGIN_PATH", gazebo_dirs),
        "LD_LIBRARY_PATH": _merged_path("LD_LIBRARY_PATH", gazebo_dirs),
    }
    gazebo = ExecuteProcess(
        cmd=[
            "gzserver", world,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_force_system.so",
        ],
        output="screen",
        additional_env=gazebo_env,
    )
    gzclient = ExecuteProcess(
        cmd=["gzclient"], output="screen", additional_env=gazebo_env)

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

    # TF 必须用 MoveIt 模型（原始 link 名 joint1..）。Gazebo 侧模型
    # （gz_joint1..）由 gazebo_robot_description 提供给 gazebo_ros2_control；
    # 两者不能共用一个 robot_state_publisher。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        additional_env=env,
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )
    gazebo_description_node = Node(
        package="litearm_obstacle_avoidance",
        executable="gazebo_robot_description.py",
        output="screen",
        additional_env=env,
        parameters=[{"robot_description": gazebo_description,
                     "use_sim_time": True}],
    )

    # 不用 gazebo_ros/spawn_entity.py：它的 env python 在 conda 环境下会缺 lxml。
    # robot_xml 用 Gazebo 侧模型，不再依赖 /robot_description 话题。
    spawn = Node(
        package="litearm_obstacle_avoidance",
        executable="spawn_robot.py",
        output="screen",
        additional_env=env,
        parameters=[{"entity_name": "litearm", "z": 0.0,
                     "robot_xml": gazebo_description}],
    )

    broadcaster = _spawner("joint_state_broadcaster", env)
    trajectory = _spawner("joint_trajectory_controller", env)

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
    actions += [gazebo]
    if gui:
        actions.append(gzclient)
    # 机器人最后插入：gazebo_ros2_control 会在模型插入线程里等待
    # gazebo_robot_description 节点的参数，期间的 spawn 请求会排队。
    actions += [gazebo_description_node, robot_state_publisher, spawn]
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
