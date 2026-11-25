#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""d435i_avoidance.launch.py — Gazebo 中的 D435i 感知 + MoveIt 避障 + 动态重规划。

完整栈
======
    Gazebo Classic (gzserver [+gzclient])
      + robot_state_publisher（臂 + LiteGrip 组合模型）
      + spawn_robot (旧包) / spawn_model (本包)
      + D435i 相机模型（depth + color 两个传感器，libgazebo_ros_camera.so）
      + static TF: base_link -> camera_link -> *_optical_frame
      + gazebo_ros2_control -> controller_manager -> JSB + JTC
      + move_group（本包 sensors_3d.yaml：点云 -> Octomap）
      + movable_block（默认开启：可随时改位的静态物块）
      + dynamic_obstacle（可选：Gazebo 里自动往复的静态方块）
      + d435i_supervisor（规划/执行/剩余轨迹复检/重规划）
      + 可选 scene_builder（旧包 scene.yaml）与 RViz

    # 有显示：Gazebo GUI + RViz
    ros2 launch litearm_d435i_perception d435i_avoidance.launch.py

    # 无显示：只跑 gzserver（验收脚本用）
    ros2 launch litearm_d435i_perception d435i_avoidance.launch.py \
        gui:=false use_rviz:=false

    # 物块就位后，另开终端（同一 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY）改位置：
    ros2 run litearm_d435i_perception move_block.py 0.25 0.10 0.20
    ros2 run litearm_d435i_perception move_block.py --reset
    物块移动后 D435i 在新位置渲染它，/camera/depth/color/points 与
    Octomap 实时更新。

前置依赖
========
    sudo apt install ros-humble-gazebo-plugins
libgazebo_ros_camera.so 在这个包里；没装时 Gazebo 会报 plugin not found，
相机不会出点云，supervisor 会以 no_sensor/no_octomap 拒绝运动。

隔离
====
和旧包一致：默认 ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1。别的终端要
export 同样的值才能看到这套图。
"""

import math
import os
import re
import subprocess
import tempfile
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


def _parse_xyz(text, fallback):
    """解析 'x y z' / 'x,y,z'；非法时回退并保留 launch 可启动。"""
    try:
        values = [float(v) for v in re.split(r"[,\s]+", str(text).strip())
                  if v]
    except ValueError:
        return list(fallback)
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        return list(fallback)
    return values


def _isolation_env(domain_id, localhost_only):
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _resolve_package_uris(urdf_text):
    """把 Gazebo 侧 URDF 的 package:// 网格解析成 file:// 绝对路径。

    Gazebo Classic 不认 ROS 的 package://（gazebo_ros 的模型路径导出也不
    覆盖 litearm/litegrip 包）。不替换的话，Gazebo 会去
    models.gazebosim.org 找网格，世界加载能卡住几分钟，表现为相机/机器人
    都“插入了”但插件和 /clock 迟迟不出现。
    """
    for package in ("litearm_description", "litegrip_description",
                    "litearm_obstacle_avoidance", "litearm_d435i_perception"):
        try:
            share = get_package_share_directory(package)
        except Exception:  # noqa: BLE001 - 缺包时保留原 URI，让报错更直接
            continue
        urdf_text = urdf_text.replace(
            f"package://{package}/", f"file://{share}/")
    return urdf_text


def _add_missing_inertia(urdf_text):
    """给几何为空的关节子 link 补一个极小惯性。

    litegrip_gripper_opening_link 是“无几何 dummy”，URDF→SDF 时 sdformat 会把
    没有惯性的 link（连同挂在它上面的 gripper_opening_joint）整段丢掉。
    结果 Gazebo 里没有这个关节，joint_state_broadcaster 发不出它的状态，
    MoveIt 一直报 "Missing gripper_opening_joint"，规划无法开始。
    补 1 g / 1e-6 惯量只影响 SDF 保留，不影响运动学。
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


def _normalize_gripper_prefix(urdf_text):
    """把夹爪的未加前缀 link 改名为 litegrip_*。

    litearm_litegrip.geometry.urdf.xacro 声明了 link_prefix=litegrip_，但它
    include litegrip_description 时没有把 arg 传进去；被包含文件自己的
    <xacro:arg name="link_prefix" default=""/> 会把值重置成空。结果夹爪的
    base_link 与机械臂 base_link 重名，URDF 解析直接报
    "link 'base_link' is not unique"，Gazebo/MoveIt 都起不来。

    这里做一次确定性的后处理：只改夹爪的 link（joint 名是控制器契约，保持
    不变），并同步 joint 的 parent/child 引用。已带前缀时函数是幂等的。
    """
    root = ET.fromstring(urdf_text)
    links = {link.get("name"): link for link in root.findall("link")}
    if "base_footprint" not in links:
        return urdf_text

    rename = {
        "base_footprint": "litegrip_base_footprint",
        "gripper_opening_link": "litegrip_gripper_opening_link",
        "gripper_slider_link1": "litegrip_gripper_slider_link1",
        "gripper_slider_link2": "litegrip_gripper_slider_link2",
    }

    # 夹爪 base_link 是 base_footprint 之后的第一个 base_link；机械臂的
    # base_link 在 base_footprint 之前。不要用“最后一个”这种顺序假设。
    link_elements = root.findall("link")
    footprint_index = next(
        (i for i, link in enumerate(link_elements)
         if link.get("name") == "base_footprint"), None)
    gripper_base_link = None
    if footprint_index is not None:
        for link in link_elements[footprint_index + 1:]:
            if link.get("name") == "base_link":
                gripper_base_link = link
                break

    for link in root.findall("link"):
        name = link.get("name")
        if name in ("base_footprint", "gripper_opening_link",
                    "gripper_slider_link1", "gripper_slider_link2"):
            link.set("name", rename[name])

    if gripper_base_link is not None:
        gripper_base_link.set("name", "litegrip_base_link")
        rename["base_link"] = "litegrip_base_link"

    gripper_joints = {
        "base_footprint_joint", "gripper_opening_joint",
        "gripper_slide_joint_right", "gripper_slide_joint_left",
    }
    for joint in root.findall("joint"):
        if joint.get("name") not in gripper_joints:
            continue
        for tag in ("parent", "child"):
            ref = joint.find(tag)
            if ref is not None and ref.get("link") in rename:
                ref.set("link", rename[ref.get("link")])

    for elem in root.iter("gazebo"):
        ref = elem.get("reference")
        if ref in rename:
            elem.set("reference", rename[ref])
    return ET.tostring(root, encoding="unicode")


def _rename_gazebo_links(urdf_text):
    """只给 Gazebo 侧 URDF 的冲突 link 加前缀。

    litearm 描述里 link 和 joint 都叫 joint1..joint7。sdformat9 从 1.7 起
    把 link/joint/frame 放在同一个命名空间里检查，名字相同就报
    "Joint ... has a name collision" 并拒绝加载模型，gazebo_ros2_control
    因而不会启动。MoveIt 侧模型不能动（SRDF、碰撞矩阵、控制器都用这些
    名字），所以只在送进 Gazebo 的那份 URDF 上把冲突 link 改成 gz_<name>，
    同步更新 joint 的 parent/child 和 <gazebo reference>。
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

    # gazebo_ros2_control 从独立的参数节点读取 Gazebo 模型；TF 由
    # robot_state_publisher 用 MoveIt 模型（原始 link 名）发布。
    for plugin in root.findall(".//plugin"):
        if plugin.get("name") != "gazebo_ros2_control":
            continue
        param_node = plugin.find("robot_param_node")
        if param_node is None:
            param_node = ET.SubElement(plugin, "robot_param_node")
        param_node.text = "/gazebo_robot_description"
    return ET.tostring(root, encoding="unicode")


def _spawner(name, env):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
        output="screen",
        additional_env=env,
        emulate_tty=True,
    )


def _declare_arguments(share, old_share):
    return [
        DeclareLaunchArgument("gui", default_value="true",
                              description="启动 Gazebo 客户端（无显示用 false）"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="启动 RViz2"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                share, "worlds", "d435i_workcell.world"),
            description="Gazebo world；默认使用带 gazebo_ros_state 的工位"),
        DeclareLaunchArgument(
            "scene_file", default_value="",
            description="可选：旧包 scene.yaml 静态碰撞对象；空 = 只用相机 "
                        "Octomap 建环境"),
        DeclareLaunchArgument(
            "camera_mount_file",
            default_value=os.path.join(share, "config", "camera_mount.yaml"),
            description="相机安装位姿 YAML"),
        DeclareLaunchArgument("start_camera", default_value="true",
                              description="插入并发布 Gazebo D435i；接真机时设 false"),
        DeclareLaunchArgument(
            "sensor_heartbeat_topic", default_value="/camera/color/camera_info",
            description="supervisor 判传感器存活用的小消息话题；真机可改为 "
                        "对应的 CameraInfo 话题"),
        DeclareLaunchArgument("start_supervisor", default_value="true",
                              description="启动 d435i_supervisor"),
        DeclareLaunchArgument(
            "start_movable_block", default_value="true",
            description="插入可通过 ROS 接口实时改位的演示物块（默认开启）"),
        DeclareLaunchArgument(
            "movable_block_pose", default_value="0.30 0.55 -0.06",
            description="物块初始位置 x y z（world/base_link，m）；默认在"
                        "工位外的地面上，相机可见但不挡机械臂"),
        DeclareLaunchArgument(
            "movable_block_yaw", default_value="0.0",
            description="物块初始绕 Z 轴朝向（rad）"),
        DeclareLaunchArgument("start_dynamic_obstacle", default_value="false",
                              description="默认关闭；true = 插入并驱动 Gazebo 动态方块"),
        DeclareLaunchArgument("dynamic_mode", default_value="sweep",
                              description="sweep = 往复；once = 走一遍就停"),
        DeclareLaunchArgument("dynamic_speed", default_value="0.20",
                              description="动态障碍速度 m/s"),
        DeclareLaunchArgument("gripper_envelope_margin", default_value="0.01",
                              description="夹爪包络外扩量（m）"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument(
            "extra_gazebo_args", default_value="",
            description="追加给 gzserver 的参数；默认 world 已用 WorldPlugin "
                        "加载 libgazebo_ros_state.so"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID),
        DeclareLaunchArgument("ros_localhost_only", default_value="true"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    gui = _parse_bool(resolve("gui"), True)
    use_rviz = _parse_bool(resolve("use_rviz"), True)
    start_supervisor = _parse_bool(resolve("start_supervisor"), True)
    start_movable = _parse_bool(resolve("start_movable_block"), True)
    movable_pose = _parse_xyz(resolve("movable_block_pose"),
                              [0.30, 0.55, -0.06])
    try:
        movable_yaw = float(resolve("movable_block_yaw"))
    except ValueError:
        movable_yaw = 0.0
    if not math.isfinite(movable_yaw):
        movable_yaw = 0.0
    start_dynamic = _parse_bool(resolve("start_dynamic_obstacle"), False)
    start_camera = _parse_bool(resolve("start_camera"), True)
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    share = get_package_share_directory("litearm_d435i_perception")
    old_share = get_package_share_directory("litearm_obstacle_avoidance")
    litearm_share = get_package_share_directory("litearm_moveit_config")

    world = resolve("world").strip() or os.path.join(
        share, "worlds", "d435i_workcell.world")
    scene_file = resolve("scene_file").strip()
    mount_file = resolve("camera_mount_file").strip() or os.path.join(
        share, "config", "camera_mount.yaml")
    extra_gazebo_args = resolve("extra_gazebo_args").strip()
    dynamic_mode = resolve("dynamic_mode").strip() or "sweep"

    # Gazebo 侧的机器人描述：组合 URDF + 固定基座 world link。
    # 注释必须先去掉：ros2_control 的 --param 解析器会把 "冒号+空格" 当参数
    # 覆盖规则。之后把与 joint 同名的 link 重命名，绕开 sdformat9 的 frame
    # 命名空间冲突。
    gazebo_xacro = os.path.join(
        old_share, "config", "litearm_litegrip_gazebo.urdf.xacro")
    raw = subprocess.check_output(
        ["xacro", gazebo_xacro,
         f"gripper_envelope_margin:={resolve('gripper_envelope_margin')}"],
        text=True)
    raw = re.sub(r"<!--.*?-->", "", raw, flags=re.S)
    raw = re.sub(r"<\?xml[^>]*\?>", "", raw)
    raw = _normalize_gripper_prefix(raw)
    raw = _resolve_package_uris(raw)
    gazebo_description = _add_missing_inertia(_rename_gazebo_links(raw))
    robot_description = " ".join(gazebo_description.split())

    # spawn_model.py 从文件读 URDF；gazebo_robot_description.py 再把同一份
    # 描述作为参数暴露给 gazebo_ros2_control（见脚本头注释）。
    gazebo_urdf_fd, gazebo_urdf_path = tempfile.mkstemp(
        prefix="litearm_gazebo_", suffix=".urdf")
    with os.fdopen(gazebo_urdf_fd, "w", encoding="utf-8") as handle:
        handle.write(gazebo_description)

    # MoveIt 侧的机器人描述：不带 world link（SRDF 有 world->base_link）。
    moveit_xacro = os.path.join(
        old_share, "config", "litearm_litegrip.urdf.xacro")
    builder = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(
            file_path=moveit_xacro,
            mappings={"gripper_envelope_margin":
                      resolve("gripper_envelope_margin")})
        .robot_description_semantic(
            file_path=os.path.join(old_share, "config",
                                   "litearm_litegrip.srdf"))
        .planning_pipelines(pipelines=["ompl"])
        .sensors_3d(file_path=os.path.join(share, "config",
                                           "sensors_3d.yaml"))
    )
    moveit_config = builder.to_moveit_configs()

    # TF 必须用 MoveIt 模型（原始 link 名 joint1..）。Octomap 的 ShapeMask
    # 靠 TF 取机器人各 link 的位姿做自滤波；用 Gazebo 侧 gz_joint* 的 TF
    # 会报 “Missing transform for shape”，机器人点云会被当成障碍物。
    rsp_description = subprocess.check_output(
        ["xacro", moveit_xacro,
         f"gripper_envelope_margin:={resolve('gripper_envelope_margin')}"],
        text=True)
    rsp_description = _normalize_gripper_prefix(rsp_description)
    rsp_description = " ".join(rsp_description.split())

    # builder 展开 xacro 时同样会带上未加前缀的夹爪 link；统一用后处理后的
    # 描述，保证 robot_state_publisher / move_group / RViz 看到同一份模型。
    moveit_config.robot_description = {
        "robot_description": rsp_description}

    environment = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", env["ROS_DOMAIN_ID"]),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", env["ROS_LOCALHOST_ONLY"]),
    ]

    # 不用 gazebo_ros/gzserver.launch.py：它会把 GAZEBO_PLUGIN_PATH 重设成
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
    gzserver_cmd = [
        "gzserver", world,
        "-s", "libgazebo_ros_init.so",
        "-s", "libgazebo_ros_factory.so",
        "-s", "libgazebo_ros_force_system.so",
    ]
    if extra_gazebo_args:
        gzserver_cmd += extra_gazebo_args.split()

    actions = list(environment) + [ExecuteProcess(
        cmd=gzserver_cmd, output="screen", additional_env=gazebo_env)]
    if gui:
        actions.append(ExecuteProcess(
            cmd=["gzclient"], output="screen", additional_env=gazebo_env))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        additional_env=env,
        parameters=[{"robot_description": rsp_description,
                     "use_sim_time": True}],
    )
    gazebo_description_node = Node(
        package="litearm_d435i_perception",
        executable="gazebo_robot_description.py",
        output="screen",
        additional_env=env,
        parameters=[{"robot_description": robot_description}],
    )
    # 不用旧包的 spawn_robot.py：它读 /robot_description，而 Gazebo 侧需要
    # 的是重命名后的 URDF；直接用 spawn_model.py 从临时文件插入。
    spawn_robot = Node(
        package="litearm_d435i_perception",
        executable="spawn_model.py",
        output="screen",
        additional_env=env,
        parameters=[{
            "entity_name": "litearm",
            "model_file": gazebo_urdf_path,
            "z": 0.0,
        }],
    )
    broadcaster = _spawner("joint_state_broadcaster", env)
    trajectory = _spawner("joint_trajectory_controller", env)

    camera_spawn = Node(
        package="litearm_d435i_perception",
        executable="spawn_model.py",
        output="screen",
        additional_env=env,
        parameters=[{
            "entity_name": "d435i",
            "model_file": os.path.join(share, "models", "d435i_camera.sdf"),
            "mount_file": mount_file,
        }],
    )
    camera_tf = Node(
        package="litearm_d435i_perception",
        executable="camera_tf.py",
        output="screen",
        additional_env=env,
        parameters=[{"mount_file": mount_file, "use_sim_time": True}],
    )

    # 相机/障碍先插入：gazebo_ros2_control 在读取 robot_description 时会
    # 阻塞模型插入队列，机器人放最后，避免后面的 spawn 请求被拖住。
    if start_camera:
        actions += [camera_spawn, camera_tf]

    # 可改位物块：同样先于机器人插入；spawn_model 退出后再起控制节点，
    # 避免对还不存在的实体调用 set_entity_state。节点启动后会重试，
    # 所以即使 Gazebo 稍慢也不会丢掉初始位姿。
    if start_movable:
        block_spawn = Node(
            package="litearm_d435i_perception",
            executable="spawn_model.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "entity_name": "movable_block",
                "model_file": os.path.join(
                    share, "models", "movable_block.sdf"),
                "x": movable_pose[0],
                "y": movable_pose[1],
                "z": movable_pose[2],
                "yaw": movable_yaw,
            }],
        )
        block_node = Node(
            package="litearm_d435i_perception",
            executable="movable_block.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "entity_name": "movable_block",
                "initial_pose": movable_pose,
                "initial_yaw": movable_yaw,
                "use_sim_time": True,
            }],
        )
        actions.append(block_spawn)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=block_spawn, on_exit=[block_node])))

    if start_dynamic:
        obstacle_spawn = Node(
            package="litearm_d435i_perception",
            executable="spawn_model.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "entity_name": "dynamic_obstacle",
                "model_file": os.path.join(
                    share, "models", "dynamic_obstacle.sdf"),
                "x": 0.25, "y": -0.55, "z": 0.25,
            }],
        )
        dynamic_node = Node(
            package="litearm_d435i_perception",
            executable="dynamic_obstacle.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "mode": dynamic_mode,
                "speed": float(resolve("dynamic_speed")),
                "park_pose": [0.25, -0.55, 0.25],
                "sweep_start": [0.25, -0.35, 0.25],
                "sweep_end": [0.25, 0.35, 0.25],
                "use_sim_time": True,
            }],
        )
        actions.append(obstacle_spawn)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=obstacle_spawn, on_exit=[dynamic_node])))

    # 机器人最后插入：gazebo_ros2_control 会在模型插入线程里等待
    # gazebo_robot_description 节点的参数，期间的 spawn 请求会排队，
    # 所以相机与障碍必须先就位。
    actions += [gazebo_description_node, robot_state_publisher, spawn_robot]
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[broadcaster])))
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
            package="litearm_d435i_perception",
            executable="d435i_supervisor.py",
            output="screen",
            additional_env=env,
            parameters=[{
                "use_sim_time": True,
                "sensor_topic": resolve("sensor_heartbeat_topic"),
                "require_octomap": True,
                "velocity_scaling": 0.12,
                "acceleration_scaling": 0.12,
                "max_sensor_age": 2.5,
                "validity_check_period": 0.8,
                "validity_stride": 1,
                "planning_time": 4.0,
                "plan_attempts": 8,
                "max_replans": 8,
            }],
        ))

    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "d435i.rviz")],
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

    actions.append(LogInfo(msg=(
        "──────── litearm D435i 感知避障仿真 ────────\n"
        f"  world:       {world}\n"
        f"  scene:       {scene_file or '未加载（只用相机 Octomap）'}\n"
        f"  相机模型:     {os.path.join(share, 'models', 'd435i_camera.sdf')}\n"
        f"  安装位姿:     {mount_file}\n"
        f"  点云话题:     /camera/depth/color/points\n"
        f"  Gazebo GUI:  {gui}    RViz: {use_rviz}\n"
        f"  Supervisor:  {start_supervisor}   动态障碍: {start_dynamic} "
        f"({dynamic_mode})\n"
        f"  可移动物块:  "
        f"{movable_pose if start_movable else '未加载'}\n"
        f"  Gazebo 相机: {start_camera}\n"
        f"  ⚠ 需要 ros-humble-gazebo-plugins（libgazebo_ros_camera.so）\n"
        f"  ⚠ 终端里用 ros2 命令前先执行: export ROS_DOMAIN_ID={domain_id}; "
        f"export ROS_LOCALHOST_ONLY="
        f"{'1' if _parse_bool(localhost_only, True) else '0'}\n"
        "  移动物块: ros2 run litearm_d435i_perception move_block.py "
        "0.25 0.10 0.20\n"
        "  验收: ros2 run litearm_d435i_perception d435i_probe.py\n"
        "────────────────────────────────────────")))
    return actions


def generate_launch_description():
    share = get_package_share_directory("litearm_d435i_perception")
    old_share = get_package_share_directory("litearm_obstacle_avoidance")
    return LaunchDescription(
        _declare_arguments(share, old_share)
        + [OpaqueFunction(function=_launch_setup)])
