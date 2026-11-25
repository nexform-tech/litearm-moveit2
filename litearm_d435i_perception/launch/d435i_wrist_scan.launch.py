#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""d435i_wrist_scan.launch.py — 腕装 D435i 多视角扫描 + MoveIt 避障。

和固定外置相机的 d435i_avoidance.launch.py 并存，互不影响：

    d435i_avoidance.launch.py    固定外置 D435i（单视角，原样保留）
    d435i_wrist_scan.launch.py   腕装 D435i + 多视角扫描（本文件）

腕装相机不是独立的 Gazebo 模型，而是机器人 URDF 里挂在 ee_link 上的固定
link（config/wrist_camera.xacro）。这样 robot_state_publisher 会发布
base_link -> ... -> camera_link -> camera_*_optical_frame 的动态 TF，
PointCloudOctomapUpdater 的 ShapeMask 也把相机当机器人自身的一部分。

启动后 wrist_scan.py：
  1. 等 MoveIt / IK / JTC / 相机 / Octomap 就绪；
  2. 顶视 + 55° 环视生成候选视角（可配 elevation/azimuth/radius）；
  3. 逐视角 IK -> 规划 -> 执行 -> 停稳 dwell；
  4. 点云持续融进同一张 Octomap（frame=base_link），并统计覆盖率；
  5. 覆盖率达到目标或视角跑完 -> 回 ready。

    # 有显示
    ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py

    # 无显示（验收）
    ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py \
        gui:=false use_rviz:=false

    # 只验证视角可达性，不动机械臂
    ros2 launch litearm_d435i_perception d435i_wrist_scan.launch.py \
        gui:=false use_rviz:=false scan_dry_run:=true

手动触发 / 取消（auto_start:=false 时用）：
    ros2 service call /wrist_scan/start std_srvs/srv/Trigger
    ros2 service call /wrist_scan/cancel std_srvs/srv/Trigger
    ros2 topic echo /wrist_scan/status

分离域沿用旧约定：默认 ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1。
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

CAMERA_DISABLE_COLLISIONS = (
    ("camera_link", "ee_link"),
    ("camera_link", "joint7"),
    ("camera_link", "litegrip_base_footprint"),
    ("camera_link", "litegrip_base_link"),
    ("camera_link", "litegrip_gripper_slider_link1"),
    ("camera_link", "litegrip_gripper_slider_link2"),
    ("camera_link", "litegrip_finger_envelope_r"),
    ("camera_link", "litegrip_finger_envelope_l"),
)


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
    for package in ("litearm_description", "litegrip_description",
                    "litearm_obstacle_avoidance", "litearm_d435i_perception"):
        try:
            share = get_package_share_directory(package)
        except Exception:  # noqa: BLE001
            continue
        urdf_text = urdf_text.replace(
            f"package://{package}/", f"file://{share}/")
    return urdf_text


def _add_missing_inertia(urdf_text):
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


def _wrist_srdf_text(source_path):
    """在旧 SRDF 末尾补腕装相机的结构碰撞对。

    相机经固定关节刚性挂在 ee_link 上，和夹爪/端盖/包络的相对位姿永不改变，
    这些对不需要参与动态避障；不补的话 MoveIt 可能把相机与夹爪的轻微穿插
    判成自碰撞，表现为起始状态无效、所有规划失败。
    """
    with open(source_path, "r", encoding="utf-8") as handle:
        text = handle.read()
    entries = "\n".join(
        f'  <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>'
        for a, b in CAMERA_DISABLE_COLLISIONS)
    additions = (
        "\n  <!-- 腕装 D435i：固定连接导致的刚性接触对 -->\n"
        f"{entries}\n\n</robot>")
    if "</robot>" not in text:
        raise ValueError(f"{source_path}: 缺少 </robot>")
    return text.replace("</robot>", additions, 1)


def _spawner(name, env):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
        output="screen",
        additional_env=env,
        emulate_tty=True,
    )


def _declare_arguments(share):
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
            description="可选：额外加载的已知静态碰撞对象。默认为空——"
                        "工位由固定外置相机 + 腕装相机共同观测，不再预置"
                        "已知工位"),
        DeclareLaunchArgument(
            "start_fixed_camera", default_value="true",
            description="启动固定外置 D435i，与腕装相机一起喂同一张 Octomap；"
                        "接真机或不需要全局视野时设 false"),
        DeclareLaunchArgument(
            "fixed_camera_mount_file",
            default_value=os.path.join(
                share, "config", "fixed_camera_mount.yaml"),
            description="固定外置相机的安装位姿 YAML（使用 fixed_camera_* 帧）"),
        DeclareLaunchArgument(
            "camera_parent_link", default_value="ee_link",
            description="腕装相机挂点（机器人 link）"),
        DeclareLaunchArgument(
            "camera_mount_xyz", default_value="0.0 0.075 0.02",
            description="相机相对挂点的安装位置 x y z（m）"),
        DeclareLaunchArgument(
            "camera_mount_rpy", default_value="0 0 0",
            description="相机相对挂点的安装姿态（rad，固定轴 ZYX）；默认侧视，"
                        "工具轴朝下的安装方式在本臂扫描高度上 IK 不可达"),
        DeclareLaunchArgument(
            "sensor_heartbeat_topic", default_value="/camera/color/camera_info",
            description="监督/扫描节点判传感器存活用的小消息话题"),
        DeclareLaunchArgument("start_supervisor", default_value="true",
                              description="启动 d435i_supervisor"),
        DeclareLaunchArgument(
            "start_movable_block", default_value="true",
            description="插入可通过 ROS 接口实时改位的演示物块"),
        DeclareLaunchArgument(
            "movable_block_pose", default_value="0.30 0.55 -0.06",
            description="物块初始位置 x y z（world/base_link，m）"),
        DeclareLaunchArgument(
            "movable_block_yaw", default_value="0.0",
            description="物块初始绕 Z 轴朝向（rad）"),
        DeclareLaunchArgument("start_dynamic_obstacle", default_value="false",
                              description="默认关闭；true = 插入并驱动动态方块"),
        DeclareLaunchArgument("dynamic_mode", default_value="sweep",
                              description="sweep = 往复；once = 走一遍就停"),
        DeclareLaunchArgument("dynamic_speed", default_value="0.20",
                              description="动态障碍速度 m/s"),
        DeclareLaunchArgument("gripper_envelope_margin", default_value="0.01",
                              description="夹爪包络外扩量（m）"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument(
            "extra_gazebo_args", default_value="",
            description="追加给 gzserver 的参数"),

        # ── 扫描参数 ──
        DeclareLaunchArgument(
            "scan_auto_start", default_value="true",
            description="栈就绪后自动开始一轮扫描"),
        DeclareLaunchArgument(
            "scan_start_delay", default_value="8.0",
            description="自动扫描前的等待秒数"),
        DeclareLaunchArgument(
            "scan_targets", default_value="0.10,0.0,0.10",
            description="注视点列表，分号分隔；每个注视点生成一组环视"),
        DeclareLaunchArgument("scan_radius", default_value="0.40",
                              description="相机到注视点距离（m）"),
        DeclareLaunchArgument(
            "scan_elevations", default_value="90,55",
            description="仰角列表（deg）；90=顶视，55=环视"),
        DeclareLaunchArgument("scan_azimuths", default_value="8",
                              description="环视方位角数量（每圈）"),
        DeclareLaunchArgument(
            "scan_low_max_azimuth", default_value="90",
            description="仰角低于 45° 时只保留 ±该角度的方位"),
        DeclareLaunchArgument("scan_dwell", default_value="0.60",
                              description="每个视角停稳采集秒数"),
        DeclareLaunchArgument("scan_velocity_scaling", default_value="0.20"),
        DeclareLaunchArgument("scan_acceleration_scaling", default_value="0.20"),
        DeclareLaunchArgument("scan_planning_time", default_value="6.0"),
        DeclareLaunchArgument("scan_plan_attempts", default_value="12"),
        DeclareLaunchArgument("scan_ik_seeds", default_value="7",
                              description="每个视角最多尝试的 IK 种子数"),
        DeclareLaunchArgument(
            "scan_clear_octomap", default_value="false",
            description="扫描前清空 Octomap；默认 false：相机从启动起就在建图，"
                        "清空后桌子会在扫描过程中重新长出来，后续远侧视角可能"
                        "因地图变化而规划失败。需要干净地图时再设 true"),
        DeclareLaunchArgument("scan_coverage_target", default_value="0.90",
                              description="覆盖率达到后提前结束"),
        DeclareLaunchArgument("scan_coverage_resolution", default_value="0.05",
                              description="覆盖率体素边长（m）"),
        DeclareLaunchArgument(
            "scan_roi_min", default_value="-0.40,-0.50,-0.05",
            description="覆盖率统计 ROI 最小角 x,y,z；默认从台面上方开始，"
                        "不统计相机永远看不到的桌子下方体积"),
        DeclareLaunchArgument(
            "scan_roi_max", default_value="0.60,0.50,0.45",
            description="覆盖率统计 ROI 最大角 x,y,z"),
        DeclareLaunchArgument("scan_max_views", default_value="0",
                              description="最多执行视角数；0=全部"),
        DeclareLaunchArgument("scan_dry_run", default_value="false",
                              description="只做 IK/可达性检查，不执行"),
        DeclareLaunchArgument("scan_return_home", default_value="true"),
        DeclareLaunchArgument(
            "scan_top_first", default_value="true",
            description="顶视优先、低仰角最后，减少低位转移"),
        DeclareLaunchArgument(
            "scan_safe_transfer", default_value="true",
            description="视角之间先到上方预视角，再短距离下降；跨工位转移加"
                        "ee_link 最低高度路径约束"),
        DeclareLaunchArgument(
            "scan_safe_transfer_z", default_value="0.30",
            description="跨工位转移时 ee_link 的最低高度（m）"),
        DeclareLaunchArgument(
            "scan_approach_height", default_value="0.08",
            description="预视角相对目标视角的抬升高度（m）"),
        DeclareLaunchArgument(
            "scan_runtime_check", default_value="true",
            description="执行中监听 planning scene，复检剩余轨迹并取消/重规划"),
        DeclareLaunchArgument(
            "scan_check_period", default_value="0.50",
            description="剩余轨迹复检限频（s）"),
        DeclareLaunchArgument(
            "scan_check_lookahead", default_value="0.40",
            description="从当前时刻之后多久开始复检（s）"),
        DeclareLaunchArgument(
            "scan_check_stride", default_value="2",
            description="剩余轨迹采样步长（轨迹点）"),
        DeclareLaunchArgument(
            "scan_max_exec_replans", default_value="3",
            description="单段轨迹被挡时最多重规划次数"),
        DeclareLaunchArgument(
            "scan_cancel_settle", default_value="0.50",
            description="取消 JTC goal 后等待停稳的时间（s）"),
        DeclareLaunchArgument("scan_require_octomap", default_value="true"),
        DeclareLaunchArgument("scan_startup_timeout", default_value="90.0"),
        DeclareLaunchArgument(
            "scan_cloud_topic",
            default_value="/camera/depth/color/points_filtered",
            description="覆盖率统计用的自滤波点云"),
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
    start_fixed = _parse_bool(resolve("start_fixed_camera"), True)
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    share = get_package_share_directory("litearm_d435i_perception")
    old_share = get_package_share_directory("litearm_obstacle_avoidance")

    world = resolve("world").strip() or os.path.join(
        share, "worlds", "d435i_workcell.world")
    scene_file = resolve("scene_file").strip()
    fixed_mount_file = resolve("fixed_camera_mount_file").strip() or os.path.join(
        share, "config", "fixed_camera_mount.yaml")
    sensors_file = os.path.join(
        share, "config",
        "sensors_3d_wrist.yaml" if start_fixed
        else "sensors_3d_wrist_only.yaml")
    extra_gazebo_args = resolve("extra_gazebo_args").strip()
    dynamic_mode = resolve("dynamic_mode").strip() or "sweep"

    camera_parent = resolve("camera_parent_link").strip() or "ee_link"
    camera_xyz = " ".join(str(v) for v in _parse_xyz(
        resolve("camera_mount_xyz"), [0.0, 0.075, 0.02]))
    camera_rpy = " ".join(str(v) for v in _parse_xyz(
        resolve("camera_mount_rpy"), [0.0, 0.0, 0.0]))
    envelope_margin = resolve("gripper_envelope_margin").strip() or "0.01"

    xacro_args = [
        f"camera_parent_link:={camera_parent}",
        f"camera_mount_xyz:={camera_xyz}",
        f"camera_mount_rpy:={camera_rpy}",
        f"gripper_envelope_margin:={envelope_margin}",
    ]

    # ── Gazebo 侧：腕装组合模型 + 相机传感器 ──
    gazebo_xacro = os.path.join(
        share, "config", "litearm_litegrip_wrist_gazebo.urdf.xacro")
    raw = subprocess.check_output(
        ["xacro", gazebo_xacro, "camera_add_gazebo_sensor:=true"]
        + xacro_args, text=True)
    raw = re.sub(r"<!--.*?-->", "", raw, flags=re.S)
    raw = re.sub(r"<\?xml[^>]*\?>", "", raw)
    raw = _normalize_gripper_prefix(raw)
    raw = _resolve_package_uris(raw)
    gazebo_description = _add_missing_inertia(_rename_gazebo_links(raw))
    robot_description = " ".join(gazebo_description.split())

    gazebo_urdf_fd, gazebo_urdf_path = tempfile.mkstemp(
        prefix="litearm_wrist_gazebo_", suffix=".urdf")
    with os.fdopen(gazebo_urdf_fd, "w", encoding="utf-8") as handle:
        handle.write(gazebo_description)

    # ── MoveIt 侧：腕装组合模型 + 相机结构碰撞对 ──
    moveit_xacro = os.path.join(
        share, "config", "litearm_litegrip_wrist.urdf.xacro")
    source_srdf = os.path.join(old_share, "config", "litearm_litegrip.srdf")
    wrist_srdf = _wrist_srdf_text(source_srdf)
    srdf_fd, srdf_path = tempfile.mkstemp(
        prefix="litearm_wrist_", suffix=".srdf")
    with os.fdopen(srdf_fd, "w", encoding="utf-8") as handle:
        handle.write(wrist_srdf)

    builder = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(
            file_path=moveit_xacro,
            mappings={
                "gripper_envelope_margin": envelope_margin,
                "camera_parent_link": camera_parent,
                "camera_mount_xyz": camera_xyz,
                "camera_mount_rpy": camera_rpy,
            })
        .robot_description_semantic(file_path=srdf_path)
        .planning_pipelines(pipelines=["ompl"])
        .sensors_3d(file_path=sensors_file)
    )
    moveit_config = builder.to_moveit_configs()

    rsp_description = subprocess.check_output(
        ["xacro", moveit_xacro] + xacro_args, text=True)
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

    # ── Gazebo 进程 ──
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

    # 固定外置 D435i：独立命名空间，和腕装相机一起喂同一张 Octomap。
    # 先于机器人插入，避免 gazebo_ros2_control 初始化期间 spawn 请求排队。
    fixed_camera_spawn = Node(
        package="litearm_d435i_perception",
        executable="spawn_model.py",
        output="screen",
        additional_env=env,
        parameters=[{
            "entity_name": "fixed_d435i",
            "model_file": os.path.join(
                share, "models", "fixed_d435i_camera.sdf"),
            "mount_file": fixed_mount_file,
        }],
    )
    fixed_camera_tf = Node(
        package="litearm_d435i_perception",
        executable="camera_tf.py",
        name="fixed_camera_tf",
        output="screen",
        additional_env=env,
        parameters=[{"mount_file": fixed_mount_file,
                     "use_sim_time": True}],
    )
    if start_fixed:
        actions += [fixed_camera_spawn, fixed_camera_tf]

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

    scan_parameters = {
        "use_sim_time": True,
        "group_name": "litearm_arm",
        "ee_link": "ee_link",
        "camera_frame": "camera_depth_optical_frame",
        "sensor_topic": resolve("sensor_heartbeat_topic"),
        "cloud_topic": resolve("scan_cloud_topic"),
        "targets": resolve("scan_targets"),
        "radius": float(resolve("scan_radius")),
        "elevations": resolve("scan_elevations"),
        "azimuths": int(resolve("scan_azimuths")),
        "low_max_azimuth": float(resolve("scan_low_max_azimuth")),
        "dwell": float(resolve("scan_dwell")),
        "velocity_scaling": float(resolve("scan_velocity_scaling")),
        "acceleration_scaling": float(resolve("scan_acceleration_scaling")),
        "planning_time": float(resolve("scan_planning_time")),
        "plan_attempts": int(resolve("scan_plan_attempts")),
        "ik_seeds": int(resolve("scan_ik_seeds")),
        "clear_octomap": _parse_bool(resolve("scan_clear_octomap"), False),
        "require_octomap": _parse_bool(resolve("scan_require_octomap"), True),
        "coverage_target": float(resolve("scan_coverage_target")),
        "coverage_resolution": float(resolve("scan_coverage_resolution")),
        "roi_min": resolve("scan_roi_min"),
        "roi_max": resolve("scan_roi_max"),
        "max_views": int(resolve("scan_max_views")),
        "dry_run": _parse_bool(resolve("scan_dry_run"), False),
        "return_home": _parse_bool(resolve("scan_return_home"), True),
        "top_first": _parse_bool(resolve("scan_top_first"), True),
        "safe_transfer": _parse_bool(resolve("scan_safe_transfer"), True),
        "safe_transfer_z": float(resolve("scan_safe_transfer_z")),
        "approach_height": float(resolve("scan_approach_height")),
        "runtime_check": _parse_bool(resolve("scan_runtime_check"), True),
        "check_period": float(resolve("scan_check_period")),
        "check_lookahead": float(resolve("scan_check_lookahead")),
        "check_stride": int(resolve("scan_check_stride")),
        "max_exec_replans": int(resolve("scan_max_exec_replans")),
        "cancel_settle": float(resolve("scan_cancel_settle")),
        "require_scene_objects": bool(scene_file),
        "auto_start": _parse_bool(resolve("scan_auto_start"), True),
        "start_delay": float(resolve("scan_start_delay")),
        "startup_timeout": float(resolve("scan_startup_timeout")),
    }
    actions.append(Node(
        package="litearm_d435i_perception",
        executable="wrist_scan.py",
        output="screen",
        additional_env=env,
        parameters=[scan_parameters],
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

    point_cloud_topics = "腕装 /camera/depth/color/points"
    if start_fixed:
        point_cloud_topics += "；固定 /fixed_camera/depth/color/points"
    else:
        point_cloud_topics += "（固定相机未启动）"

    actions.append(LogInfo(msg=(
        "──────── litearm 腕装 D435i 扫描 ────────\n"
        f"  world:       {world}\n"
        f"  scene:       {scene_file or '未预置（固定+腕装相机 Octomap）'}\n"
        f"  腕装相机:     {os.path.join(share, 'config', 'wrist_camera.xacro')}"
        f"  xyz=({camera_xyz}) rpy=({camera_rpy})\n"
        f"  固定相机:     {start_fixed}"
        f"（{fixed_mount_file if start_fixed else '未启动'}）\n"
        f"  点云话题:     {point_cloud_topics}\n"
        f"  传感器配置:   {sensors_file}\n"
        f"  扫描:         targets={resolve('scan_targets')} "
        f"radius={resolve('scan_radius')} "
        f"elevations={resolve('scan_elevations')} "
        f"azimuths={resolve('scan_azimuths')}\n"
        f"  自动开始:     {_parse_bool(resolve('scan_auto_start'), True)}"
        f"（延迟 {resolve('scan_start_delay')} s）"
        f"  dry_run={resolve('scan_dry_run')}\n"
        f"  Gazebo GUI:  {gui}    RViz: {use_rviz}\n"
        f"  Supervisor:  {start_supervisor}   动态障碍: {start_dynamic} "
        f"({dynamic_mode})\n"
        f"  可移动物块:  "
        f"{movable_pose if start_movable else '未加载'}\n"
        f"  ⚠ 不要和 d435i_avoidance.launch.py 同时运行（旧 launch 的固定"
        f"相机占用 /camera/...）\n"
        f"  ⚠ 终端里用 ros2 命令前先执行: export ROS_DOMAIN_ID={domain_id}; "
        f"export ROS_LOCALHOST_ONLY="
        f"{'1' if _parse_bool(localhost_only, True) else '0'}\n"
        "  手动扫描: ros2 service call /wrist_scan/start "
        "std_srvs/srv/Trigger\n"
        "  查看状态: ros2 topic echo /wrist_scan/status\n"
        "────────────────────────────────────────")))

    return actions


def generate_launch_description():
    share = get_package_share_directory("litearm_d435i_perception")
    return LaunchDescription(
        _declare_arguments(share)
        + [OpaqueFunction(function=_launch_setup)])
