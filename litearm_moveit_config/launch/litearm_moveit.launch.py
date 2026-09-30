#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_moveit.launch.py — bring up litearm's MoveIt planning stack
(optionally together with the low-level control stack).

Combine as needed:

    1. control stack   litearm_ros2_control/litearm_control.launch.py
                       (hardware daemon + controller_manager + JSB + JTC)
    2. move_group      MoveIt planning node (OMPL + KDL)
    3. RViz            MoveIt motion planning panel

By default all three come up together. If the control stack is already running
elsewhere (e.g. started by hand while debugging the real robot), use
start_control:=false to bring up only the MoveIt part, so that two
controller_managers do not fight over the same set of command interfaces.

    # real robot (board powered, USB connected, license activated)
    ros2 launch litearm_moveit_config litearm_moveit.launch.py

    # full-stack dry run without hardware
    ros2 launch litearm_moveit_config litearm_moveit.launch.py dry_run:=true

    # MoveIt only (control stack already running)
    ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false

Feedforward is computed by the **firmware** (no switch to turn on here): the PD
gains and the gravity/friction/integral/kd_extra feedforward all live in the
litearm-stm32 firmware, whose model is generated from the URDF and compiled into
the firmware. On the default channel (MOVE_JS) there is **no M·q̈ or C·q̇** —
MOVE_JS has no acceleration source.

The five switches below are **tri-state**, default empty = leave the firmware
alone (the firmware's factory mask already carries
G/inertia/Coriolis/friction/integral/quantization/velocity reference):

    gravity_compensation:=true|false    overrides FF_G
    friction_compensation:=true|false   overrides FF_FRICTION
    inertia_compensation:=true|false    overrides FF_INERTIA|FF_CORIOLIS
                                        (no effect on MOVE_JS)
    integral_compensation:=true|false   overrides FF_INTEGRAL
    damping_compensation:=true|false    overrides the kd_extra vector
                                        (true = restore factory values)

To fall back to pure PD (e.g. for an A/B comparison), set all five to false:
    ros2 launch litearm_moveit_config litearm_moveit.launch.py \\
        gravity_compensation:=false friction_compensation:=false \\
        inertia_compensation:=false integral_compensation:=false \\
        damping_compensation:=false

To compute feedforward yourself (KP/Kd/effort applied every frame), use the
control stack's mit_passthrough:=true and change the
joint_trajectory_controller command_interfaces to the matching combination.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from litearm_ros2_control.ros_env import isolation_hint, parse_bool, ros_isolation_env


def _is_true(value: str) -> bool:
    return parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument("start_control", default_value="true",
                              description="whether to start the ros2_control stack as well"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="passed through to the control stack: true = hardware-free mode"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="whether to start RViz2 (the MoveIt motion planning panel)"),
        DeclareLaunchArgument("port", default_value="",
                              description="passed through to the control stack: USB CDC "
                                          "device path of the litearm-stm32, empty = "
                                          "auto-detect (1d50:606f)"),
        DeclareLaunchArgument("shm_name", default_value="/litearm_hw",
                              description="passed through to the control stack: shared memory segment name"),
        DeclareLaunchArgument("daemon_rate_hz", default_value="0",
                              description="passed through to the control stack: daemon "
                                          "command rate, 0 = use the default"),
        # The five feedforward switches are **tri-state**: default empty = leave
        # the firmware alone (the firmware has its own factory mask, which
        # already carries G/inertia/Coriolis/friction/integral/quantization/
        # velocity reference).
        # Before the lower layer was swapped this was true here (the daemon
        # stacked its own feedforward on tau_ff); now that the firmware computes
        # it, passing true merely "re-sets a bit that is already set" — harmless
        # but meaningless, and it makes people think the MoveIt entry point and
        # starting the control stack directly are two different behaviours.
        DeclareLaunchArgument("gravity_compensation", default_value="",
                              description="passed through to the control stack: overrides the "
                                          "FF_G bit of the firmware ff_mask "
                                          "(empty = leave the firmware alone, true/false = set/clear)"),
        DeclareLaunchArgument("friction_compensation", default_value="",
                              description="passed through to the control stack: overrides the "
                                          "FF_FRICTION bit of the firmware ff_mask"),
        DeclareLaunchArgument("inertia_compensation", default_value="",
                              description="passed through to the control stack: overrides the "
                                          "FF_INERTIA|FF_CORIOLIS bits of the firmware ff_mask "
                                          "(⚠ on the MOVE_JS channel the firmware does not "
                                          "compute inertia terms, so setting them is useless)"),
        DeclareLaunchArgument("integral_compensation", default_value="",
                              description="passed through to the control stack: overrides the "
                                          "FF_INTEGRAL bit of the firmware ff_mask"),
        DeclareLaunchArgument("damping_compensation", default_value="",
                              description="passed through to the control stack: overrides the "
                                          "firmware kd_extra vector "
                                          "(false = zero it, true = restore factory values)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group log level"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="ROS domain used by this stack (the default deliberately "
                                          "avoids 0). Set it to 0 to interoperate with external systems"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = local discovery only. Cross-machine crosstalk makes "
                                          "RViz show someone else's robot and puts multiple "
                                          "servers on /move_action"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    start_control = _is_true(resolve("start_control"))
    use_rviz = _is_true(resolve("use_rviz"))
    shm_name = resolve("shm_name")
    dry_run = resolve("dry_run")
    port = resolve("port")
    daemon_rate_hz = resolve("daemon_rate_hz")
    gravity_compensation = resolve("gravity_compensation")
    friction_compensation = resolve("friction_compensation")
    inertia_compensation = resolve("inertia_compensation")
    integral_compensation = resolve("integral_compensation")
    damping_compensation = resolve("damping_compensation")
    domain_id = resolve("ros_domain_id").strip() or "42"
    localhost_only = resolve("ros_localhost_only")
    env = ros_isolation_env(domain_id, localhost_only)

    moveit_share = get_package_share_directory("litearm_moveit_config")
    control_share = get_package_share_directory("litearm_ros2_control")

    # shm_name must match the control stack: the MoveIt side has to inject it
    # when expanding the URDF as well, otherwise the robot_description move_group
    # gets does not match the hardware plugin parameters actually loaded.
    moveit_config = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(mappings={"litearm_shm_name": shm_name})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    actions = []

    if start_control:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_share, "launch", "litearm_control.launch.py")),
            launch_arguments={
                "dry_run": dry_run,
                "port": port,
                "shm_name": shm_name,
                "daemon_rate_hz": daemon_rate_hz,
                "gravity_compensation": gravity_compensation,
                "friction_compensation": friction_compensation,
                "inertia_compensation": inertia_compensation,
                "integral_compensation": integral_compensation,
                "damping_compensation": damping_compensation,
                "use_rviz": "false",  # either the control stack's own RViz or the MoveIt RViz below
                # The isolation parameters must be forwarded: the included
                # processes have to land in the same domain, otherwise move_group
                # cannot see controller_manager's actions at all.
                "ros_domain_id": domain_id,
                "ros_localhost_only": localhost_only,
            }.items(),
        ))

    actions.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=env,
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", resolve("log_level")],
    ))

    if use_rviz:
        # Whether RViz starts is decided only by the Python check above; do
        # **not** add a runtime IfCondition(LaunchConfiguration("use_rviz")):
        # ROS 2 launch implements the launch_arguments of an
        # IncludeLaunchDescription as SetLaunchConfiguration, which writes into
        # the shared context — the control stack include above explicitly passes
        # use_rviz:=false ("either the control stack RViz or the MoveIt RViz"),
        # which overwrites this function's use_rviz configuration as false, so
        # the runtime condition becomes permanently false and RViz is silently
        # skipped (we hit this in this project: the launch log did not even show
        # an rviz2 process).
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(moveit_share, "rviz", "moveit.rviz")],
            output="log",
            additional_env=env,
            # The MoveIt RViz plugin needs these parameter sets itself; it
            # cannot just rely on move_group.
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ))

    banner_target = ("dry-run (fake firmware on a pty)" if _is_true(dry_run)
                     else f"real robot, port={port or '(auto-detect)'}")
    actions.insert(0, LogInfo(msg=(
        "──────── litearm MoveIt ────────\n"
        f"  {banner_target}\n"
        "  ⚠ This launch pins the ROS domain; to use the ros2 CLI in a terminal\n"
        "    or look at RViz data, first run this in your own terminal:\n"
        f"        {isolation_hint(domain_id, localhost_only)}\n"
        "────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
