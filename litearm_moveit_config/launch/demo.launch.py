#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo.launch.py — MoveIt demo that does not touch real hardware.

Brings up the MoveIt panel with a single command: drag the goal configuration
with the mouse in RViz, click Plan & Execute, and watch the arm actually "move":

    ros2 launch litearm_moveit_config demo.launch.py

In essence this is litearm_moveit.launch.py with dry_run hard-pinned to true.
The reason for a separate entry point is the same as for litearm_demo.launch.py:
**splitting "demo" and "real robot" into two commands** means there is no way to
mix up a parameter and bring up the real arm by mistake.

For the real robot use:
    ros2 launch litearm_moveit_config litearm_moveit.launch.py
(board powered, USB connected, license activated, and make sure the arm is
supported and the emergency stop is within reach)

Named states available in RViz (SRDF group_state):

    zero    all joints at zero
    ready   stretched-out configuration with the elbow raised; a better
            planning start than zero (zero is near a singularity, where
            Cartesian planning tends to fail)

In a headless environment add use_rviz:=false and verify from the command line:
    ros2_ws/src/litearm_moveit_config/scripts/acceptance_moveit.sh --execute
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _declare_arguments():
    return [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="whether to start RViz2 (the MoveIt motion planning panel)"),
        DeclareLaunchArgument("shm_name", default_value="/litearm_hw",
                              description="shared memory segment name (no need to change for the demo)"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="ROS domain used by this demo (default deliberately avoids 0)"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = local discovery only, avoids cross-machine crosstalk"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("litearm_moveit_config") + "/launch/litearm_moveit.launch.py"),
        launch_arguments={
            # Hard-pinned: this entry point is always simulated (fake firmware
            # on a pty) and never touches real hardware
            "dry_run": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "shm_name": resolve("shm_name"),
            "ros_domain_id": resolve("ros_domain_id"),
            "ros_localhost_only": resolve("ros_localhost_only"),
        }.items())
    return [
        LogInfo(msg=(
            "──────── litearm MoveIt hardware-free demo ────────\n"
            "  Mode: dry-run (daemon runs fake firmware on a pty; the link uses the real protocol)\n"
            "  In RViz, drag a goal in the MotionPlanning panel → Plan & Execute\n"
            "  For the real robot use: ros2 launch litearm_moveit_config litearm_moveit.launch.py\n"
            "──────────────────────────────────────────")),
        moveit_launch,
    ]


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
