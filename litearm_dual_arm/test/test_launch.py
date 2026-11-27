"""launch 参数与执行门控的静态测试。"""

from __future__ import annotations

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "dual_arm_sim.launch.py"


def _launch_module():
    spec = importlib.util.spec_from_file_location("dual_arm_sim_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_launch_declares_expected_arguments():
    description = _launch_module().generate_launch_description()
    names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        "use_rviz",
        "mounts_file",
        "scene",
        "robot_padding_m",
        "allowed_start_tolerance",
        "publish_rate",
        "initial_positions",
        "rviz_config",
        "log_level",
        "ros_domain_id",
        "ros_localhost_only",
        "execute",
        "task",
        "task_plan_only",
        "task_output",
        "task_timeout",
        "bridge_start_tolerance",
        "bridge_max_joint_velocity",
        "bridge_max_state_age",
        "bridge_max_state_skew",
    } <= names


def test_launch_supports_execution_and_bridge():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"allow_trajectory_execution": execute' in text
    assert "dual_arm_sim.py" in text
    assert "dual_arm_bridge.py" in text
    assert "dual_arm_scene.py" in text
    assert "generate_dual_arm_model.py" in text


def test_launch_defaults_isolate_the_stack():
    module = _launch_module()
    description = module.generate_launch_description()
    args = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    # LaunchArgument 的默认值在 Humble 里是私有属性；用模块常量检查 domain 即可。
    assert module.DEFAULT_DOMAIN_ID == "42"
    assert "ros_domain_id" in args
