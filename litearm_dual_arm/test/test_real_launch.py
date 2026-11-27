"""真机双臂 launch 的安全门与参数测试。"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "dual_arm_real.launch.py"


def _launch_module():
    spec = importlib.util.spec_from_file_location("dual_arm_real_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_execute_gate_requires_all_safety_confirmations():
    module = _launch_module()
    with pytest.raises(RuntimeError, match="authorization"):
        module._validate_execute_gate(True, "", True, True, True)
    with pytest.raises(RuntimeError, match="estop"):
        module._validate_execute_gate(True, module.MOTION_TOKEN, False, True, True)
    with pytest.raises(RuntimeError, match="workspace"):
        module._validate_execute_gate(True, module.MOTION_TOKEN, True, False, True)
    with pytest.raises(RuntimeError, match="measured"):
        module._validate_execute_gate(True, module.MOTION_TOKEN, True, True, False)
    module._validate_execute_gate(
        True, module.MOTION_TOKEN, True, True, True
    )


def test_plan_only_needs_no_execute_gate():
    module = _launch_module()
    module._validate_execute_gate(False, "", False, False, False)


def test_real_launch_declares_safety_and_control_arguments():
    description = _launch_module().generate_launch_description()
    names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        "execute",
        "authorization",
        "estop_verified",
        "workspace_verified",
        "start_control",
        "left_port",
        "right_port",
        "preflight",
        "bridge_start_tolerance",
        "bridge_max_joint_velocity",
        "mounts_file",
        "scene",
        "ros_domain_id",
        "ros_localhost_only",
    } <= names


def test_real_launch_is_plan_only_by_default_and_uses_state_only_bridge():
    text = LAUNCH.read_text(encoding="utf-8")
    assert '"execute", default_value="false"' in text
    assert '"allow_trajectory_execution": execute' in text
    assert '[] if execute else ["--state-only"]' in text
    assert "dual_arm_preflight.py" in text
    assert "dual_arm_bridge.py" in text


def test_real_launch_starts_no_simulator():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "dual_arm_sim.py" not in text
