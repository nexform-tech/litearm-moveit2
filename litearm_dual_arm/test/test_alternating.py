"""交替作业配置与目标一致性测试。"""

from __future__ import annotations

from pathlib import Path

from dual_alternating_demo import (
    load_alternating_config,
    load_target,
    load_targets,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "dual_alternating.yaml"
GOALS = ROOT / "config" / "dual_goals.yaml"
LEFT = {f"left_joint{index}" for index in range(1, 8)}
RIGHT = {f"right_joint{index}" for index in range(1, 8)}
GROUP_JOINTS = {
    "left_arm": LEFT,
    "right_arm": RIGHT,
    "both_arms": LEFT | RIGHT,
}


def test_alternating_config_has_expected_stage_order():
    config = load_alternating_config(str(CONFIG))
    names = [stage["name"] for stage in config["stages"]]
    assert names == [
        "both_ready",
        "left_enter",
        "left_retreat",
        "right_enter",
        "right_retreat",
        "both_park",
    ]
    assert config["scene"] == "dual_alternating_static"


def test_every_stage_target_exists_and_matches_group():
    config = load_alternating_config(str(CONFIG))
    targets = load_targets(str(GOALS))
    for stage in config["stages"]:
        group, positions = load_target(targets, stage["target"])
        assert group == stage["active"], stage["name"]
        assert set(positions) == GROUP_JOINTS[group], stage["name"]


def test_alternating_requires_a_motion_block():
    config = load_alternating_config(str(CONFIG))
    assert config["motion"]["samples"] >= 2
    assert config["validation"]["idle_tolerance_rad"] > 0.0
    assert config["motion"]["goal_tolerance_rad"] > 0.0
