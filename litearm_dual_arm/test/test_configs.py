"""配置一致性测试：组名、关节映射、目标范围、场景引用。"""

from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
LEFT = tuple(f"left_joint{index}" for index in range(1, 8))
RIGHT = tuple(f"right_joint{index}" for index in range(1, 8))
ALL = LEFT + RIGHT
GROUP_JOINTS = {"left_arm": LEFT, "right_arm": RIGHT, "both_arms": ALL}


def load(name):
    with open(CONFIG / name, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def test_mounts_are_well_formed():
    mounts = load("dual_arm_mounts.yaml")
    assert mounts["frame"] == "world"
    assert mounts["calibration"]["measured"] is False
    for side in ("left", "right"):
        assert len(mounts[side]["xyz"]) == 3
        assert len(mounts[side]["rpy"]) == 3
    assert mounts["tool"]["type"] in ("none", "cylinder")


def test_kinematics_only_defines_single_arm_groups():
    kinematics = load("dual_kinematics.yaml")
    assert set(kinematics) == {"left_arm", "right_arm"}
    for group in ("left_arm", "right_arm"):
        assert kinematics[group]["kinematics_solver"].startswith("kdl_kinematics_plugin/")


def test_ompl_covers_all_groups_and_does_not_fix_start_collision():
    ompl = load("dual_ompl.yaml")
    assert set(GROUP_JOINTS) <= set(ompl)
    assert ompl["planning_plugin"] == "ompl_interface/OMPLPlanner"
    assert "ompl_interface/OMPLPlanner" in ompl["planning_plugins"]
    adapters = ompl["request_adapters"]
    assert "FixStartStateCollision" not in adapters
    for group in GROUP_JOINTS:
        default = ompl[group]["default_planner_config"]
        assert default in ompl["planner_configs"]
        assert default in ompl[group]["planner_configs"]


def test_controllers_map_exactly_seven_joints_per_side():
    controllers = load("dual_controllers.yaml")
    simple = controllers["moveit_simple_controller_manager"]
    assert simple["controller_names"] == ["left_arm_controller", "right_arm_controller"]
    for side, joints in (("left", LEFT), ("right", RIGHT)):
        entry = simple[f"{side}_arm_controller"]
        assert entry["type"] == "FollowJointTrajectory"
        assert tuple(entry["joints"]) == joints


def test_goals_match_group_joint_sets():
    targets = load("dual_goals.yaml")["targets"]
    assert targets
    for name, entry in targets.items():
        group = entry["group"]
        assert group in GROUP_JOINTS, name
        positions = entry["positions"]
        assert set(positions) == set(GROUP_JOINTS[group]), name
        for value in positions.values():
            assert isinstance(value, (int, float)), name


def test_goal_values_are_inside_single_arm_joint_limits():
    source = (
        ROOT.parents[1]
        / "litearm-moveit2"
        / "litearm_moveit_config"
        / "config"
        / "joint_limits.yaml"
    )
    if not source.exists():
        from ament_index_python.packages import get_package_share_directory

        source = (
            Path(get_package_share_directory("litearm_moveit_config"))
            / "config"
            / "joint_limits.yaml"
        )
    with open(source, "r", encoding="utf-8") as stream:
        limits = yaml.safe_load(stream)["joint_limits"]

    targets = load("dual_goals.yaml")["targets"]
    for name, entry in targets.items():
        for joint, value in entry["positions"].items():
            local = joint.split("_", 1)[1]
            limit = limits[local]
            assert limit["min_position"] - 1e-9 <= value <= limit["max_position"] + 1e-9, (
                name,
                joint,
                value,
            )


def test_scene_objects_and_scene_references_are_consistent():
    scenes = load("dual_scenes.yaml")
    assert scenes["frames"]["dual"] == "world"
    objects = scenes["objects"]
    for scene_name, names in scenes["scenes"].items():
        assert names, scene_name
        for name in names:
            assert name in objects, (scene_name, name)
    for name, spec in objects.items():
        assert spec["type"] in ("box", "cylinder"), name
        assert len(spec["pose"]) == 3, name
        expected = 2 if spec["type"] == "cylinder" else 3
        assert len(spec["dimensions"]) == expected, name
