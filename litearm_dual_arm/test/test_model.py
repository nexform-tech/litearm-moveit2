"""双臂模型生成测试：前缀、world 挂载、三组、跨臂碰撞对。"""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from generate_dual_arm_model import build_dual_srdf, build_dual_urdf, load_mounts


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT.parents[1]


def _source_or_installed(path: Path, package: str, relative: str) -> Path:
    if path.exists():
        return path
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package)) / relative


SOURCE_URDF = _source_or_installed(
    SOURCE_ROOT / "litearm-description" / "SE" / "litearm_description" / "urdf" / "litearm_description.urdf",
    "litearm_description",
    "urdf/litearm_description.urdf",
)
SOURCE_SRDF = _source_or_installed(
    SOURCE_ROOT / "litearm-moveit2" / "litearm_moveit_config" / "config" / "litearm.srdf",
    "litearm_moveit_config",
    "config/litearm.srdf",
)
MOUNTS = ROOT / "config" / "dual_arm_mounts.yaml"
SIDES = ("left", "right")


@pytest.fixture(scope="module")
def mounts():
    return load_mounts(str(MOUNTS))


@pytest.fixture(scope="module")
def urdf_root(mounts):
    return ET.fromstring(
        build_dual_urdf(ET.parse(SOURCE_URDF).getroot(), mounts)
    )


@pytest.fixture(scope="module")
def srdf_root(mounts):
    return ET.fromstring(
        build_dual_srdf(ET.parse(SOURCE_SRDF).getroot(), mounts)
    )


def test_urdf_has_world_and_two_chains(urdf_root):
    assert urdf_root.get("name") == "litearm_dual"
    links = {link.get("name") for link in urdf_root.findall("link")}
    joints = {joint.get("name"): joint for joint in urdf_root.findall("joint")}

    assert "world" in links
    for side in SIDES:
        assert f"{side}_base_link" in links
        assert f"{side}_ee_link" in links
        for index in range(1, 8):
            assert f"{side}_joint{index}" in links
            assert f"{side}_joint{index}" in joints
        assert f"{side}_world_joint" in joints
        assert joints[f"{side}_world_joint"].get("type") == "fixed"
        assert joints[f"{side}_world_joint"].find("parent").get("link") == "world"
        assert (
            joints[f"{side}_world_joint"].find("child").get("link")
            == f"{side}_base_link"
        )

    # 单臂源里的名字不应再以裸名出现，否则 MoveIt 会把两套链当成一套。
    assert "base_link" not in links
    assert "ee_link" not in links
    assert len([name for name in joints if name.startswith("joint")]) == 0


def test_world_mount_origins_match_yaml(urdf_root, mounts):
    for side in SIDES:
        joint = next(
            item
            for item in urdf_root.findall("joint")
            if item.get("name") == f"{side}_world_joint"
        )
        origin = joint.find("origin")
        xyz = [float(value) for value in origin.get("xyz").split()]
        rpy = [float(value) for value in origin.get("rpy").split()]
        assert xyz == pytest.approx(list(mounts["mounts"][side][0]))
        assert rpy == pytest.approx(list(mounts["mounts"][side][1]))


def test_srdf_has_three_groups(srdf_root):
    groups = {group.get("name"): group for group in srdf_root.findall("group")}
    assert set(groups) == {"left_arm", "right_arm", "both_arms"}

    for side in SIDES:
        chain = groups[f"{side}_arm"].find("chain")
        assert chain.get("base_link") == f"{side}_base_link"
        assert chain.get("tip_link") == f"{side}_ee_link"

    subgroups = {
        subgroup.get("name") for subgroup in groups["both_arms"].findall("group")
    }
    assert subgroups == {"left_arm", "right_arm"}


def test_srdf_group_states_are_complete(srdf_root):
    states = {state.get("name"): state for state in srdf_root.findall("group_state")}
    assert {"left_zero", "right_zero", "both_zero", "left_ready", "right_ready", "both_ready"} <= set(states)
    for name in ("left_zero", "left_ready", "right_zero", "right_ready"):
        assert len(states[name].findall("joint")) == 7
    for name in ("both_zero", "both_ready"):
        assert len(states[name].findall("joint")) == 14
        names = {joint.get("name") for joint in states[name].findall("joint")}
        assert {f"left_joint{index}" for index in range(1, 8)} <= names
        assert {f"right_joint{index}" for index in range(1, 8)} <= names


def test_srdf_disables_only_intra_arm_pairs(srdf_root):
    disables = srdf_root.findall("disable_collisions")
    assert disables
    for disable in disables:
        link1 = disable.get("link1")
        link2 = disable.get("link2")
        assert not (
            link1.startswith("left_") and link2.startswith("right_")
        ), f"cross-arm pair must stay enabled: {link1} <-> {link2}"
        assert not (
            link1.startswith("right_") and link2.startswith("left_")
        ), f"cross-arm pair must stay enabled: {link1} <-> {link2}"

    pairs = {(disable.get("link1"), disable.get("link2")) for disable in disables}
    for side in SIDES:
        assert ("world", f"{side}_base_link") in pairs
        assert (f"{side}_base_link", f"{side}_joint1") in pairs
        assert (f"{side}_joint7", f"{side}_ee_link") in pairs


def test_srdf_end_effectors(srdf_root):
    effectors = {
        item.get("name"): item for item in srdf_root.findall("end_effector")
    }
    assert set(effectors) == {"left_ee", "right_ee"}
    for side in SIDES:
        assert effectors[f"{side}_ee"].get("parent_link") == f"{side}_ee_link"
        assert effectors[f"{side}_ee"].get("group") == f"{side}_arm"


def test_tool_option_changes_tip_and_adds_link(mounts):
    tool_mounts = copy.deepcopy(mounts)
    tool_mounts["tool"] = {"type": "cylinder", "length": 0.08, "radius": 0.025}
    urdf = ET.fromstring(
        build_dual_urdf(ET.parse(SOURCE_URDF).getroot(), tool_mounts)
    )
    links = {link.get("name") for link in urdf.findall("link")}
    joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    for side in SIDES:
        assert f"{side}_tool_link" in links
        assert joints[f"{side}_tool_joint"].get("type") == "fixed"

    srdf = ET.fromstring(
        build_dual_srdf(ET.parse(SOURCE_SRDF).getroot(), tool_mounts)
    )
    groups = {group.get("name"): group for group in srdf.findall("group")}
    for side in SIDES:
        assert groups[f"{side}_arm"].find("chain").get("tip_link") == f"{side}_tool_link"
    pairs = {
        (item.get("link1"), item.get("link2"))
        for item in srdf.findall("disable_collisions")
    }
    for side in SIDES:
        assert (f"{side}_ee_link", f"{side}_tool_link") in pairs
