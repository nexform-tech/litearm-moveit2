#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""generate_dual_arm_model.py — 由单臂 LiteArm 描述生成双臂 URDF / SRDF。

用途
====
`litearm_description` 里只有一条 LiteArm 链。双臂任务需要两套同名但带前缀的
链，并且要共享一个 ``world`` 坐标系。本脚本不复制网格、不改上游包，只做：

    单臂 URDF  ──加 left_/right_ 前缀──┐
                                        ├─→ 双臂 URDF
    dual_arm_mounts.yaml ──world 固定挂载┘
    单臂 SRDF  ──加前缀 + 三组 + 跨臂碰撞规则──→ 双臂 SRDF

生成结果直接输出到 stdout，由 launch 作为 robot_description / semantic 参数
使用；也可以手工运行后重定向到文件检查。

    python3 generate_dual_arm_model.py --mode urdf
    python3 generate_dual_arm_model.py --mode srdf
    python3 generate_dual_arm_model.py --mode both

设计约束
========
* 左右臂的关节名固定为 ``left_joint1..7`` / ``right_joint1..7``。这是
  MoveIt 组、bridge 和控制器映射共同依赖的命名契约，不能随意改。
* SRDF 的 ``disable_collisions`` 只从单臂继承“同一侧内部”的相邻对，再补
  ``world`` 到各自 ``base_link``；左右臂之间的任何 link 对都**不禁用**，
  这样 MoveIt 才会在规划时检查双臂互碰。
* 当前只支持 ``world`` 作为根坐标系。以后要挂到别的世界系时，先改这里和
  RViz 的 Fixed Frame，不要只改 YAML，否则模型和显示会对不上。
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Mapping, Sequence

import yaml


SIDES = ("left", "right")
PREFIX = {"left": "left_", "right": "right_"}
ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 8))
REQUIRED_LINKS = ("base_link", "ee_link")
DEFAULT_ROBOT_NAME = "litearm_dual"
SUPPORTED_TOP_LEVEL_TAGS = {"link", "joint", "material"}


class ModelError(RuntimeError):
    """输入描述或配置不满足双臂模型生成约定。"""


def _finite_vector(value, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ModelError(f"{field} must be a list of {length} numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ModelError(f"{field} contains a non-finite number")
    return result


def _vector_text(values: Sequence[float]) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def load_mounts(path: str) -> dict:
    """读取并校验左右基座挂载与（可选）末端工具配置。"""
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ModelError(f"mount configuration must be a mapping: {path}")

    frame = str(config.get("frame", "world")).strip() or "world"
    if frame != "world":
        raise ModelError(
            "only 'world' is supported as the root frame; changing it also "
            "requires changing the RViz Fixed Frame and the planning frame"
        )

    mounts = {}
    for side in SIDES:
        entry = config.get(side)
        if not isinstance(entry, dict):
            raise ModelError(f"missing '{side}' mount entry in {path}")
        xyz = _finite_vector(entry.get("xyz"), 3, f"{side}.xyz")
        rpy = _finite_vector(entry.get("rpy"), 3, f"{side}.rpy")
        mounts[side] = (xyz, rpy)

    raw_tool = config.get("tool") or {}
    if not isinstance(raw_tool, dict):
        raise ModelError("tool must be a mapping or empty")
    tool_type = str(raw_tool.get("type", "none")).strip().lower()
    if tool_type not in ("none", "cylinder"):
        raise ModelError("tool.type must be 'none' or 'cylinder'")
    tool = {
        "type": tool_type,
        "length": float(raw_tool.get("length", 0.0) or 0.0),
        "radius": float(raw_tool.get("radius", 0.0) or 0.0),
    }
    if tool_type == "cylinder":
        if not (
            math.isfinite(tool["length"])
            and tool["length"] > 0.0
            and math.isfinite(tool["radius"])
            and tool["radius"] > 0.0
        ):
            raise ModelError("cylinder tool length and radius must be positive")

    calibration = config.get("calibration") or {}
    if not isinstance(calibration, dict):
        raise ModelError("calibration must be a mapping")
    return {
        "frame": frame,
        "mounts": mounts,
        "tool": tool,
        "calibration": calibration,
    }


def _validate_source_urdf(root: ET.Element) -> None:
    if root.tag != "robot":
        raise ModelError("source URDF root element must be <robot>")
    link_names = {link.get("name") for link in root.findall("link")}
    joint_names = {joint.get("name") for joint in root.findall("joint")}
    missing_links = sorted(set(REQUIRED_LINKS) - link_names)
    if missing_links:
        raise ModelError(f"source URDF is missing links: {missing_links}")
    missing_joints = sorted(set(ARM_JOINTS) - joint_names)
    if missing_joints:
        raise ModelError(f"source URDF is missing joints: {missing_joints}")
    for child in root:
        if child.tag not in SUPPORTED_TOP_LEVEL_TAGS:
            raise ModelError(
                f"unsupported top-level URDF element <{child.tag}>; extend the "
                "generator before adding it to the description package"
            )


def _prefix_child(child: ET.Element, prefix: str) -> ET.Element:
    """深拷贝一个 URDF 子元素，并把 link/joint 名称加前缀。"""
    element = copy.deepcopy(child)
    if element.tag == "link":
        name = element.get("name")
        if not name:
            raise ModelError("source URDF has a <link> without a name")
        element.set("name", prefix + name)
    elif element.tag == "joint":
        name = element.get("name")
        if not name:
            raise ModelError("source URDF has a <joint> without a name")
        element.set("name", prefix + name)
        parent = element.find("parent")
        child_link = element.find("child")
        if parent is None or child_link is None:
            raise ModelError(f"source URDF joint {name!r} misses parent/child")
        parent.set("link", prefix + parent.get("link", ""))
        child_link.set("link", prefix + child_link.get("link", ""))
        mimic = element.find("mimic")
        if mimic is not None and mimic.get("joint"):
            mimic.set("joint", prefix + mimic.get("joint", ""))
    return element


def _fixed_joint(
    name: str,
    parent: str,
    child: str,
    xyz: Sequence[float] = (0.0, 0.0, 0.0),
    rpy: Sequence[float] = (0.0, 0.0, 0.0),
) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": _vector_text(xyz), "rpy": _vector_text(rpy)})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def _add_tool(robot: ET.Element, prefix: str, tool: Mapping) -> None:
    """在 ee_link 后追加一个圆柱工具碰撞体；本期默认不启用。"""
    link = ET.SubElement(robot, "link", {"name": f"{prefix}tool_link"})
    length = float(tool["length"])
    radius = float(tool["radius"])
    center_z = length / 2.0
    for kind in ("visual", "collision"):
        element = ET.SubElement(link, kind)
        ET.SubElement(
            element,
            "origin",
            {"xyz": f"0 0 {center_z:.9g}", "rpy": "0 0 0"},
        )
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(
            geometry,
            "cylinder",
            {"radius": f"{radius:.9g}", "length": f"{length:.9g}"},
        )
    visual = link.find("visual")
    material = ET.SubElement(visual, "material", {"name": "litearm_tool"})
    ET.SubElement(material, "color", {"rgba": "0.12 0.62 0.88 1.0"})
    robot.append(
        _fixed_joint(
            f"{prefix}tool_joint",
            f"{prefix}ee_link",
            f"{prefix}tool_link",
        )
    )


def _indent(root: ET.Element) -> None:
    try:
        ET.indent(root, space="  ")
    except AttributeError:  # Python < 3.9
        pass


def _xml(root: ET.Element) -> str:
    _indent(root)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def build_dual_urdf(
    source_root: ET.Element,
    mounts: Mapping,
    *,
    robot_name: str = DEFAULT_ROBOT_NAME,
) -> str:
    _validate_source_urdf(source_root)
    frame = mounts["frame"]
    tool = mounts["tool"]

    robot = ET.Element("robot", {"name": robot_name})
    robot.append(ET.Element("link", {"name": frame}))

    for material in source_root.findall("material"):
        robot.append(copy.deepcopy(material))

    for side in SIDES:
        prefix = PREFIX[side]
        for child in source_root:
            if child.tag in ("link", "joint"):
                robot.append(_prefix_child(child, prefix))
        xyz, rpy = mounts["mounts"][side]
        robot.append(
            _fixed_joint(
                f"{prefix}world_joint",
                frame,
                f"{prefix}base_link",
                xyz,
                rpy,
            )
        )
        if tool["type"] == "cylinder":
            _add_tool(robot, prefix, tool)

    return _xml(robot)


def _primary_chain_group(source: ET.Element) -> ET.Element:
    """返回单臂 SRDF 里唯一的 chain 组；多余组先拒绝，避免静默丢语义。"""
    groups = source.findall("group")
    if not groups:
        raise ModelError("source SRDF has no <group>")
    chain_groups = [group for group in groups if group.find("chain") is not None]
    if len(chain_groups) != 1:
        raise ModelError(
            "source SRDF must contain exactly one group with a <chain>; "
            "extend the generator before adding more arm groups"
        )
    group = chain_groups[0]
    if len(groups) != 1:
        raise ModelError(
            "source SRDF has groups beyond the single arm chain; "
            "extend the generator before using them"
        )
    chain = group.find("chain")
    if not chain.get("base_link") or not chain.get("tip_link"):
        raise ModelError("source SRDF chain misses base_link or tip_link")
    return group


def build_dual_srdf(
    source: ET.Element,
    mounts: Mapping,
    *,
    robot_name: str = DEFAULT_ROBOT_NAME,
) -> str:
    frame = mounts["frame"]
    tool = mounts["tool"]
    group = _primary_chain_group(source)
    chain = group.find("chain")
    base_link = chain.get("base_link")
    tip_link = chain.get("tip_link")
    group_name = group.get("name")

    states = [
        state
        for state in source.findall("group_state")
        if state.get("group") == group_name
    ]
    other_states = [
        state
        for state in source.findall("group_state")
        if state.get("group") != group_name
    ]
    if other_states:
        raise ModelError(
            "source SRDF has group_state entries for a non-arm group; "
            "extend the generator before using them"
        )

    disables = source.findall("disable_collisions")

    robot = ET.Element("robot", {"name": robot_name})

    for side in SIDES:
        prefix = PREFIX[side]
        arm_group = ET.SubElement(robot, "group", {"name": f"{side}_arm"})
        arm_tip = (
            f"{prefix}tool_link"
            if tool["type"] == "cylinder"
            else f"{prefix}{tip_link}"
        )
        ET.SubElement(
            arm_group,
            "chain",
            {"base_link": f"{prefix}{base_link}", "tip_link": arm_tip},
        )
        for state in states:
            state_name = state.get("name")
            if not state_name:
                raise ModelError("source SRDF has a group_state without a name")
            target = ET.SubElement(
                robot,
                "group_state",
                {"name": f"{side}_{state_name}", "group": f"{side}_arm"},
            )
            for joint in state.findall("joint"):
                joint_name = joint.get("name")
                if not joint_name:
                    raise ModelError("source SRDF group_state has a joint without a name")
                ET.SubElement(
                    target,
                    "joint",
                    {
                        "name": f"{prefix}{joint_name}",
                        "value": joint.get("value", "0"),
                    },
                )

    both = ET.SubElement(robot, "group", {"name": "both_arms"})
    ET.SubElement(both, "group", {"name": "left_arm"})
    ET.SubElement(both, "group", {"name": "right_arm"})

    # 合并左右同名的命名状态，例如 both_zero / both_ready。两臂共用单臂
    # SRDF 的状态定义，只是关节名前缀不同。
    state_by_name = {state.get("name"): state for state in states}
    for state_name in sorted(state_by_name):
        target = ET.SubElement(
            robot,
            "group_state",
            {"name": f"both_{state_name}", "group": "both_arms"},
        )
        for side in SIDES:
            prefix = PREFIX[side]
            for joint in state_by_name[state_name].findall("joint"):
                ET.SubElement(
                    target,
                    "joint",
                    {
                        "name": f"{prefix}{joint.get('name')}",
                        "value": joint.get("value", "0"),
                    },
                )

    for side in SIDES:
        prefix = PREFIX[side]
        ET.SubElement(
            robot,
            "end_effector",
            {
                "name": f"{side}_ee",
                "parent_link": f"{prefix}ee_link",
                "group": f"{side}_arm",
            },
        )

    for side in SIDES:
        prefix = PREFIX[side]
        ET.SubElement(
            robot,
            "disable_collisions",
            {
                "link1": frame,
                "link2": f"{prefix}{base_link}",
                "reason": "Default",
            },
        )
        for disable in disables:
            link1 = disable.get("link1")
            link2 = disable.get("link2")
            if not link1 or not link2:
                raise ModelError("source SRDF disable_collisions misses a link name")
            ET.SubElement(
                robot,
                "disable_collisions",
                {
                    "link1": f"{prefix}{link1}",
                    "link2": f"{prefix}{link2}",
                    "reason": disable.get("reason", "Default"),
                },
            )
        if tool["type"] == "cylinder":
            ET.SubElement(
                robot,
                "disable_collisions",
                {
                    "link1": f"{prefix}ee_link",
                    "link2": f"{prefix}tool_link",
                    "reason": "Adjacent",
                },
            )

    return _xml(robot)


def _default_path(package: str, relative: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory(package), relative)


def default_source_urdf() -> str:
    return _default_path("litearm_description", "urdf/litearm_description.urdf")


def default_source_srdf() -> str:
    return _default_path("litearm_moveit_config", "config/litearm.srdf")


def default_mounts() -> str:
    return _default_path("litearm_dual_arm", "config/dual_arm_mounts.yaml")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=("urdf", "srdf", "both"),
        default="both",
        help="which description to print (default: both)",
    )
    parser.add_argument("--source-urdf", default="", help="single-arm URDF path")
    parser.add_argument("--source-srdf", default="", help="single-arm SRDF path")
    parser.add_argument("--mounts", default="", help="dual_arm_mounts.yaml path")
    parser.add_argument("--robot-name", default=DEFAULT_ROBOT_NAME)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        source_urdf_path = args.source_urdf or default_source_urdf()
        source_srdf_path = args.source_srdf or default_source_srdf()
        mounts_path = args.mounts or default_mounts()
        mounts = load_mounts(mounts_path)

        if args.mode in ("urdf", "both"):
            source_root = ET.parse(source_urdf_path).getroot()
            sys.stdout.write(
                build_dual_urdf(source_root, mounts, robot_name=args.robot_name)
            )
        if args.mode == "both":
            sys.stdout.write("\n")
        if args.mode in ("srdf", "both"):
            source = ET.parse(source_srdf_path).getroot()
            sys.stdout.write(
                build_dual_srdf(source, mounts, robot_name=args.robot_name)
            )
    except (OSError, ET.ParseError, ModelError, yaml.YAMLError) as error:
        print(f"generate_dual_arm_model: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
