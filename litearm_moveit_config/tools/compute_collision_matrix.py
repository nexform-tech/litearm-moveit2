#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compute_collision_matrix.py — generate the SRDF disable_collisions list by sampling.

Why this is needed
------------------
The MoveIt SRDF has to tell MoveIt "which link pairs don't need a self-collision
check". The two ways of getting it wrong are not equally costly:

* **Missing a disable** (leaving a permanently touching link pair checked) →
  MoveIt considers the start state to be in self-collision, every plan fails
  outright, and the configuration is completely unusable.
* **One disable too many** (turning off a pair that comes close but never
  touches) → one layer of protection less.

This script therefore errs on the conservative side with its thresholds (20 mm
by default) and prints every disabled link pair together with the "sampled
minimum distance", so it can be reviewed by hand.

Algorithm
---------
It's exactly what moveit_setup_assistant does, just implemented with
numpy/scipy:

1. Parse the URDF: link chain, joint origins, collision meshes and their origins
2. Read the binary STL, deduplicate the vertices and thin them by a stride
   (default ≤ 12000 points per part)
3. Run forward kinematics at the zero pose + a number of random valid
   configurations
4. For each link pair, transform A's vertices into the world frame and use a
   cKDTree to find their nearest distance to each of B's vertices
5. If the minimum distance under any sampled configuration is < the threshold,
   the pair is disabled

Usage::

    # Default: use ament_index to locate the installed litearm package (no
    # absolute paths involved)
    python3 compute_collision_matrix.py --output srdf_snippet.xml

    # Or say it explicitly
    python3 compute_collision_matrix.py \\
        --urdf <litearm.urdf> --mesh-root <package dir holding meshes/> \\
        --output srdf_snippet.xml

The output is a <disable_collisions> snippet you can paste straight into the SRDF.
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree


def _default_share_dir() -> str:
    """Locate the litearm description package via ament_index.

    Equivalent to $(find litearm) in xacro.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:  # pragma: no cover - environment problem, not a logic branch
        raise SystemExit(
            f"ament_index_python is required, source your ROS environment "
            f"first: {exc}")
    try:
        return get_package_share_directory("litearm")
    except Exception as exc:  # pragma: no cover
        raise SystemExit(
            f"ament_index cannot find the litearm package (did you colcon build "
            f"and source install/setup.bash?): {exc}")

# URDF rpy convention: extrinsic XYZ fixed-axis rotation, i.e. R = Rz(y)Ry(p)Rx(r)
def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = axis / norm
    kx, ky, kz = k
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return np.array([
        [c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v],
    ])


def read_binary_stl(path: str, max_points: int) -> np.ndarray:
    """Read a binary STL, returning deduplicated/thinned vertices (N,3) float64."""
    with open(path, "rb") as handle:
        header = handle.read(80)
        if header[:5].lower() == b"solid" and not header[80:84]:
            pass  # a "solid" header may also be binary; decide by triangle count
        raw_count = handle.read(4)
        if len(raw_count) < 4:
            raise ValueError(f"{path}: file too short to be a valid STL")
        triangles = int(np.frombuffer(raw_count, dtype="<u4")[0])
        payload = handle.read(triangles * 50)
    if len(payload) < triangles * 50:
        raise ValueError(f"{path}: binary STL data is incomplete")
    record = np.frombuffer(payload, dtype=np.uint8).reshape(triangles, 50)
    # Each record: normal (3×f4) + 3 vertices (9×f4) + attribute (2B)
    floats = record[:, :48].copy().view("<f4").reshape(triangles, 12)
    vertices = floats[:, 3:12].reshape(-1, 3).astype(np.float64)

    vertices = np.unique(vertices, axis=0)
    if max_points > 0 and len(vertices) > max_points:
        step = int(math.ceil(len(vertices) / max_points))
        vertices = vertices[::step]
    return vertices


class LinkGeometry:
    """A link's collision point cloud and its pose in the link frame."""

    def __init__(self, name: str, points: np.ndarray, origin: np.ndarray) -> None:
        self.name = name
        self.points = points          # (N,3) in the link frame
        self.origin = origin          # 4x4, collision origin relative to link frame


class UrdfModel:
    """Just enough URDF parsing + forward kinematics for serial chains.

    Only revolute/continuous/fixed joints are handled.
    """

    def __init__(self, urdf_path: str, mesh_root: str, max_points: int) -> None:
        root = ET.parse(urdf_path).getroot()
        self.mesh_root = mesh_root.rstrip("/")

        self.links: Dict[str, LinkGeometry] = {}
        for link in root.findall("link"):
            name = link.get("name")
            collision = link.find("collision")
            if collision is None:
                continue
            geometry = collision.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None:
                # non-mesh collision bodies (box/cylinder) are unused here; just skip
                continue
            filename = mesh.get("filename")
            local = filename.split("package://")[-1]
            local = local.split("/", 1)[1] if "/" in local else local
            path = f"{self.mesh_root}/{local}"
            points = read_binary_stl(path, max_points)
            origin_el = collision.find("origin")
            origin = self._origin_matrix(origin_el)
            self.links[name] = LinkGeometry(name, points, origin)

        self.joints = []      # (name, parent, child, origin4x4, axis, type)
        self.child_joint: Dict[str, Tuple[str, np.ndarray, np.ndarray, str]] = {}
        for joint in root.findall("joint"):
            jtype = joint.get("type")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = self._origin_matrix(joint.find("origin"))
            axis_el = joint.find("axis")
            axis = (np.array([float(v) for v in axis_el.get("xyz").split()])
                    if axis_el is not None else np.array([1.0, 0.0, 0.0]))
            self.joints.append((joint.get("name"), parent, child, origin, axis, jtype))
            if jtype in ("revolute", "continuous"):
                index = self._revolute_index(joint.get("name"))
                self.child_joint[child] = (joint.get("name"), parent, origin, axis, index, jtype)
            else:
                self.child_joint[child] = (joint.get("name"), parent, origin, axis, -1, jtype)

        self.root_link = self._find_root(root)
        # Order along the chain (from the root) so parent poses are computed first
        self.order = self._chain_order()

    def _revolute_index(self, joint_name: str) -> int:
        """jointN → array index N-1."""
        digits = "".join(ch for ch in joint_name if ch.isdigit())
        return int(digits) - 1

    @staticmethod
    def _origin_matrix(origin_el: Optional[ET.Element]) -> np.ndarray:
        matrix = np.eye(4)
        if origin_el is None:
            return matrix
        xyz = origin_el.get("xyz", "0 0 0")
        rpy = origin_el.get("rpy", "0 0 0")
        matrix[:3, 3] = [float(v) for v in xyz.split()]
        matrix[:3, :3] = rpy_to_matrix(*[float(v) for v in rpy.split()])
        return matrix

    @staticmethod
    def _find_root(root: ET.Element) -> str:
        children = {j.find("child").get("link") for j in root.findall("joint")}
        for link in root.findall("link"):
            if link.get("name") not in children:
                return link.get("name")
        raise ValueError("URDF has no root link")

    def _chain_order(self) -> List[str]:
        order, stack = [], [self.root_link]
        while stack:
            link = stack.pop(0)
            order.append(link)
            for child, (_jn, parent, *_rest) in self.child_joint.items():
                if parent == link:
                    stack.append(child)
        return order

    def link_poses(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        """Given joint angles, return each link's 4x4 pose in the world (root) frame."""
        poses = {self.root_link: np.eye(4)}
        for link in self.order[1:]:
            _name, parent, origin, axis, index, jtype = self.child_joint[link]
            parent_pose = poses[parent]
            joint_pose = origin.copy()
            if jtype in ("revolute", "continuous") and index >= 0:
                angle = float(q[index]) if index < len(q) else 0.0
                rotation = np.eye(4)
                rotation[:3, :3] = axis_angle_to_matrix(axis, angle)
                joint_pose = origin @ rotation
            poses[link] = parent_pose @ joint_pose
        return poses

    def world_points(self, link: str, q: np.ndarray,
                     poses: Optional[Dict[str, np.ndarray]] = None) -> np.ndarray:
        """The link's collision point cloud in world-frame coordinates."""
        poses = poses if poses is not None else self.link_poses(q)
        geometry = self.links[link]
        transform = poses[link] @ geometry.origin
        pts = geometry.points
        return pts @ transform[:3, :3].T + transform[:3, 3]


def sample_configurations(rng: np.random.Generator, limits: np.ndarray,
                          count: int) -> List[np.ndarray]:
    """Zero pose plus random valid configurations.

    The zero pose must always be included: the SRDF's Default decision relies on it.
    """
    configs = [np.zeros(limits.shape[0])]
    if count > 1:
        low, high = limits[:, 0], limits[:, 1]
        configs.extend(low + (high - low) * rng.random((count - 1, limits.shape[0])))
    return configs


def joint_limits(urdf_path: str) -> Tuple[List[str], np.ndarray]:
    root = ET.parse(urdf_path).getroot()
    names, limits = [], []
    for joint in root.findall("joint"):
        if joint.get("type") not in ("revolute", "continuous"):
            continue
        name = joint.get("name")
        limit = joint.find("limit")
        if limit is None:
            lower, upper = -math.pi, math.pi
        else:
            lower = float(limit.get("lower", -math.pi))
            upper = float(limit.get("upper", math.pi))
            if joint.get("type") == "continuous":
                lower, upper = -math.pi, math.pi
        names.append(name)
        limits.append((lower, upper))
    # Sort by jointN to match the index convention used by the forward kinematics
    order = sorted(range(len(names)), key=lambda i: names[i])
    return [names[i] for i in order], np.array([limits[i] for i in order])


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", default="",
                        help="path to the URDF; leave empty to locate the "
                             "installed litearm package via ament_index")
    parser.add_argument("--mesh-root", default="",
                        help="local directory corresponding to "
                             "package://<pkg>/ (i.e. the level holding meshes/, "
                             "not meshes/ itself); leave empty to locate the "
                             "installed litearm package via ament_index")
    parser.add_argument("--samples", type=int, default=120,
                        help="number of random configurations to sample "
                             "(zero pose included, default 120)")
    parser.add_argument("--max-points", type=int, default=12000,
                        help="vertex thinning cap per link (default 12000)")
    parser.add_argument("--touch-epsilon", type=float, default=0.002,
                        help="nearest-distance threshold (m) for deciding that "
                             "a pair touches in all configurations, default "
                             "0.002. Under the point-cloud approximation, "
                             "measured values for touching pairs are usually "
                             "<1mm")
    parser.add_argument("--default-epsilon", type=float, default=0.005,
                        help="nearest-distance threshold (m) for deciding that "
                             "a pair touches at the zero pose, default 0.005. "
                             "Leaving such pairs enabled makes MoveIt consider "
                             "the start state to be in self-collision")
    parser.add_argument("--seed", type=int, default=20260917,
                        help="random seed for sampling")
    parser.add_argument("--output", default="",
                        help="output file; leave empty to print to stdout")
    args = parser.parse_args(argv)

    if not args.urdf or not args.mesh_root:
        default_share = _default_share_dir()
        args.urdf = args.urdf or os.path.join(default_share, "urdf", "litearm.urdf")
        args.mesh_root = args.mesh_root or default_share

    model = UrdfModel(args.urdf, args.mesh_root, args.max_points)
    names, limits = joint_limits(args.urdf)
    print(f"# URDF: {args.urdf}", file=sys.stderr)
    print(f"# links (with collision bodies): {sorted(model.links)}", file=sys.stderr)
    print(f"# joints: {names}", file=sys.stderr)
    print(f"# sampling {args.samples} configurations (zero pose included); "
          f"touch thresholds "
          f"Always<{args.touch_epsilon * 1000:.0f}mm / "
          f"Default<{args.default_epsilon * 1000:.0f}mm; "
          f"≤{args.max_points} points per link", file=sys.stderr)

    rng = np.random.default_rng(args.seed)
    configs = sample_configurations(rng, limits, args.samples)

    # Pre-compute the world point cloud for every sampled configuration
    cache: List[Dict[str, np.ndarray]] = []
    for index, q in enumerate(configs):
        poses = model.link_poses(q)
        cache.append({link: model.world_points(link, q, poses)
                      for link in model.links})
        if (index + 1) % 20 == 0:
            print(f"#   configuration {index + 1}/{len(configs)}", file=sys.stderr)

    links = [l for l in model.order if l in model.links]

    # Pre-build and cache one KD-tree per sampled configuration: each link's tree
    # is reused by many link pairs, and rebuilding it inside the pairing loop
    # would cost O(pairs × samples) tree builds — measured over 3x slower.
    trees: List[Dict[str, Tuple[np.ndarray, cKDTree]]] = [
        {link: (cloud[link], cKDTree(cloud[link])) for link in links}
        for cloud in cache
    ]

    # Structurally adjacent pairs (directly connected by the same joint): the
    # collision meshes naturally interpenetrate at the joint, which is a property
    # of how the model is built and independent of the configuration, so always
    # disable them.
    adjacent = set()
    for _name, parent, child, _origin, _axis, _jtype in model.joints:
        if parent in model.links and child in model.links:
            adjacent.add(tuple(sorted((parent, child))))

    results = []  # (a, b, min_over_samples, min_at_zero, contact_samples, is_adjacent)
    total_samples = len(configs)
    for i, a in enumerate(links):
        for b in links[i + 1:]:
            best_all = math.inf
            best_zero = math.inf
            contact = 0
            for index, entry in enumerate(trees):
                pts_a, _ = entry[a]
                _pts_b, tree_b = entry[b]
                distance, _ = tree_b.query(pts_a, k=1, workers=-1)
                minimum = float(distance.min())
                if index == 0:
                    best_zero = minimum
                if minimum < args.touch_epsilon:
                    contact += 1
                if minimum < best_all:
                    best_all = minimum
            results.append((a, b, best_all, best_zero, contact,
                            tuple(sorted((a, b))) in adjacent))

    # Three categories, matching the classification semantics of
    # moveit_setup_assistant:
    #
    #   Adjacent — structurally adjacent, interpenetrating by construction, disable
    #   Always   — touching/interpenetrating in **every** sampled configuration, disable
    #   Default  — touching/interpenetrating already at the zero pose (e.g. two
    #              shell sections sitting flush at zero), disable, otherwise MoveIt
    #              considers the start state to be in self-collision and every plan
    #              fails
    #
    # Key point: the criterion is the "number of touching samples", not the
    # "minimum sampled distance". A small minimum distance only means that under
    # **some** configurations the two shell sections do come into contact, and
    # that is exactly the region MoveIt needs to check and avoid — disabling
    # based on minimum distance would turn that protection off.
    adjacent_pairs, always_pairs, default_pairs, kept = [], [], [], []
    for a, b, best_all, best_zero, contact, is_adjacent in results:
        if is_adjacent:
            adjacent_pairs.append((a, b))
        elif contact == total_samples:
            always_pairs.append((a, b, best_all))
        elif best_zero < args.default_epsilon:
            default_pairs.append((a, b, best_zero))
        else:
            kept.append((a, b, best_all, best_zero, contact, False))

    lines = [
        "<!-- Generated by tools/compute_collision_matrix.py; do not edit by hand.",
        f"     Method: nearest point-cloud distance over {len(configs)} "
        f"configurations (zero pose + random);",
        f"     at most {args.max_points} thinned vertices per link.",
        f"     Rules: Adjacent=structural interpenetration; "
        f"Always=every configuration <{args.touch_epsilon * 1000:.0f}mm;",
        f"          Default=zero pose <{args.default_epsilon * 1000:.0f}mm.",
        "     Pairs that merely come close or touch in some configurations are NOT",
        "     disabled — those are exactly the regions MoveIt needs to check and avoid.",
        "     This is a point-cloud approximation, not an exact mesh distance; "
        "regenerate after changing the URDF. -->",
    ]
    for a, b in sorted(adjacent_pairs):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" '
                     f'reason="Adjacent"/>')
    for a, b, distance in sorted(always_pairs, key=lambda item: item[2]):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" '
                     f'reason="Always (MinDistance {distance * 1000:.1f}mm)"/>')
    for a, b, distance in sorted(default_pairs, key=lambda item: item[2]):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" '
                     f'reason="Default (zero-pose MinDistance '
                     f'{distance * 1000:.1f}mm)"/>')

    text = "\n".join(lines) + "\n"
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text)
        print(f"# wrote {args.output}", file=sys.stderr)
    else:
        print(text)

    total_disabled = len(adjacent_pairs) + len(always_pairs) + len(default_pairs)
    print(f"# {len(results)} candidate pairs: {len(adjacent_pairs)} adjacent, "
          f"{len(always_pairs)} touching in every configuration, "
          f"{len(default_pairs)} touching at the zero pose; "
          f"{total_disabled} disabled, {len(kept)} left checked", file=sys.stderr)
    if kept:
        print(f"# pairs left checked ({len(kept)} total; sorted by number of "
              f"configurations they ever touched, descending; top 10):",
              file=sys.stderr)
        for a, b, best_all, _best_zero, contact, _adj in sorted(
                kept, key=lambda i: (-i[4], i[2]))[:10]:
            print(f"#   {a:10s} {b:10s} touching configs {contact:3d}/{total_samples} "
                  f"min distance {best_all * 1000:7.1f}mm", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
