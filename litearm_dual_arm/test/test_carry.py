"""同步搬运配置与纯函数测试。"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dual_carry_demo import (
    _build_synchronized_trajectory,
    _carry_poses,
    _geometry,
    _interpolate_path,
    _quaternion_error_rad,
    load_carry_config,
)
from dual_arm_task_client import ALL_JOINTS, LEFT_JOINTS, RIGHT_JOINTS


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "dual_carry.yaml"


def test_carry_config_is_well_formed():
    config = load_carry_config(str(CONFIG))
    assert config["frame"] == "world"
    assert config["hand_separation_m"] == pytest.approx(0.20)
    assert config["distance_m"] == pytest.approx(0.08)
    assert config["start_center"] == pytest.approx([0.0, 0.05, 0.50])
    assert config["motion"]["samples"] >= 2
    assert len(config["start_left_rad"]) == 7
    assert len(config["start_right_rad"]) == 7
    length = math.sqrt(sum(value * value for value in config["left_xyzw"]))
    assert length == pytest.approx(1.0)


def test_carry_poses_place_hands_symmetrically():
    config = load_carry_config(str(CONFIG))
    poses = _carry_poses(config)
    assert poses["left_start"] == pytest.approx([0.1, 0.05, 0.50])
    assert poses["right_start"] == pytest.approx([-0.1, 0.05, 0.50])
    assert poses["left_end"] == pytest.approx([0.18, 0.05, 0.50])
    assert poses["right_end"] == pytest.approx([-0.02, 0.05, 0.50])
    assert poses["end_center"] == pytest.approx([0.08, 0.05, 0.50])


def test_quaternion_error_is_zero_for_equal_and_pi_for_opposite():
    quat = (0.0, 0.0, 0.0, 1.0)
    assert _quaternion_error_rad(quat, quat) == pytest.approx(0.0)
    assert _quaternion_error_rad(quat, (0.0, 0.0, 0.0, -1.0)) == pytest.approx(
        0.0
    )
    assert _quaternion_error_rad(
        quat, (1.0, 0.0, 0.0, 0.0)
    ) == pytest.approx(math.pi)


def test_interpolate_path_midpoint():
    trajectory = JointTrajectory()
    trajectory.joint_names = list(LEFT_JOINTS)
    first = JointTrajectoryPoint()
    first.positions = [0.0] * 7
    second = JointTrajectoryPoint()
    second.positions = [1.0] * 7
    trajectory.points = [first, second]
    assert _interpolate_path(trajectory, 0.0) == pytest.approx([0.0] * 7)
    assert _interpolate_path(trajectory, 0.5) == pytest.approx([0.5] * 7)
    assert _interpolate_path(trajectory, 1.0) == pytest.approx([1.0] * 7)


def test_synchronized_trajectory_has_all_joints_and_monotonic_time():
    left = [[0.0] * 7, [0.1] * 7]
    right = [[0.2] * 7, [0.3] * 7]
    trajectory = _build_synchronized_trajectory(left, right, 4.0)
    assert trajectory.joint_names == list(ALL_JOINTS)
    assert len(trajectory.points) == 2
    assert trajectory.points[0].time_from_start.sec == 0
    assert trajectory.points[1].time_from_start.sec == 4
    assert trajectory.points[1].positions[:7] == pytest.approx([0.1] * 7)
    assert trajectory.points[1].positions[7:] == pytest.approx([0.3] * 7)


def _pose_at(xyz, quat):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose


def test_geometry_matches_expected_start():
    config = load_carry_config(str(CONFIG))
    fk = {
        "left_ee_link": _pose_at(
            [0.1, 0.05, 0.50], config["left_xyzw"]
        ),
        "right_ee_link": _pose_at(
            [-0.1, 0.05, 0.50], config["right_xyzw"]
        ),
    }
    geometry = _geometry(fk, config, 0.0)
    assert geometry["separation_m"] == pytest.approx(0.20)
    assert geometry["perpendicular_error_m"] == pytest.approx(0.0)
    assert geometry["plane_difference_m"] == pytest.approx(0.0)
    assert geometry["left_position_error_m"] == pytest.approx(0.0)
    assert geometry["right_position_error_m"] == pytest.approx(0.0)
    assert geometry["left_orientation_error_rad"] == pytest.approx(0.0)
    assert geometry["right_orientation_error_rad"] == pytest.approx(0.0)
