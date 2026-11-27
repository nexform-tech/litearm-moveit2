"""执行桥与仿真脚本的纯函数测试。"""

from __future__ import annotations

import pytest
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dual_arm_bridge import ARMS, BASE_JOINTS, DualArmBridge, _seconds
from dual_arm_sim import ALL_JOINTS, parse_positions


def _trajectory(side: str):
    trajectory = JointTrajectory()
    trajectory.joint_names = list(ARMS[side])
    point = JointTrajectoryPoint()
    point.positions = [0.01 * (index + 1) for index in range(7)]
    point.velocities = [0.1] * 7
    point.accelerations = [0.2] * 7
    point.time_from_start = Duration(sec=1, nanosec=500000000)
    trajectory.points = [point]
    return trajectory


def test_upstream_goal_maps_prefixed_names_to_native_names():
    goal = DualArmBridge._upstream_goal("left", _trajectory("left"))
    assert goal.trajectory.joint_names == list(BASE_JOINTS)
    assert goal.trajectory.points[0].positions == pytest.approx(
        [0.01 * (index + 1) for index in range(7)]
    )
    assert goal.trajectory.points[0].velocities == pytest.approx([0.1] * 7)
    assert goal.trajectory.points[0].accelerations == pytest.approx([0.2] * 7)
    assert _seconds(goal.trajectory.points[0].time_from_start) == pytest.approx(1.5)


def test_upstream_goal_rejects_missing_joint():
    trajectory = _trajectory("right")
    trajectory.joint_names = trajectory.joint_names[:-1]
    trajectory.points[0].positions = trajectory.points[0].positions[:-1]
    with pytest.raises(ValueError):
        DualArmBridge._upstream_goal("right", trajectory)


def test_parse_positions_defaults_to_all_zero():
    values = parse_positions("")
    assert values == [0.0] * len(ALL_JOINTS)


def test_parse_positions_requires_fourteen_values():
    with pytest.raises(ValueError, match="14"):
        parse_positions("0,0,0")
