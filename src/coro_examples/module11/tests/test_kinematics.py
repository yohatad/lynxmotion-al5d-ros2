"""Kinematics tests: pure Python, no ROS and no CoraPlex needed."""

import math

from module11.kinematics import (
    A3, A4, ARM_JOINTS, D1, forward_kinematics, gripper_joint_value, GRIPPER_LENGTH,
    inverse_kinematics, JOINT_LIMITS, UnreachablePose, within_working_envelope, WristPose)
import pytest


def reachable_grid():
    for x in range(-120, 121, 40):
        for y in range(100, 321, 55):
            for z in range(20, 361, 70):
                yield x, y, z


def test_inverse_then_forward_returns_the_wrist_position():
    checked = 0
    for x, y, z in reachable_grid():
        for pitch in (-180.0, 0.0):
            try:
                joints = inverse_kinematics(WristPose(x, y, z, pitch))
            except UnreachablePose:
                continue
            fx, fy, fz = forward_kinematics(joints)
            assert (fx, fy, fz) == pytest.approx((x, y, z), abs=1e-6)
            checked += 1
    assert checked > 30


@pytest.mark.parametrize('x, y, z', [(0, 600, 0), (400, 400, 300), (0, 0, D1), (0, 1, D1 + 1)])
def test_unreachable_positions_are_rejected(x, y, z):
    with pytest.raises(UnreachablePose):
        inverse_kinematics(WristPose(x, y, z))


def test_joint_limits_are_enforced():
    # The gripper cannot point straight down with the wrist this high: Joint4 would exceed pi/2.
    with pytest.raises(UnreachablePose, match='Joint4'):
        inverse_kinematics(WristPose(0, 200, 250, -180.0))


def test_every_returned_solution_is_within_the_joint_limits():
    for x, y, z in reachable_grid():
        try:
            joints = inverse_kinematics(WristPose(x, y, z, -180.0))
        except UnreachablePose:
            continue
        for name, value, (low, high) in zip(ARM_JOINTS, joints, JOINT_LIMITS):
            assert low - 1e-6 <= value <= high + 1e-6, name


def test_elbow_is_always_bent_the_same_way():
    for z in range(40, 341, 60):
        try:
            joints = inverse_kinematics(WristPose(0, 200, z, -180.0))
        except UnreachablePose:
            continue
        assert joints[2] < 0


def test_base_angle_follows_the_target_direction():
    assert inverse_kinematics(WristPose(0, 200, 150))[0] == pytest.approx(0.0)
    assert inverse_kinematics(WristPose(100, 100, 150))[0] == pytest.approx(math.pi / 4)
    assert inverse_kinematics(WristPose(-100, 100, 150))[0] == pytest.approx(-math.pi / 4)


def test_gripper_pointing_down_has_its_axis_pointing_down():
    # shoulder + elbow + wrist pitch = -90 degrees for a gripper that points straight down
    for x, y, z in [(0, 200, 110), (-60, 175, 110), (60, 175, 150)]:
        joints = inverse_kinematics(WristPose(x, y, z, -180.0))
        assert math.degrees(joints[1] + joints[2] + joints[3]) == pytest.approx(-90.0, abs=1e-6)


def test_matches_the_joint_commands_of_the_module4_program():
    # Joint commands the C++ pickAndPlace sent to the simulator for a brick at (-60, 175),
    # gripper pointing down 5 mm above the brick (wrist at 5 + 105 mm), brick turned by 90 degrees.
    joints = inverse_kinematics(WristPose(-60, 175, 5 + GRIPPER_LENGTH, -180.0, -90.0))
    assert joints[0] == pytest.approx(-0.33, abs=0.01)
    assert joints[1] == pytest.approx(1.37, abs=0.01)
    assert joints[2] == pytest.approx(-1.95, abs=0.01)
    assert joints[3] == pytest.approx(-0.99, abs=0.01)
    assert joints[4] == pytest.approx(0.33, abs=0.01)


def test_working_envelope():
    assert within_working_envelope(0, 200, 100)
    assert not within_working_envelope(200, 200, 100)
    assert not within_working_envelope(0, 50, 100)
    assert not within_working_envelope(0, 200, 500)


def test_gripper_value_is_clamped_and_in_metres():
    assert gripper_joint_value(15) == pytest.approx(0.015)
    assert gripper_joint_value(-5) == 0.0
    assert gripper_joint_value(100) == pytest.approx(0.03175)


def test_arm_lengths_match_module4():
    assert (A3, A4, D1) == (146.0, 187.0, 70.0)
