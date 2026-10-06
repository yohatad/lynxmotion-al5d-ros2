"""CoraPlex tests: the actions run as plans on the AL5D in a CoraPlex world.

They need CoraPlex (the cognitive_robot_abstract_machine monorepo) and the sourced ROS 2
workspace of the course; conftest.py leaves them out when CoraPlex is not installed.
"""

from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import execute_single, sequential

from module11.actions import (
    ApproachAction, DemoAction, MoveWristAction, PickAction, PickAndPlaceAction, PlaceAction,
    SetGripperDistanceAction)
from module11.al5d import joint_positions, make_context
from module11.kinematics import (
    forward_kinematics, GRIPPER_CLOSED, GRIPPER_OPEN, inverse_kinematics, UnreachablePose,
    WristPose)
import pytest


@pytest.fixture(scope='module')
def world_and_context():
    context, world = make_context()
    return world, context


def run(action, context):
    with simulated_robot:
        execute_single(action, context=context).perform()


def test_world_contains_the_arm(world_and_context):
    world, _ = world_and_context
    positions = joint_positions(world)
    assert set(positions) == {'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Gripper'}


def test_move_wrist_reaches_the_joint_targets(world_and_context):
    world, context = world_and_context
    action = MoveWristAction(0.0, 200.0, 150.0)
    run(action, context)
    positions = joint_positions(world)
    target = inverse_kinematics(WristPose(0.0, 200.0, 150.0))
    for name, expected in zip(('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5'), target):
        assert positions[name] == pytest.approx(expected, abs=2e-3), name
    assert forward_kinematics([positions[n] for n in ('Joint1', 'Joint2', 'Joint3')]) == \
        pytest.approx((0.0, 200.0, 150.0), abs=1.0)


@pytest.mark.parametrize('distance_mm', [0.0, 15.0, 30.0])
def test_gripper_distance_is_reached_precisely(world_and_context, distance_mm):
    world, context = world_and_context
    run(SetGripperDistanceAction(distance_mm), context)
    assert joint_positions(world)['Gripper'] == pytest.approx(distance_mm / 1000.0, abs=1.5e-3)


def test_fingers_follow_the_gripper_joint(world_and_context):
    world, context = world_and_context
    fingers = {c.name.name: c for c in world.connections
               if c.name.name in ('right_finger_joint', 'left_finger_joint')}
    run(SetGripperDistanceAction(GRIPPER_OPEN), context)
    open_right = float(fingers['right_finger_joint'].position)
    run(SetGripperDistanceAction(GRIPPER_CLOSED), context)
    closed_right = float(fingers['right_finger_joint'].position)
    # the URDF mimic relation: right finger = 0.015875 - 0.5 * Gripper
    assert closed_right - open_right == pytest.approx(0.5 * (GRIPPER_OPEN - GRIPPER_CLOSED) / 1000,
                                                      abs=1e-3)


def test_unreachable_move_fails_before_anything_moves(world_and_context):
    world, context = world_and_context
    before = joint_positions(world)
    with pytest.raises(UnreachablePose):
        run(MoveWristAction(0.0, 600.0, 100.0), context)
    assert joint_positions(world) == before


def test_wrist_outside_the_working_envelope_is_rejected(world_and_context):
    _, context = world_and_context
    with pytest.raises(UnreachablePose, match='envelope'):
        run(MoveWristAction(200.0, 200.0, 100.0), context)


def test_pick_has_the_expected_steps():
    steps = PickAction(-60.0, 175.0, 0.0, 90.0).steps()
    kinds = [type(step).__name__ for step in steps]
    assert kinds == ['SetGripperDistanceAction', 'MoveWristAction', 'MoveWristAction',
                     'SetGripperDistanceAction', 'MoveWristAction']
    assert steps[0].distance == GRIPPER_OPEN
    assert steps[3].distance == GRIPPER_CLOSED
    # the grasp pose is lower than the approach poses, which are identical
    assert steps[2].z < steps[1].z == steps[4].z


def test_place_has_the_expected_steps():
    kinds = [type(s).__name__ for s in PlaceAction(60.0, 175.0, 0.0).steps()]
    assert kinds == ['MoveWristAction', 'MoveWristAction', 'SetGripperDistanceAction',
                     'MoveWristAction']


def test_pick_and_place_is_pick_followed_by_place():
    action = PickAndPlaceAction(-60, 175, 0, 60, 175, 0, 90, 0)
    expected = PickAction(-60, 175, 0, 90).steps() + PlaceAction(60, 175, 0, 0).steps()
    assert action.steps() == expected


def test_gripper_roll_is_the_negative_of_the_brick_orientation():
    assert ApproachAction(0, 200, 0, 90.0).steps()[0].roll == -90.0
    assert ApproachAction(0, 200, 0, 0.0).steps()[0].roll == 0.0


def test_pick_and_place_runs_as_a_plan(world_and_context):
    world, context = world_and_context
    run(PickAndPlaceAction(-60.0, 175.0, 0.0, 60.0, 175.0, 0.0, 90.0, 0.0), context)
    positions = joint_positions(world)
    # last step of a place: the arm is above the destination (base turned right) and the gripper
    # is open
    assert positions['Joint1'] == pytest.approx(0.33, abs=0.01)
    assert positions['Gripper'] == pytest.approx(GRIPPER_OPEN / 1000.0, abs=1.5e-3)


def test_each_step_can_be_replayed_one_by_one(world_and_context):
    world, context = world_and_context
    positions_after_each = []
    for step in PickAction(-60.0, 175.0, 0.0, 90.0).steps():
        run(step, context)
        positions_after_each.append(joint_positions(world))
    assert positions_after_each[0]['Gripper'] == pytest.approx(0.030, abs=1.5e-3)
    assert positions_after_each[3]['Gripper'] == pytest.approx(0.015, abs=1.5e-3)
    # the base joint does not move while only the gripper changes
    assert positions_after_each[3]['Joint1'] == pytest.approx(
        positions_after_each[2]['Joint1'], abs=1e-3)


def test_plans_compose_with_the_plan_language(world_and_context):
    world, context = world_and_context
    with simulated_robot:
        sequential([DemoAction(), SetGripperDistanceAction(GRIPPER_CLOSED)],
                   context=context).perform()
    assert joint_positions(world)['Gripper'] == pytest.approx(0.015, abs=1.5e-3)


def test_demo_steps_are_all_reachable():
    for step in DemoAction().steps():
        step.joint_targets()
