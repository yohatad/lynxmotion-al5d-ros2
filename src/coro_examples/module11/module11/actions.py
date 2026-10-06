"""Designators for the AL5D: the CoraPlex version of the Lisp CRAM AL5D tutorial.

The ROS 1 tutorial was written in Common Lisp for the original CRAM. CoraPlex expresses the
same ideas in Python. This table shows where each Lisp concept went:

=======================================  =====================================================
CRAM (Lisp)                              CoraPlex (this module)
=======================================  =====================================================
``(a motion (type moving) ...)``         :class:`MoveWristAction` (built on
                                         :class:`~module11.motions.PreciseMoveJointsMotion`)
``(a motion (type grasping) ...)``       :class:`SetGripperDistanceAction`
``(an action (type approaching) ...)``   :class:`ApproachAction`
``(an action (type picking) ...)``       :class:`PickAction`
``(an action (type placing) ...)``       :class:`PlaceAction`
``(an action (type picking-and-placing)  :class:`PickAndPlaceAction`
...)``
``(an action (type demoing))``           :class:`DemoAction`
process module + ROS publisher           the plan runs on the world model; ``ros2_bridge``
                                         forwards the result to the simulator
=======================================  =====================================================

An *action description* has parameters (its fields) and builds a plan (``_action_plan``).
Plans combine actions and motions with ``sequential``, ``parallel``, ``repeat`` and so on.
Every action here also offers :meth:`steps`, the flat list of leaf steps (wrist moves and
gripper moves) it consists of, so that the steps can be inspected, tested and replayed one by
one on the simulator. Positions are in millimetres and angles in degrees, as in module 4.
"""

from dataclasses import dataclass

from coraplex.plans.factories import execute_single, sequential
from coraplex.robot_plans.actions.base import ActionDescription

from module11.kinematics import (
    ARM_JOINTS, GRIPPER_CLOSED, gripper_joint_value, GRIPPER_LENGTH, GRIPPER_OPEN,
    inverse_kinematics, UnreachablePose, within_working_envelope, WristPose)
from module11.motions import PreciseMoveJointsMotion

#: Height above the object at which the gripper is lowered for a grasp (mm)
GRASP_HEIGHT = 5.0
#: Distance above the grasp pose of the approach and depart poses (mm)
APPROACH_DISTANCE = 40.0


@dataclass
class SetGripperDistanceAction(ActionDescription):
    """Open or close the gripper until the fingers are ``distance`` mm apart."""

    distance: float
    """Distance between the finger tips in millimetres (0 to 31.75)."""

    def steps(self):
        return [self]

    @property
    def _action_plan(self):
        return execute_single(
            PreciseMoveJointsMotion(['Gripper'], [gripper_joint_value(self.distance)]))


@dataclass
class MoveWristAction(ActionDescription):
    """Move the wrist of the arm to ``(x, y, z)`` mm with the given gripper orientation."""

    x: float
    y: float
    z: float
    pitch: float = -180.0
    """0: gripper up, -180 (or 180): gripper down."""
    roll: float = 0.0

    def steps(self):
        return [self]

    def joint_targets(self):
        """Joint angles for this move; raises ``UnreachablePose`` when the arm cannot do it."""
        if not within_working_envelope(self.x, self.y, self.z):
            raise UnreachablePose(
                f'({self.x:.0f}, {self.y:.0f}, {self.z:.0f}) mm is outside the working envelope')
        return inverse_kinematics(WristPose(self.x, self.y, self.z, self.pitch, self.roll))

    @property
    def _action_plan(self):
        return execute_single(
            PreciseMoveJointsMotion(list(ARM_JOINTS), list(self.joint_targets())))


def _wrist_above(x, y, z, lift):
    """Wrist position for a gripper pointing down whose finger tips are at ``z + lift``."""
    return x, y, z + GRIPPER_LENGTH + lift


@dataclass
class ApproachAction(ActionDescription):
    """Move above the object until the finger tips are ``APPROACH_DISTANCE`` mm over it."""

    x: float
    y: float
    z: float
    phi: float = 0.0
    """Orientation of the brick about the vertical axis, in degrees (as in module 4)."""

    def steps(self):
        wx, wy, wz = _wrist_above(self.x, self.y, self.z, GRASP_HEIGHT + APPROACH_DISTANCE)
        return [MoveWristAction(wx, wy, wz, -180.0, -self.phi)]

    @property
    def _action_plan(self):
        return sequential(self.steps())


@dataclass
class PickAction(ActionDescription):
    """Pick up the brick at ``(x, y, z)`` mm: open, approach, lower, close, rise."""

    x: float
    y: float
    z: float
    phi: float = 0.0
    """Orientation of the brick about the vertical axis, in degrees (as in module 4)."""

    def steps(self):
        wx, wy, wz = _wrist_above(self.x, self.y, self.z, GRASP_HEIGHT)
        approach = ApproachAction(self.x, self.y, self.z, self.phi).steps()
        return (SetGripperDistanceAction(GRIPPER_OPEN).steps() + approach
                + [MoveWristAction(wx, wy, wz, -180.0, -self.phi)]
                + SetGripperDistanceAction(GRIPPER_CLOSED).steps() + approach)

    @property
    def _action_plan(self):
        return sequential(self.steps())


@dataclass
class PlaceAction(ActionDescription):
    """Put the brick held in the gripper down at ``(x, y, z)`` mm and rise again."""

    x: float
    y: float
    z: float
    phi: float = 0.0
    """Orientation of the brick about the vertical axis, in degrees (as in module 4)."""

    def steps(self):
        wx, wy, wz = _wrist_above(self.x, self.y, self.z, GRASP_HEIGHT)
        approach = ApproachAction(self.x, self.y, self.z, self.phi).steps()
        return (approach + [MoveWristAction(wx, wy, wz, -180.0, -self.phi)]
                + SetGripperDistanceAction(GRIPPER_OPEN).steps() + approach)

    @property
    def _action_plan(self):
        return sequential(self.steps())


@dataclass
class PickAndPlaceAction(ActionDescription):
    """Move a brick from ``(source_x, source_y, source_z)`` to the destination (mm)."""

    source_x: float
    source_y: float
    source_z: float
    destination_x: float
    destination_y: float
    destination_z: float
    source_phi: float = 0.0
    destination_phi: float = 0.0

    def steps(self):
        return (PickAction(self.source_x, self.source_y, self.source_z,
                           self.source_phi).steps()
                + PlaceAction(self.destination_x, self.destination_y, self.destination_z,
                              self.destination_phi).steps())

    @property
    def _action_plan(self):
        return sequential(self.steps())


@dataclass
class DemoAction(ActionDescription):
    """Show the arm: visit three poses and return, like the module 4 robotProgramming exercise."""

    def steps(self):
        return [
            MoveWristAction(0.0, 200.0, 180.0, -180.0, 0.0),
            MoveWristAction(-80.0, 200.0, 150.0, -180.0, 0.0),
            MoveWristAction(80.0, 200.0, 150.0, -180.0, 0.0),
            MoveWristAction(0.0, 200.0, 180.0, -180.0, 0.0),
        ]

    @property
    def _action_plan(self):
        return sequential(self.steps())
