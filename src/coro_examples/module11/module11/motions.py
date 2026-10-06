"""A custom motion designator.

Motions are the lowest level of a CoraPlex plan: each one builds exactly one *goal* for the
motion statechart that moves the robot. The built-in ``MoveJointsMotion`` calls a goal "reached"
once every joint is within 0.01 (rad or m) of its target. That is fine for the revolute joints
of the arm but not for the gripper, whose whole range is 0.032 m: a request for 15 mm would
stop after about 7 mm. :class:`PreciseMoveJointsMotion` is the same motion with a tolerance
you choose, and shows how little code a new motion needs.
"""

from dataclasses import dataclass
from typing import List

from coraplex.robot_plans.motions.base import BaseMotion
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState

#: Tolerance used by the tutorial: 1 mrad for the arm joints, 1 mm for the gripper
DEFAULT_TOLERANCE = 0.001


@dataclass
class PreciseMoveJointsMotion(BaseMotion):
    """Move the named joints to the given positions within ``tolerance``."""

    names: List[str]
    """Names of the joints to move (``Joint1`` ... ``Joint5``, ``Gripper``)."""

    positions: List[float]
    """Target positions, in the order of ``names`` (radians; metres for the gripper)."""

    tolerance: float = DEFAULT_TOLERANCE
    """The motion is finished when every joint error is below this value."""

    def perform(self):
        return

    @property
    def _motion_chart(self):
        connections = [self.world.get_connection_by_name(name) for name in self.names]
        return JointPositionList(
            goal_state=JointState.from_mapping(dict(zip(connections, self.positions))),
            threshold=self.tolerance,
        )
