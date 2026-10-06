"""Kinematics of the Lynxmotion AL5D arm (pure Python, no ROS and no CoraPlex needed).

This is the Python version of ``computeJointAngles`` in module 4 (``lynxmotionUtilities.cpp``),
so that the CoraPlex tutorial and the C++ examples agree. Lengths are in millimetres and
angles in degrees at the interface (as in module 4) and in radians in the joint vectors.
"""

from dataclasses import dataclass
import math

#: Height of the shoulder axis above the base plane (mm)
D1 = 70.0
#: Shoulder-to-elbow link (mm)
A3 = 146.0
#: Elbow-to-wrist link (mm)
A4 = 187.0
#: Length of the gripper from the wrist to the middle of the finger tips (mm)
GRIPPER_LENGTH = 105.0

#: Gripper opening for an open and for a closed gripper (mm); a Lego brick is 15.8 mm wide
GRIPPER_OPEN = 30.0
GRIPPER_CLOSED = 15.0

#: Working envelope of the arm (mm), as in module 4
MIN_X, MAX_X = -130.0, 130.0
MIN_Y, MAX_Y = 80.0, 330.0
MIN_Z, MAX_Z = 0.0, 380.0

#: Names of the five arm joints, in the order of the joint vector returned by the IK
ARM_JOINTS = ('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5')

#: Joint limits (rad) of the AL5D model, as in urdf/lynxmotion_al5d.xacro
JOINT_LIMITS = (
    (-math.pi, math.pi),        # Joint1: base rotation
    (0.0, math.pi),             # Joint2: shoulder
    (-math.pi, 0.0),            # Joint3: elbow
    (-math.pi / 2, math.pi / 2),  # Joint4: wrist pitch
    (-math.pi, math.pi),        # Joint5: wrist roll
)


class UnreachablePose(ValueError):
    """Raised when no joint configuration of the arm places the wrist at the pose."""


@dataclass(frozen=True)
class WristPose:
    """Position (mm) of the wrist and the orientation of the gripper (degrees).

    ``pitch`` is 0 for a gripper pointing straight up and -180 (or +180) for straight down;
    ``roll`` rotates the gripper about its own axis.
    """

    x: float
    y: float
    z: float
    pitch: float = -180.0
    roll: float = 0.0


def within_working_envelope(x, y, z):
    """Whether the point lies in the working envelope of the arm."""
    return (MIN_X < int(x) <= MAX_X and MIN_Y < int(y) <= MAX_Y
            and MIN_Z < int(z) <= MAX_Z)


def inverse_kinematics(pose):
    """Joint angles (rad) of Joint1 to Joint5 that put the wrist at ``pose``.

    :raises UnreachablePose: if the wrist cannot reach the position.
    """
    x, y, z = pose.x, pose.y, pose.z
    base = math.atan2(x, y)
    radial = math.hypot(x, y)
    wrist_z = z - D1
    reach_squared = wrist_z * wrist_z + radial * radial
    reach = math.sqrt(reach_squared)
    if reach == 0.0:
        raise UnreachablePose('the wrist cannot be on the shoulder axis')

    alpha = math.atan2(wrist_z, radial)
    cos_beta = ((A3 * A3 - A4 * A4) + reach_squared) / (2.0 * A3 * reach)
    cos_elbow = (reach_squared - A3 * A3 - A4 * A4) / (2.0 * A3 * A4)
    if not (-1.0 <= cos_beta <= 1.0 and -1.0 <= cos_elbow <= 1.0):
        raise UnreachablePose(f'({x:.0f}, {y:.0f}, {z:.0f}) mm is outside the reach of the arm')

    shoulder = alpha + math.acos(cos_beta)
    elbow = -math.acos(cos_elbow)           # elbow-up solution, always negative

    pitch = pose.pitch
    wrist_pitch = (pitch - math.degrees(elbow)) - math.degrees(shoulder) + 90.0
    if int(pitch) == 0:                     # pointing up: roll compensates the base rotation
        wrist_roll = pose.roll + math.degrees(base) + 90.0
    elif int(pitch) in (-180, 180):         # pointing down
        wrist_roll = pose.roll - math.degrees(base) + 90.0
    else:
        wrist_roll = pose.roll + 90.0
    joints = (base, shoulder, elbow, _wrap(math.radians(wrist_pitch)),
              _wrap(math.radians(wrist_roll)))
    for name, value, (low, high) in zip(ARM_JOINTS, joints, JOINT_LIMITS):
        if not low - 1e-6 <= value <= high + 1e-6:
            raise UnreachablePose(
                f'({x:.0f}, {y:.0f}, {z:.0f}) mm with pitch {pitch:.0f} needs '
                f'{name} = {math.degrees(value):.0f} deg, outside its limits')
    return joints


def _wrap(angle):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def forward_kinematics(joints):
    """Wrist position (x, y, z) in mm for the first three joint angles (rad)."""
    base, shoulder, elbow = joints[0], joints[1], joints[2]
    reach = A3 * math.cos(shoulder) + A4 * math.cos(shoulder + elbow)
    height = D1 + A3 * math.sin(shoulder) + A4 * math.sin(shoulder + elbow)
    return (reach * math.sin(base), reach * math.cos(base), height)


def gripper_joint_value(distance_mm):
    """Value of the ``Gripper`` joint (metres) for a finger distance in millimetres."""
    return min(max(distance_mm, 0.0), 31.75) / 1000.0
