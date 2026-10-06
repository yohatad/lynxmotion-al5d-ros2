"""The AL5D in a CoraPlex world.

CoraPlex plans run on a *world*: a model of the robot and its surroundings. These helpers
load the AL5D model of the ``lynxmotion_al5d_description`` package into such a world and wrap
it in the objects a plan needs.
"""

from module11.kinematics import ARM_JOINTS

AL5D_URDF = 'package://lynxmotion_al5d_description/urdf/lynxmotion_al5d.xacro'
GRIPPER_JOINT = 'Gripper'
ALL_JOINTS = ARM_JOINTS + (GRIPPER_JOINT,)


def load_world():
    """Load the AL5D model into a new CoraPlex world.

    The URDF is expanded with xacro and parsed. ``package://`` paths are resolved through the
    ROS 2 environment, so the workspace of the course must be sourced.
    """
    from semantic_digital_twin.adapters.urdf import URDFParser
    return URDFParser.from_file(AL5D_URDF).parse()


def make_robot(world):
    """Annotate the arm in ``world`` as a robot (the simplest annotation CoraPlex offers)."""
    from semantic_digital_twin.robots.minimal_robot import MinimalRobot
    return MinimalRobot.from_branch_in_world(world.get_body_by_name('robot_support'))


def make_context(world=None):
    """World, robot and plan context in one call.

    :return: ``(context, world)``; pass ``context`` to the plan factories.
    """
    from coraplex.datastructures.dataclasses import Context
    world = world if world is not None else load_world()
    return Context(world, make_robot(world)), world


def joint_positions(world):
    """Return the joint positions of the AL5D as ``{name: value}`` (rad; m for the Gripper)."""
    positions = {}
    for connection in world.connections:
        name = connection.name.name
        if name in ALL_JOINTS:
            positions[name] = float(connection.position)
    return positions
