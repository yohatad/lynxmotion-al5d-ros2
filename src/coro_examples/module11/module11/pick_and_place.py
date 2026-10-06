"""Pick a brick and place it: ``python -m module11.pick_and_place``.

Runs :class:`~module11.actions.PickAndPlaceAction` on the AL5D in a CoraPlex world. With
``--simulator`` the same steps are also sent to the Gazebo simulator (start it first with
``ros2 launch lynxmotion_al5d_description sim.launch.py``).
"""

import argparse


def build_parser():
    """Return the command-line parser."""
    parser = argparse.ArgumentParser(description='Pick and place a brick with CoraPlex.')
    parser.add_argument('--source', type=float, nargs=3, default=(-60.0, 175.0, 0.0),
                        metavar=('X', 'Y', 'Z'),
                        help='brick position in mm (default: %(default)s)')
    parser.add_argument('--destination', type=float, nargs=3, default=(60.0, 175.0, 0.0),
                        metavar=('X', 'Y', 'Z'),
                        help='place position in mm (default: %(default)s)')
    parser.add_argument('--source-phi', type=float, default=90.0,
                        help='brick orientation about the vertical axis in degrees')
    parser.add_argument('--destination-phi', type=float, default=0.0,
                        help='brick orientation at the destination in degrees')
    parser.add_argument('--simulator', action='store_true',
                        help='also move the arm in the Gazebo simulator')
    return parser


def main(argv=None):
    """Run the pick and place; the joint values at the end are printed."""
    args = build_parser().parse_args(argv)

    from coraplex.execution_environment import simulated_robot
    from coraplex.plans.factories import execute_single

    from module11.actions import PickAndPlaceAction
    from module11.al5d import joint_positions, make_context

    context, world = make_context()
    action = PickAndPlaceAction(*args.source, *args.destination,
                                args.source_phi, args.destination_phi)
    print(f'{len(action.steps())} steps')

    if args.simulator:
        from module11.ros2_bridge import ArmMirror, run_mirrored
        mirror = ArmMirror()
        try:
            mirror.wait_for_simulator()
            arrived = run_mirrored(action, context, world, mirror)
            print(f'the simulator reached {arrived} of {len(action.steps())} steps')
        finally:
            mirror.close()
    else:
        with simulated_robot:
            execute_single(action, context=context).perform()

    for name, value in joint_positions(world).items():
        print(f'{name:8s} {value:8.4f}')


if __name__ == '__main__':
    main()
