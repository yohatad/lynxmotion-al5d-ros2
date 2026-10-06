"""Forward a CoraPlex plan to the Gazebo simulator of the course (ROS 2).

A CoraPlex plan runs on a world model. To see the same motion in Gazebo, after every step
the target joint values are sent to ``/lynxmotion_al5d/joints_positions/commands`` (six values:
Joint1..Joint5 in radians, Gripper in metres), and the bridge waits until
``/lynxmotion_al5d/joint_states`` reports that the arm has arrived. This is what the
*process module* of the Lisp tutorial did.
"""

import math
import time

from module11.al5d import joint_positions
from module11.kinematics import ARM_JOINTS

COMMAND_TOPIC = '/lynxmotion_al5d/joints_positions/commands'
STATE_TOPIC = '/lynxmotion_al5d/joint_states'
ORDER = ARM_JOINTS + ('Gripper',)


class SimulatorNotFound(RuntimeError):
    """Raised when nobody listens on the arm command topic."""


def command_vector(positions):
    """Return the six-value command for a ``{joint: value}`` dictionary."""
    return [float(positions[name]) for name in ORDER]


def arrived(command, state, tolerance=0.02):
    """Whether the measured ``state`` (a ``{joint: value}`` dictionary) matches ``command``."""
    return all(name in state and abs(state[name] - value) < tolerance
               for name, value in zip(ORDER, command))


class ArmMirror:
    """A ROS 2 node that sends joint targets to the simulator and waits for the arm to arrive."""

    def __init__(self, node=None, timeout=20.0, tolerance=0.02):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._rclpy = rclpy
        self.node = node or rclpy.create_node('module11_arm_mirror')
        self.timeout = timeout
        self.tolerance = tolerance
        self.state = {}
        self._message_type = Float64MultiArray
        self._publisher = self.node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
        self.node.create_subscription(JointState, STATE_TOPIC, self._on_state, 10)

    def _on_state(self, message):
        self.state.update(dict(zip(message.name, message.position)))

    def wait_for_simulator(self, timeout=30.0):
        """Block until the arm controller listens; raises :class:`SimulatorNotFound`."""
        deadline = time.monotonic() + timeout
        while self._publisher.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                raise SimulatorNotFound(
                    f'nobody listens on {COMMAND_TOPIC}: is the simulator running '
                    '(ros2 launch lynxmotion_al5d_description sim.launch.py)?')
            self._rclpy.spin_once(self.node, timeout_sec=0.1)

    def send(self, positions):
        """Send ``{joint: value}`` to Gazebo and wait until the arm is there.

        :return: True when the arm arrived within the timeout, False otherwise.
        """
        command = command_vector(positions)
        if not all(math.isfinite(value) for value in command):
            raise ValueError(f'not a finite command: {command}')
        message = self._message_type()
        message.data = command
        self._publisher.publish(message)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if arrived(command, self.state, self.tolerance):
                return True
        return False

    def send_world(self, world):
        """Send the current joint values of a CoraPlex world."""
        return self.send(joint_positions(world))

    def close(self):
        self.node.destroy_node()
        if self._owns_rclpy:
            self._rclpy.shutdown()


def run_mirrored(action, context, world, mirror):
    """Run ``action`` on the CoraPlex world one step at a time, sending each result to Gazebo.

    The simulated plan is executed by CoraPlex as a whole, so the steps are run separately to
    be able to show each intermediate pose in the simulator.

    :param action: an action of :mod:`module11.actions` (anything with a ``steps()`` method)
    :return: the number of steps for which the simulator arrived at the target
    """
    from coraplex.execution_environment import simulated_robot
    from coraplex.plans.factories import execute_single

    arrived_count = 0
    for step in action.steps():
        with simulated_robot:
            execute_single(step, context=context).perform()
        if mirror.send_world(world):
            arrived_count += 1
    return arrived_count
