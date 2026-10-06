"""Tests of the ROS 2 bridge. The pure helpers need nothing; the node tests need ``rclpy``."""

import importlib.util
import threading
import time

from module11.ros2_bridge import (
    arrived, COMMAND_TOPIC, command_vector, ORDER, SimulatorNotFound, STATE_TOPIC)
import pytest

STATE = dict(zip(ORDER, [0.1, 1.2, -1.2, -0.5, 0.3, 0.02]))


def test_command_vector_follows_the_joint_order():
    assert command_vector(STATE) == [0.1, 1.2, -1.2, -0.5, 0.3, 0.02]


def test_arrived_compares_every_joint():
    assert arrived(command_vector(STATE), STATE)
    off = dict(STATE, Joint3=-1.0)
    assert not arrived(command_vector(STATE), off)
    assert not arrived(command_vector(STATE), {})


def test_topics_are_those_of_the_simulator():
    assert COMMAND_TOPIC == '/lynxmotion_al5d/joints_positions/commands'
    assert STATE_TOPIC == '/lynxmotion_al5d/joint_states'


needs_ros = pytest.mark.skipif(
    importlib.util.find_spec('rclpy') is None,
    reason='rclpy (ROS 2) is not available')


@pytest.fixture()
def fake_controller():
    """Provide a stand-in for the simulator: follows every command and publishes states."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('fake_al5d_controller')
    received = []
    state = JointState()
    state.name = list(ORDER)
    state.position = [0.0] * 6
    publisher = node.create_publisher(JointState, STATE_TOPIC, 10)

    def on_command(message):
        received.append(list(message.data))
        state.position = list(message.data)

    node.create_subscription(Float64MultiArray, COMMAND_TOPIC, on_command, 10)
    node.create_timer(0.05, lambda: publisher.publish(state))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield received
    executor.shutdown()
    node.destroy_node()


@needs_ros
def test_send_publishes_the_command_and_waits_for_the_arm(fake_controller):
    from module11.ros2_bridge import ArmMirror
    mirror = ArmMirror(timeout=5.0)
    try:
        mirror.wait_for_simulator(timeout=10.0)
        assert mirror.send(STATE) is True
        assert fake_controller[-1] == pytest.approx(command_vector(STATE))
    finally:
        mirror.close()


@needs_ros
def test_send_reports_failure_when_the_arm_does_not_arrive():
    from module11.ros2_bridge import ArmMirror
    mirror = ArmMirror(timeout=0.5)  # nobody answers on the state topic
    try:
        start = time.monotonic()
        assert mirror.send(STATE) is False
        assert time.monotonic() - start < 3.0
    finally:
        mirror.close()


@needs_ros
def test_send_rejects_non_finite_commands():
    from module11.ros2_bridge import ArmMirror
    mirror = ArmMirror(timeout=0.5)
    try:
        with pytest.raises(ValueError):
            mirror.send(dict(STATE, Joint1=float('nan')))
    finally:
        mirror.close()


@needs_ros
def test_wait_for_simulator_times_out_with_a_clear_error():
    from module11.ros2_bridge import ArmMirror
    mirror = ArmMirror()
    try:
        with pytest.raises(SimulatorNotFound, match='simulator'):
            mirror.wait_for_simulator(timeout=0.5)
    finally:
        mirror.close()
