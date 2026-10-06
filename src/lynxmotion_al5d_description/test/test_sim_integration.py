"""End-to-end test of the headless Gazebo simulation: controllers, arm, gripper, camera, bricks."""

import os
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
from lynxmotion_al5d_description.msg import Pose
from lynxmotion_al5d_description.srv import (
    KillBrick, SpawnBrick, TeleportAbsolute, TeleportRelative)
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty

NS = '/lynxmotion_al5d'
HOME = [0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
GRIPPER_OPEN = 0.03175


@pytest.mark.launch_test
def generate_test_description():
    pkg = get_package_share_directory('lynxmotion_al5d_description')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': 'true', 'camera_update_rate': '5.0'}.items())
    # Gazebo discovers peers by partition, not by ROS domain: isolate this run from any other
    # (or leftover) simulation on the same machine.
    partition = SetEnvironmentVariable('GZ_PARTITION', f'rpp_test_{os.getpid()}')
    sliders = Node(package='lynxmotion_al5d_description', executable='slider_command')
    return LaunchDescription([partition, sim, sliders, launch_testing.actions.ReadyToTest()])


class TestSimulation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('al5d_sim_test')
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

        cls.joints = {}
        cls.images = []
        cls.brick_pose = None
        cls.node.create_subscription(JointState, f'{NS}/joint_states', cls._on_joints, 10)
        cls.node.create_subscription(Image, f'{NS}/external_vision/image_raw',
                                     cls.images.append, 10)
        cls.node.create_subscription(Pose, f'{NS}/test_brick/pose', cls._on_brick, 10)
        cls.command = cls.node.create_publisher(
            Float64MultiArray, f'{NS}/joints_positions/commands', 10)
        cls.slider_state = cls.node.create_publisher(JointState, '/arm_sliders/joint_states', 10)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joints(cls, msg):
        cls.joints = dict(zip(msg.name, msg.position))

    @classmethod
    def _on_brick(cls, msg):
        cls.brick_pose = msg

    # --- helpers -----------------------------------------------------------------------
    def wait_for(self, predicate, timeout, message):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return
            time.sleep(0.1)
        self.fail(message)

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout), f'{name} not available')
        future = client.call_async(request)
        self.wait_for(future.done, timeout, f'{name} did not answer')
        return future.result()

    def send_arm(self, values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.command.publish(msg)

    def arm_near(self, values, tol=0.02):
        names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Gripper']
        if not all(n in self.joints for n in names):
            return False
        return all(abs(self.joints[n] - v) < tol for n, v in zip(names, values))

    def move_arm(self, values, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.send_arm(values)  # repeated: the first message may precede discovery
            time.sleep(0.5)
            if self.arm_near(values):
                return
        self.fail(f'arm did not reach {values}; joints are {self.joints}')

    def spawn(self, **kwargs):
        request = SpawnBrick.Request()
        request.color = kwargs.get('color', 'red')
        request.name = kwargs.get('name', '')
        request.pose.position.x = kwargs.get('x', 0.1)
        request.pose.position.y = kwargs.get('y', 0.15)
        request.pose.position.z = kwargs.get('z', 0.1)
        return self.call(SpawnBrick, f'{NS}/spawn_brick', request)

    # --- tests (alphabetical order is the execution order) -------------------------------
    def test_01_controllers_become_active(self):
        client = self.node.create_client(
            ListControllers, f'{NS}/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(180.0), 'controller_manager never appeared')

        def active():
            future = client.call_async(ListControllers.Request())
            deadline = time.time() + 5
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if not future.done():
                return False
            states = {c.name: c.state for c in future.result().controller}
            return (states.get('arm_controller') == 'active' and
                    states.get('joint_state_broadcaster') == 'active')

        self.wait_for(active, 120.0, 'arm_controller / joint_state_broadcaster never active')

    def test_02_initial_pose_is_home_with_open_gripper(self):
        self.wait_for(lambda: len(self.joints) >= 8, 30.0, 'no joint states')
        self.assertTrue(self.arm_near(HOME[:5] + [GRIPPER_OPEN], tol=0.05), self.joints)

    def test_03_arm_follows_commands(self):
        target = [0.5, 1.2, -1.2, 0.3, 0.2, 0.0]
        self.move_arm(target)
        self.move_arm(HOME[:5] + [GRIPPER_OPEN])

    def test_04_fingers_follow_the_gripper(self):
        # Gripper = 0.02 m: right = 0.015875 - 0.5 * 0.02, left = right - 0.0025. Smaller
        # openings (below ~0.016 m) make the two fingers hit each other, so the test avoids them.
        self.move_arm(HOME[:5] + [0.02])
        self.wait_for(lambda: abs(self.joints.get('right_finger_joint', 9) - 0.005875) < 0.001,
                      15.0, f'right finger did not follow the gripper: {self.joints}')
        self.assertAlmostEqual(self.joints['left_finger_joint'], 0.003375, delta=0.001)
        self.move_arm(HOME[:5] + [GRIPPER_OPEN])  # open again
        self.wait_for(lambda: self.joints.get('right_finger_joint', 9) < 0.002, 15.0,
                      f'right finger did not open: {self.joints}')
        self.assertLess(self.joints['left_finger_joint'], 0.002)

    def test_05_malformed_command_is_ignored(self):
        before = dict(self.joints)
        self.send_arm([1.0, 1.0, 1.0])            # wrong size
        self.send_arm([float('nan')] * 6)          # not finite
        time.sleep(2.0)
        for name in ('Joint1', 'Joint2', 'Joint3'):
            self.assertAlmostEqual(self.joints[name], before[name], delta=0.02)

    def test_06_camera_streams_rgb_images(self):
        self.wait_for(lambda: len(self.images) >= 2, 60.0, 'no camera images')
        image = self.images[-1]
        self.assertEqual((image.width, image.height), (640, 480))
        self.assertEqual(image.encoding, 'rgb8')
        self.assertEqual(len(image.data), 640 * 480 * 3)
        self.assertGreater(len(set(image.data[::997])), 1, 'image is a single flat colour')

    def test_07_brick_lifecycle(self):
        response = self.spawn(name='test_brick', color='blue', z=0.1)
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.name, 'test_brick')

        # the pose topic reports the brick, which falls onto the ground plane
        self.wait_for(lambda: self.brick_pose is not None, 20.0, 'no brick pose published')
        self.wait_for(lambda: abs(self.brick_pose.position.z) < 0.01, 20.0,
                      f'brick did not fall to the floor: z={self.brick_pose.position.z}')
        self.assertAlmostEqual(self.brick_pose.position.x, 0.1, delta=0.02)

        # duplicate names are refused
        self.assertFalse(self.spawn(name='test_brick').success)

        # absolute teleport
        request = TeleportAbsolute.Request()
        request.pose.position.x = 0.3
        request.pose.position.y = -0.1
        request.pose.position.z = 0.05
        self.assertTrue(
            self.call(TeleportAbsolute, f'{NS}/test_brick/teleport_absolute', request).success)
        self.wait_for(lambda: abs(self.brick_pose.position.x - 0.3) < 0.02, 20.0,
                      f'teleport not reflected: {self.brick_pose.position}')

        # relative teleport
        request = TeleportRelative.Request()
        request.pose.position.x = 0.1
        self.assertTrue(
            self.call(TeleportRelative, f'{NS}/test_brick/teleport_relative', request).success)
        self.wait_for(lambda: abs(self.brick_pose.position.x - 0.4) < 0.03, 20.0,
                      f'relative teleport not reflected: {self.brick_pose.position}')

        # removal
        request = KillBrick.Request()
        request.name = 'test_brick'
        self.assertTrue(self.call(KillBrick, f'{NS}/kill_brick', request).result)
        request.name = 'test_brick'
        self.assertFalse(self.call(KillBrick, f'{NS}/kill_brick', request).result)

    def test_08_invalid_spawn_requests_are_rejected(self):
        self.assertFalse(self.spawn(color='purple').success)
        self.assertFalse(self.spawn(name='bad name').success)

    def test_09_reset_clears_bricks_and_sends_arm_home(self):
        first = self.spawn(color='green', x=0.15, y=0.1)
        second = self.spawn(color='red', x=0.1, y=0.2)
        self.assertTrue(first.success and second.success)
        self.move_arm([0.4, 1.0, -1.0, 0.2, 0.1, 0.01])

        self.call(Empty, f'{NS}/reset', Empty.Request(), timeout=60.0)
        self.wait_for(lambda: self.arm_near(HOME, tol=0.03), 30.0,
                      f'arm did not return home: {self.joints}')
        # names are free again: the counter restarted
        response = self.spawn(color='green')
        self.assertTrue(response.success)
        self.assertEqual(response.name, 'brick1')

    def test_10_slider_changes_move_the_arm(self):
        names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Gripper']
        baseline = [0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
        target = [0.4, 1.0, -1.0, 0.2, 0.1, 0.02]

        def publish(values):
            message = JointState()
            message.name = names
            message.position = values
            self.slider_state.publish(message)

        # the first message is only a baseline: the arm must not move to it
        for _ in range(10):
            publish(baseline)
            time.sleep(0.1)
        time.sleep(1.0)
        self.assertFalse(self.arm_near(target), 'the arm moved before a slider changed')

        deadline = time.time() + 30
        while time.time() < deadline and not self.arm_near(target):
            publish(target)
            time.sleep(0.2)
        self.assertTrue(self.arm_near(target),
                        f'the arm did not follow the sliders: {self.joints}')


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):

    def test_remove_leftover_gazebo_servers(self):
        # Gazebo needs longer to exit than the launch system waits; do not leave it running.
        subprocess.run(['pkill', '-f', 'gz sim.*al5d.sdf'], check=False)
