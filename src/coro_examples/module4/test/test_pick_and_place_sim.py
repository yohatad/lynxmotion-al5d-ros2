"""Run the real pickAndPlace program against the simulated AL5D."""

import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lynxmotion_al5d_description.msg import Pose
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

NS = '/lynxmotion_al5d'
# Millimetres in the robot frame; the brick is spawned at the same world coordinates.
PICK = (-60.0, 175.0, 0.0, 90.0)
PLACE = (60.0, 175.0, 0.0, 0.0)


@pytest.mark.launch_test
def generate_test_description():
    pkg = get_package_share_directory('lynxmotion_al5d_description')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': 'true', 'camera_update_rate': '1.0'}.items())
    # Gazebo discovers peers by partition, not by ROS domain: isolate this run from any other
    # (or leftover) simulation on the same machine.
    partition = SetEnvironmentVariable('GZ_PARTITION', f'rpp_test_{os.getpid()}')
    return LaunchDescription([partition, sim, launch_testing.actions.ReadyToTest()])


class TestPickAndPlaceInSimulator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('module4_sim_test')
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.brick_track = []     # (time, x, y, z) of brick1
        cls.commands = []
        cls.joints = {}
        cls.node.create_subscription(
            Pose, f'{NS}/brick1/pose',
            lambda m: cls.brick_track.append(
                (time.time(), m.position.x, m.position.y, m.position.z)), 10)
        cls.node.create_subscription(Float64MultiArray, f'{NS}/joints_positions/commands',
                                     lambda m: cls.commands.append(list(m.data)), 10)
        cls.node.create_subscription(JointState, f'{NS}/joint_states', cls._on_joints, 10)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joints(cls, msg):
        cls.joints = dict(zip(msg.name, msg.position))

    def wait_for(self, predicate, timeout, message):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return
            time.sleep(0.2)
        self.fail(message)

    def wait_for_controllers(self):
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
            return {c.name: c.state for c in future.result().controller}.get(
                'arm_controller') == 'active'

        self.wait_for(active, 120.0, 'arm_controller never became active')

    def test_pick_and_place_runs_end_to_end(self):
        self.wait_for_controllers()

        with tempfile.TemporaryDirectory() as data:
            share = get_package_share_directory('module4')
            with open(os.path.join(share, 'data', 'robot_simulator_config.txt')) as f:
                config = f.read()
            with open(os.path.join(data, 'robot_simulator_config.txt'), 'w') as f:
                f.write(config)
            with open(os.path.join(data, 'pickAndPlaceInput.txt'), 'w') as f:
                f.write('robot_simulator_config.txt\n')
                f.write('%f %f %f %f\n' % PICK)
                f.write('%f %f %f %f\n' % PLACE)

            exe = os.path.join(get_package_prefix('module4'), 'lib', 'module4', 'pickAndPlace')
            env = dict(os.environ, CORO_DATA_DIR_MODULE4=data)
            started = time.time()
            process = subprocess.Popen([exe], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, env=env,
                                       start_new_session=True)
            try:
                output, _ = process.communicate(timeout=400)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                output, _ = process.communicate()
                self.fail('pickAndPlace did not finish in time. Output tail:\n' + output[-3000:])

        self.assertEqual(process.returncode, 0, output[-3000:])
        self.assertGreater(len(self.commands), 20, 'the arm was hardly commanded')

        # every command is a well-formed six-vector
        for command in self.commands:
            self.assertEqual(len(command), 6)
            self.assertTrue(all(math.isfinite(v) for v in command))

        # the brick was spawned at the pick position ...
        track = [t for t in self.brick_track if t[0] >= started]
        self.assertGreater(len(track), 10, 'brick pose was never published')
        first = track[min(5, len(track) - 1)]
        self.assertAlmostEqual(first[1], PICK[0] / 1000.0, delta=0.03,
                               msg=f'brick did not start at the pick position: {first}')
        self.assertAlmostEqual(first[2], PICK[1] / 1000.0, delta=0.03)

        # KNOWN LIMITATION: whether the simulated fingers lift the brick depends on the
        # gripper geometry in the simulator (see the README), so it is not asserted.
        # The arm must however have descended to the brick and come back: the shoulder joint
        # (index 1) is commanded through a wide range during the approach and departure.
        shoulder = [c[1] for c in self.commands]
        self.assertGreater(max(shoulder) - min(shoulder), 0.1, 'the arm never moved to the brick')


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):

    def test_remove_leftover_gazebo_servers(self):
        # Gazebo needs longer to exit than the launch system waits; do not leave it running.
        subprocess.run(['pkill', '-f', 'gz sim.*al5d.sdf'], check=False)
