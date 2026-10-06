"""goToPosition drives the real (offscreen) turtlesim to the goals in its input file."""

import math
import os
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from turtlesim.msg import Pose


def executable():
    return os.path.join(get_package_prefix('module3'), 'lib', 'module3', 'goToPosition')


@pytest.mark.launch_test
def generate_test_description():
    turtlesim = Node(package='turtlesim', executable='turtlesim_node', output='screen',
                     additional_env={'QT_QPA_PLATFORM': 'offscreen'})
    return LaunchDescription([turtlesim, launch_testing.actions.ReadyToTest()])


class TestGoToPosition(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('module3_test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.pose = None

        def on_pose(msg):
            cls.pose = msg
        cls.node.create_subscription(Pose, 'turtle1/pose', on_pose, 10)
        deadline = time.time() + 60
        while cls.pose is None and time.time() < deadline:
            cls.executor.spin_once(timeout_sec=0.1)
        assert cls.pose is not None, 'turtlesim never published a pose'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_drives_turtle_to_the_last_goal(self):
        # Input file: two goto1 commands; the last one is (1, 2, 1.57) -> (9, 8).
        result = subprocess.run([executable()], stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, timeout=180)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0.1)
        # delta_pos in the program is 0.5 m; the turtle keeps coasting slightly, so allow 1 m
        distance = math.hypot(self.pose.x - 9.0, self.pose.y - 8.0)
        self.assertLess(distance, 1.0, f'turtle ended at ({self.pose.x:.2f}, {self.pose.y:.2f})')

    def test_2_sigint_terminates_cleanly(self):
        # Interrupting the program while it is running must exit cleanly (no hang, no abort).
        process = subprocess.Popen([executable()], stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
        time.sleep(2.0)
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            self.fail('goToPosition did not terminate on SIGINT')
        self.assertEqual(process.returncode, 0)
