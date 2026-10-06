"""Run the four module2 examples against a real (offscreen) turtlesim."""

import math
import os
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from turtlesim.msg import Pose


def executable(name):
    return os.path.join(get_package_prefix('module2'), 'lib', 'module2', name)


def start(name):
    # own process group so that SIGINT reaches the program itself, like Ctrl-C in a terminal
    return subprocess.Popen([executable(name)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)


def stop(process):
    os.killpg(process.pid, signal.SIGINT)
    output, _ = process.communicate(timeout=30)
    return output


@pytest.mark.launch_test
def generate_test_description():
    turtlesim = Node(package='turtlesim', executable='turtlesim_node', output='screen',
                     additional_env={'QT_QPA_PLATFORM': 'offscreen'})
    return LaunchDescription([turtlesim, launch_testing.actions.ReadyToTest()])


class TestExamples(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('module2_test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.poses = []
        cls.cmds = []
        cls.node.create_subscription(Pose, 'turtle1/pose', cls.poses.append, 10)
        cls.node.create_subscription(Twist, 'turtle1/cmd_vel', cls.cmds.append, 10)
        # wait for turtlesim to start publishing
        cls._spin_until(lambda: len(cls.poses) > 0, 60.0, 'turtlesim never published a pose')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin_until(cls, predicate, timeout, message='timed out'):
        deadline = time.time() + timeout
        while time.time() < deadline:
            cls.executor.spin_once(timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError(message)

    def test_1_hello_logs_and_exits(self):
        result = subprocess.run([executable('hello')], capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Hello World!', result.stdout + result.stderr)

    def test_2_useservices_teleports_turtle(self):
        result = subprocess.run([executable('useservices')], capture_output=True, text=True,
                                timeout=90)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.__class__.poses.clear()
        self._spin_until(lambda: len(self.poses) > 2, 10.0, 'no pose after teleport')
        pose = self.poses[-1]
        self.assertAlmostEqual(pose.x, 2.5, places=2)
        self.assertAlmostEqual(pose.y, 3.5, places=2)
        self.assertAlmostEqual(pose.theta, math.pi / 2, places=2)

    def test_3_pubvel_publishes_bounded_random_velocities_and_turtle_moves(self):
        self.__class__.cmds.clear()
        before = self.poses[-1]
        process = start('pubvel')
        try:
            self._spin_until(lambda: len(self.cmds) >= 4, 30.0, 'pubvel published too little')
            time.sleep(1.0)
            self.executor.spin_once(timeout_sec=0.1)
        finally:
            output = stop(process)
        self.assertEqual(process.returncode, 0, 'pubvel must exit cleanly on SIGINT')
        for cmd in self.cmds:
            self.assertGreaterEqual(cmd.linear.x, 0.0)
            self.assertLessEqual(cmd.linear.x, 1.0)
            self.assertGreaterEqual(cmd.angular.z, -1.0)
            self.assertLessEqual(cmd.angular.z, 1.0)
            self.assertEqual(cmd.linear.y, 0.0)
        self.assertIn('Sending random velocity command', output)
        after = self.poses[-1]
        self.assertGreater(math.hypot(after.x - before.x, after.y - before.y)
                           + abs(after.theta - before.theta), 0.0, 'turtle did not move')

    def test_4_subpose_prints_pose(self):
        process = start('subpose')
        try:
            time.sleep(4.0)
        finally:
            output = stop(process)
        self.assertEqual(process.returncode, 0, 'subpose must exit cleanly on SIGINT')
        self.assertRegex(output, r'position=\(\d+\.\d\d,\d+\.\d\d\) direction=-?\d+\.\d\d')
