"""goToPoseCreate drives a simulated iRobot Create 2 (unicycle model) to the poses in its input."""

# The fake robot publishes nav_msgs/Odometry on `odom` and obeys geometry_msgs/Twist on
# `cmd_vel`, like create_bringup does on the real robot.

import math
import os
import re
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor


class FakeCreate:

    def __init__(self, node, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.w = 0.0
        self.max_v_seen = 0.0
        self.max_w_seen = 0.0
        self.commands = 0
        self.lock = threading.Lock()
        self.last = time.monotonic()
        node.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.pub = node.create_publisher(Odometry, 'odom', 10)
        node.create_timer(0.02, self._step)

    def _on_cmd(self, msg):
        with self.lock:
            self.v, self.w = msg.linear.x, msg.angular.z
            self.max_v_seen = max(self.max_v_seen, abs(self.v))
            self.max_w_seen = max(self.max_w_seen, abs(self.w))
            self.commands += 1

    def _step(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        with self.lock:
            self.theta += self.w * dt
            self.x += self.v * math.cos(self.theta) * dt
            self.y += self.v * math.sin(self.theta) * dt
            theta = self.theta
            x, y = self.x, self.y
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        self.pub.publish(msg)


class TestGoToPoseCreate(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fake_create')
        cls.robot = FakeCreate(cls.node)
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_follows_setpose_goto1_goto2_sequence(self):
        # data/goToPoseCreateInput.txt:
        #   setpose 0 0 0 ; goto1 0 1.2 0 (divide and conquer) ; goto2 0 0 0 (MIMO)
        exe = os.path.join(get_package_prefix('module3'), 'lib', 'module3', 'goToPoseCreate')
        result = subprocess.run([exe], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                timeout=280)
        self.assertEqual(result.returncode, 0, result.stdout[-2000:] + result.stderr[-2000:])
        time.sleep(0.5)

        # The sequence ends back at the origin with zero heading.
        with self.robot.lock:
            x, y, theta = self.robot.x, self.robot.y, self.robot.theta
        wrapped = math.atan2(math.sin(theta), math.cos(theta))
        self.assertLess(math.hypot(x, y), 0.05, f'ended at ({x:.3f}, {y:.3f})')
        self.assertLess(abs(wrapped), 0.15, f'ended with heading {wrapped:.3f}')

        # It visited the intermediate goal: the program printed odometry near (0, 1.2).
        visited = [(float(x), float(y)) for x, y in re.findall(
            r'Odometry: position = \((-?\d+\.\d+), (-?\d+\.\d+)\)', result.stdout)]
        self.assertTrue(visited, 'the program printed no odometry')
        self.assertTrue(any(math.hypot(x, 1.2 - y) < 0.1 for x, y in visited),
                        'never observed the robot close to (0, 1.2)')

    def test_2_commands_respect_velocity_limits(self):
        # parameters.txt: MAX_LINEAR_VELOCITY 0.2, MAX_ANGULAR_VELOCITY 1.0
        self.assertGreater(self.robot.commands, 0, 'test_1 must have run first')
        self.assertLessEqual(self.robot.max_v_seen, 0.2 + 1e-6)
        self.assertLessEqual(self.robot.max_w_seen, 1.0 + 1e-6)
