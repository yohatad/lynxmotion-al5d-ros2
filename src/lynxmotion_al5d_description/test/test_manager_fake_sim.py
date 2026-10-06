"""manager_node + GzBackend + CLI tools against fake Gazebo bridge services (no Gazebo)."""

import subprocess
import threading
import time
import unittest

from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lynxmotion_al5d_description.msg import Pose
from lynxmotion_al5d_description.srv import KillBrick, SpawnBrick, TeleportAbsolute
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from tf2_msgs.msg import TFMessage

WORLD = '/world/al5d'
PKG = 'lynxmotion_al5d_description'


@pytest.mark.launch_test
def generate_test_description():
    manager = Node(package=PKG, executable='manager_node', name='sim_manager', output='screen',
                   parameters=[{'service_timeout': 2.0}])
    return LaunchDescription([manager, launch_testing.actions.ReadyToTest()]), {'manager': manager}


class FakeGazebo:
    """Stands in for the ros_gz_bridge service bridges."""

    def __init__(self, node):
        self.spawned = []
        self.removed = []
        self.poses = []
        self.accept = True
        node.create_service(SpawnEntity, f'{WORLD}/create', self._spawn)
        node.create_service(DeleteEntity, f'{WORLD}/remove', self._remove)
        node.create_service(SetEntityPose, f'{WORLD}/set_pose', self._set_pose)

    def _spawn(self, request, response):
        self.spawned.append(request.entity_factory)
        response.success = self.accept
        return response

    def _remove(self, request, response):
        self.removed.append(request.entity)
        response.success = self.accept
        return response

    def _set_pose(self, request, response):
        self.poses.append((request.entity, request.pose))
        response.success = self.accept
        return response


class TestManagerWithFakeGazebo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fake_gz_and_test_client')
        cls.gz = FakeGazebo(cls.node)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.spawn = cls.node.create_client(SpawnBrick, '/lynxmotion_al5d/spawn_brick')
        cls.kill = cls.node.create_client(KillBrick, '/lynxmotion_al5d/kill_brick')
        assert cls.spawn.wait_for_service(30.0), 'manager did not come up'
        assert cls.kill.wait_for_service(30.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.gz.accept = True

    def _call(self, client, request, timeout=15.0):
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done(), 'service call timed out')
        return future.result()

    def _spawn(self, color, name='', x=0.0):
        request = SpawnBrick.Request()
        request.color = color
        request.name = name
        request.pose.position.x = x
        return self._call(self.spawn, request)

    def test_1_spawn_goes_through_gz_service_with_sdf_and_pose(self):
        response = self._spawn('red', 'fake_red', 0.25)
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.name, 'fake_red')
        factory = self.gz.spawned[-1]
        self.assertEqual(factory.name, 'fake_red')
        self.assertFalse(factory.allow_renaming)
        self.assertIn('<model', factory.sdf)
        self.assertAlmostEqual(factory.pose.position.x, 0.25)

    def test_2_pose_topic_follows_gazebo_pose_stream(self):
        received = []
        sub = self.node.create_subscription(
            Pose, '/lynxmotion_al5d/fake_red/pose', received.append, 10)
        pub = self.node.create_publisher(TFMessage, f'{WORLD}/dynamic_pose/info', 10)
        transform = TransformStamped()
        transform.child_frame_id = 'fake_red'
        transform.transform.translation.x = 0.7
        transform.transform.rotation.w = 1.0
        deadline = time.time() + 10
        while time.time() < deadline and not any(abs(p.position.x - 0.7) < 1e-6
                                                 for p in received):
            pub.publish(TFMessage(transforms=[transform]))
            time.sleep(0.1)
        self.node.destroy_subscription(sub)
        self.assertTrue(any(abs(p.position.x - 0.7) < 1e-6 for p in received))

    def test_3_teleport_goes_through_set_pose(self):
        client = self.node.create_client(TeleportAbsolute,
                                         '/lynxmotion_al5d/fake_red/teleport_absolute')
        self.assertTrue(client.wait_for_service(10.0))
        request = TeleportAbsolute.Request()
        request.pose.position.y = -0.3
        response = self._call(client, request)
        self.assertTrue(response.success, response.message)
        entity, pose = self.gz.poses[-1]
        self.assertEqual(entity.name, 'fake_red')
        self.assertAlmostEqual(pose.position.y, -0.3)

    def test_4_simulator_rejection_is_reported(self):
        self.gz.accept = False
        response = self._spawn('blue', 'rejected')
        self.assertFalse(response.success)
        self.assertIn('rejected', response.message)
        self.gz.accept = True
        self.assertTrue(self._spawn('blue', 'rejected').success)

    def test_5_kill_goes_through_remove_service(self):
        request = KillBrick.Request()
        request.name = 'rejected'
        response = self._call(self.kill, request)
        self.assertTrue(response.result, response.message)
        self.assertEqual(self.gz.removed[-1].name, 'rejected')

    def test_6_cli_spawn_and_kill(self):
        base = ['ros2', 'run', PKG]
        result = subprocess.run(
            base + ['spawn_brick', '-c', 'green', '-n', 'cli_brick', '-x', '0.1'],
            capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.gz.spawned[-1].name, 'cli_brick')
        result = subprocess.run(base + ['kill_brick', 'cli_brick'],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        result = subprocess.run(base + ['kill_brick', 'cli_brick'],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 1)  # already gone

    def test_7_cli_usage_errors(self):
        base = ['ros2', 'run', PKG]
        result = subprocess.run(base + ['spawn_brick', '-x', '1'], capture_output=True, text=True,
                                timeout=60)
        self.assertEqual(result.returncode, 2)
        self.assertIn('Usage', result.stderr)
        result = subprocess.run(base + ['spawn_brick', '--help'], capture_output=True, text=True,
                                timeout=60)
        self.assertEqual(result.returncode, 0)
        result = subprocess.run(base + ['spawn_brick', '-c', 'purple'], capture_output=True,
                                text=True, timeout=60)
        self.assertEqual(result.returncode, 1)  # manager rejects the colour
        result = subprocess.run(base + ['kill_brick'], capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 2)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_manager_exit_code(self, proc_info, manager):
        launch_testing.asserts.assertExitCodes(proc_info, process=manager)
