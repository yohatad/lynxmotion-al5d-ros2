"""Static checks of the robot description, controller config, models, world and launch files."""

import importlib.util
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / 'urdf' / 'lynxmotion_al5d.xacro'

ARM_JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Gripper']
ACTUATED_JOINTS = ARM_JOINTS + ['right_finger_joint', 'left_finger_joint']


@pytest.fixture(scope='module')
def robot():
    return ET.fromstring(xacro.process_file(str(XACRO)).toxml())


def test_xacro_expands_to_robot(robot):
    assert robot.tag == 'robot'
    assert robot.get('name') == 'lynxmotion_al5d'


def test_arm_joints_present_and_actuated(robot):
    joints = {j.get('name'): j for j in robot.findall('joint')}
    for name in ARM_JOINTS:
        assert name in joints, f'missing joint {name}'
    for name in ARM_JOINTS[:5]:
        assert joints[name].get('type') == 'revolute'
    assert joints['Gripper'].get('type') == 'prismatic'


def test_joint_limits_are_sane(robot):
    for joint in robot.findall('joint'):
        if joint.get('type') not in ('revolute', 'prismatic'):
            continue
        limit = joint.find('limit')
        assert limit is not None, joint.get('name')
        assert float(limit.get('lower')) < float(limit.get('upper')), joint.get('name')
        assert float(limit.get('effort')) > 0, joint.get('name')
        assert float(limit.get('velocity')) > 0, joint.get('name')


def test_kinematic_tree_is_connected_with_single_root(robot):
    links = {link.get('name') for link in robot.findall('link')}
    children = set()
    for joint in robot.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        assert parent in links and child in links, joint.get('name')
        assert child not in children, f'{child} has two parents'
        children.add(child)
    assert links - children == {'world'}


def test_inertials_are_physical(robot):
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        assert mass > 0, link.get('name')
        i = inertial.find('inertia')
        ixx, iyy, izz, ixy, ixz, iyz = (
            float(i.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))
        tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        a, b, c = np.linalg.eigvalsh(tensor)  # principal moments, ascending
        assert a > 0, f'{link.get("name")}: inertia tensor is not positive definite'
        # principal moments of a physical body satisfy the triangle inequality
        assert a + b >= c * (1 - 1e-9), f'{link.get("name")}: principal moments violate it'


def test_meshes_exist(robot):
    prefix = 'package://lynxmotion_al5d_description/'
    meshes = [m.get('filename') for m in robot.iter('mesh')]
    assert meshes
    for filename in meshes:
        assert filename.startswith(prefix), filename
        assert (PKG / filename[len(prefix):]).is_file(), f'missing mesh {filename}'


def test_no_ros1_leftovers(robot):
    text = ET.tostring(robot, encoding='unicode')
    for stale in ('gazebo_ros_control', 'libgazebo_ros', 'roboticsgroup', 'Gazebo/',
                  'hardware_interface/PositionJointInterface', '<transmission'):
        assert stale not in text, f'ROS 1 leftover: {stale}'


def test_ros2_control_interface(robot):
    control = robot.find('ros2_control')
    assert control is not None
    assert control.find('hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
    urdf_joints = {j.get('name'): j for j in robot.findall('joint')}
    names = []
    for joint in control.findall('joint'):
        name = joint.get('name')
        names.append(name)
        assert urdf_joints[name].get('type') != 'fixed'
        assert [c.get('name') for c in joint.findall('command_interface')] == ['position']
        states = {s.get('name') for s in joint.findall('state_interface')}
        assert {'position', 'velocity'} <= states
    assert names == ACTUATED_JOINTS


def test_gz_plugins_and_camera(robot):
    plugins = {p.get('filename') for p in robot.iter('plugin') if p.get('filename')}
    assert 'gz_ros2_control-system' in plugins
    sensor = next(robot.iter('sensor'))
    assert sensor.get('type') == 'camera'
    assert sensor.find('topic').text == '/lynxmotion_al5d/external_vision/image_raw'


def test_camera_rate_is_configurable():
    text = xacro.process_file(str(XACRO), mappings={'camera_update_rate': '7.5'}).toxml()
    assert '<update_rate>7.5</update_rate>' in text


def test_controller_config_matches_description(robot):
    config = yaml.safe_load((PKG / 'config' / 'controllers.yaml').read_text())
    ns = config['/lynxmotion_al5d']
    manager = ns['controller_manager']['ros__parameters']
    assert manager['arm_controller']['type'] == (
        'forward_command_controller/ForwardCommandController')
    assert manager['joint_state_broadcaster']['type'] == (
        'joint_state_broadcaster/JointStateBroadcaster')
    assert ns['arm_controller']['ros__parameters']['joints'] == ACTUATED_JOINTS
    assert ns['arm_controller']['ros__parameters']['interface_name'] == 'position'


def test_home_position_inside_limits(robot):
    limits = {j.get('name'): j.find('limit') for j in robot.findall('joint') if j.find('limit')
              is not None}
    home = [0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
    for name, value in zip(ARM_JOINTS, home):
        assert float(limits[name].get('lower')) <= value <= float(limits[name].get('upper')), name


@pytest.mark.parametrize('color,rgb', [('Red', '0.9'), ('Green', '0.8'), ('Blue', '1.0')])
def test_brick_models(color, rgb):
    directory = PKG / 'models' / f'{color}_Lego_Brick'
    sdf = ET.parse(directory / 'model.sdf').getroot()
    assert sdf.tag == 'sdf'
    model = sdf.find('model')
    assert model.find('link/collision') is not None
    text = (directory / 'model.sdf').read_text()
    assert 'Gazebo/' not in text, 'Gazebo Classic material scripts are not supported by gz sim'
    assert rgb in text
    assert (directory / 'model.config').is_file()


def test_world_is_wellformed():
    world = ET.parse(PKG / 'worlds' / 'al5d.sdf').getroot().find('world')
    assert world.get('name') == 'al5d'
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert {'gz-sim-physics-system', 'gz-sim-scene-broadcaster-system',
            'gz-sim-user-commands-system', 'gz-sim-sensors-system'} <= plugins


@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf not installed')
def test_check_urdf_accepts_description(tmp_path):
    urdf = tmp_path / 'al5d.urdf'
    urdf.write_text(xacro.process_file(str(XACRO)).toxml())
    result = subprocess.run(['check_urdf', str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz not installed')
def test_gz_accepts_world_and_converted_model(tmp_path):
    for path in (PKG / 'worlds' / 'al5d.sdf',):
        result = subprocess.run(['gz', 'sdf', '-k', str(path)], capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
    urdf = tmp_path / 'al5d.urdf'
    urdf.write_text(xacro.process_file(str(XACRO)).toxml())
    result = subprocess.run(['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert re.search(r"<joint name=['\"]Gripper['\"]", result.stdout)


@pytest.mark.parametrize(
    'launch_file', ['sim.launch.py', 'display.launch.py', 'sliders.launch.py'])
def test_launch_files_build(launch_file):
    path = PKG / 'launch' / launch_file
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) >= 2


def test_package_manifest_is_consistent():
    root = ET.parse(PKG / 'package.xml').getroot()
    assert root.get('format') == '3'
    assert root.find('name').text == os.path.basename(PKG)
