# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Static acceptance tests for optional challenge one: custom RGB-D camera."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]
XACRO_PATH = (
    PACKAGE_ROOT / 'urdf' / 'turtlebot3_burger_rgbd.urdf.xacro'
)


def _xacro_root():
    return ET.parse(XACRO_PATH).getroot()


def test_rgbd_xacro_extends_stock_burger_model():
    root = _xacro_root()
    xacro_namespace = 'http://www.ros.org/wiki/xacro'
    include = root.find(f'{{{xacro_namespace}}}include')
    assert root.attrib['name'] == 'turtlebot3_burger_rgbd'
    assert include is not None
    assert include.attrib['filename'].endswith(
        '/urdf/turtlebot3_burger.urdf'
    )


def test_camera_has_visual_collision_mass_and_fixed_tf_chain():
    root = _xacro_root()
    camera_link = root.find("./link[@name='camera_link']")
    assert camera_link is not None
    assert camera_link.find('./visual/geometry/box') is not None
    assert camera_link.find('./collision/geometry/box') is not None
    assert float(camera_link.find('./inertial/mass').attrib['value']) > 0.0

    joints = {
        joint.attrib['name']: joint
        for joint in root.findall('./joint')
    }
    assert {
        'camera_mount_joint',
        'camera_rgb_joint',
        'camera_rgb_optical_joint',
        'camera_depth_joint',
        'camera_depth_optical_joint',
    } <= joints.keys()
    assert all(joint.attrib['type'] == 'fixed' for joint in joints.values())
    assert (
        joints['camera_mount_joint'].find('./parent').attrib['link']
        == 'base_link'
    )


def test_gazebo_depth_sensor_publishes_all_required_interfaces():
    root = _xacro_root()
    gazebo = root.find("./gazebo[@reference='camera_link']")
    sensor = gazebo.find("./sensor[@type='depth']")
    plugin = sensor.find('./plugin')

    assert sensor.findtext('always_on') == 'true'
    assert float(sensor.findtext('update_rate')) >= 10.0
    assert sensor.findtext('pose').split()[:3] == ['0.021', '0', '0']
    assert plugin.attrib['filename'] == 'libgazebo_ros_camera.so'
    assert plugin.findtext('frame_name') == 'camera_rgb_optical_frame'
    assert float(plugin.findtext('min_depth')) >= 0.1
    assert float(plugin.findtext('max_depth')) >= 3.5

    remappings = {
        item.text for item in plugin.findall('./ros/remapping')
    }
    assert {
        'camera/image_raw:=camera/color/image_raw',
        'camera/camera_info:=camera/color/camera_info',
        'camera/depth/image_raw:=camera/depth/image_raw',
        'camera/depth/camera_info:=camera/depth/camera_info',
        'camera/points:=camera/depth/points',
    } <= remappings


def test_custom_spawn_preserves_drive_lidar_and_imu_plugins():
    root = _xacro_root()
    filenames = {
        plugin.attrib['filename']
        for plugin in root.findall('.//plugin')
    }
    assert {
        'libgazebo_ros_diff_drive.so',
        'libgazebo_ros_joint_state_publisher.so',
        'libgazebo_ros_imu_sensor.so',
        'libgazebo_ros_ray_sensor.so',
        'libgazebo_ros_camera.so',
    } <= filenames


def test_rgbd_launch_and_checker_compile_and_use_custom_description():
    launch_path = PACKAGE_ROOT / 'launch' / 'rgbd_simulation.launch.py'
    checker_path = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'rgbd_smoke_check.py'
    )
    for source in (launch_path, checker_path):
        compile(source.read_text(encoding='utf-8'), str(source), 'exec')

    launch_text = launch_path.read_text(encoding='utf-8')
    assert 'turtlebot3_burger_rgbd.urdf.xacro' in launch_text
    assert "'-topic', 'robot_description'" in launch_text
    assert "'-entity', 'turtlebot3_burger_rgbd'" in launch_text


def test_manifest_and_setup_install_rgbd_runtime_requirements():
    package_root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text for element in package_root.findall('exec_depend')
    }
    assert {
        'gazebo_plugins',
        'gazebo_ros',
        'robot_state_publisher',
        'sensor_msgs',
        'tf2_ros',
        'turtlebot3_gazebo',
        'xacro',
    } <= dependencies

    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert "glob('urdf/*')" in setup_text
    assert 'rgbd_smoke_check:main' in setup_text
