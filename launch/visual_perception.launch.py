#!/usr/bin/env python3
#
# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Launch original RGB-D mode or isolated pretrained detection mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _mode_is_detection(mode):
    return IfCondition(PythonExpression([
        "'",
        mode,
        "' == 'detection'",
    ]))


def _detection_view_enabled(mode, show_result):
    return IfCondition(PythonExpression([
        "'",
        mode,
        "' == 'detection' and '",
        show_result,
        "' == 'true'",
    ]))


def _demo_target_enabled(mode, demo_target):
    return IfCondition(PythonExpression([
        "'",
        mode,
        "' == 'detection' and '",
        demo_target,
        "' == 'true'",
    ]))


def generate_launch_description():
    """Keep the RGB-D baseline and add detection only when requested."""
    assessment_share = get_package_share_directory('tb3_nav_assessment')
    mode = LaunchConfiguration('mode')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    demo_target = LaunchConfiguration('demo_target')
    show_result = LaunchConfiguration('show_result')

    package_share = FindPackageShare('tb3_nav_assessment')
    detector_config = PathJoinSubstitution([
        package_share,
        'config',
        'visual_perception.yaml',
    ])
    model_path = PathJoinSubstitution([
        package_share,
        'models',
        'yolo',
        'yolov5n.onnx',
    ])
    poster_sdf = PathJoinSubstitution([
        package_share,
        'models',
        'perception_demo_poster',
        'model.sdf',
    ])

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = os.pathsep.join(
        item
        for item in (
            os.path.join(assessment_share, 'models'),
            existing_model_path,
        )
        if item
    )

    rgbd_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'rgbd_simulation.launch.py',
            ])
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': verbose,
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    demo_poster = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_perception_demo_poster',
        output='screen',
        arguments=[
            '-entity', 'perception_demo_poster',
            '-file', poster_sdf,
            '-x', '-2.25',
            '-y', '-2.50',
            '-z', '0.0',
        ],
        condition=_demo_target_enabled(mode, demo_target),
    )

    detector = Node(
        package='tb3_nav_assessment',
        executable='yolo_object_detector',
        name='yolo_object_detector',
        output='screen',
        parameters=[
            detector_config,
            {
                'model_path': model_path,
                'use_sim_time': use_sim_time,
            },
        ],
        condition=_mode_is_detection(mode),
    )

    result_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='yolo_result_viewer',
        output='screen',
        arguments=['/perception/yolo/annotated_image'],
        condition=_detection_view_enabled(mode, show_result),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='detection',
            choices=['original', 'detection'],
            description=(
                'original keeps the RGB-D baseline only; detection adds '
                'the pretrained YOLO node and optional demo target.'
            ),
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                assessment_share,
                'worlds',
                'assessment_world.world',
            ),
            description='Absolute path to the Gazebo Classic world file.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose Gazebo server logging.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo /clock topic.',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='-4.0',
            description='Robot spawn x coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-2.5',
            description='Robot spawn y coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Robot spawn yaw in radians.',
        ),
        DeclareLaunchArgument(
            'demo_target',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn the offline COCO poster in detection mode.',
        ),
        DeclareLaunchArgument(
            'show_result',
            default_value='false',
            choices=['true', 'false'],
            description='Open rqt_image_view on the annotated topic.',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path,
        ),
        rgbd_simulation,
        demo_poster,
        detector,
        result_viewer,
    ])
