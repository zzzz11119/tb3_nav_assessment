#!/usr/bin/env python3
#
# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Launch the assessment world with the custom RGB-D TurtleBot3 Burger."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build an isolated launch description for optional challenge one."""
    assessment_share = get_package_share_directory('tb3_nav_assessment')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('tb3_nav_assessment'),
        'urdf',
        'turtlebot3_burger_rgbd.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': verbose,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rgbd_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_burger_rgbd',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
        ],
    )

    return LaunchDescription([
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
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
    ])
