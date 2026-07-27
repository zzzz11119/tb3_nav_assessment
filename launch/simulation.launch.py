import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    assessment_share = get_package_share_directory(
        'tb3_nav_assessment'
    )
    gazebo_ros_share = get_package_share_directory(
        'gazebo_ros'
    )
    turtlebot3_gazebo_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    world_path = os.path.join(
        assessment_share,
        'worlds',
        'assessment_world.world',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gzserver.launch.py',
            )
        ),
        launch_arguments={
            'world': world_path,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gzclient.launch.py',
            )
        ),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_share,
                'launch',
                'robot_state_publisher.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_share,
                'launch',
                'spawn_turtlebot3.launch.py',
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='-4.0',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-2.5',
        ),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_turtlebot,
    ])
