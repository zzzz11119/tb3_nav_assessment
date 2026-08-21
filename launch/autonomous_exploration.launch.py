#!/usr/bin/env python3
#
# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Run Gazebo, SLAM Toolbox, Nav2 and frontier exploration together."""

import os
from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HUMBLE_BEHAVIOR_PLUGINS = {
    'spin': 'nav2_behaviors/Spin',
    'backup': 'nav2_behaviors/BackUp',
    'drive_on_heading': 'nav2_behaviors/DriveOnHeading',
    'wait': 'nav2_behaviors/Wait',
    'assisted_teleop': 'nav2_behaviors/AssistedTeleop',
}


def _find_node_parameters(document, node_name):
    """Find one node's ros__parameters in a normal Nav2 YAML tree."""
    if not isinstance(document, dict):
        return None
    node = document.get(node_name)
    if isinstance(node, dict):
        parameters = node.get('ros__parameters')
        if isinstance(parameters, dict):
            return parameters
    for value in document.values():
        parameters = _find_node_parameters(value, node_name)
        if parameters is not None:
            return parameters
    return None


def _set_use_sim_time(document, enabled):
    """Inject simulation time even if the base YAML omits the parameter."""
    if not isinstance(document, dict):
        return 0
    updated = 0
    parameters = document.get('ros__parameters')
    if isinstance(parameters, dict):
        parameters['use_sim_time'] = enabled
        updated += 1
    for value in document.values():
        updated += _set_use_sim_time(value, enabled)
    return updated


def _remove_generated_file(context, generated_path):
    del context
    try:
        Path(generated_path).unlink(missing_ok=True)
    except OSError:
        pass
    return []


def _default_nav2_params():
    share = Path(get_package_share_directory('turtlebot3_navigation2'))
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    distro = os.environ.get('ROS_DISTRO', 'humble')
    candidates = (
        share / 'param' / distro / f'{model}.yaml',
        share / 'param' / f'{model}.yaml',
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return str(candidates[-1])


def _default_rviz_config():
    share = Path(get_package_share_directory('turtlebot3_navigation2'))
    candidates = (
        share / 'rviz' / 'tb3_navigation2.rviz',
        share / 'rviz' / 'tb3_navigation2_humble.rviz',
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return str(candidates[0])


def _navigation_setup(context):
    base_path = Path(
        LaunchConfiguration('base_nav2_params_file').perform(context)
    ).expanduser()
    if not base_path.is_file():
        raise RuntimeError(f'Nav2 base parameter file not found: {base_path}')

    document = yaml.safe_load(base_path.read_text(encoding='utf-8'))
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower()
        in {'1', 'true', 'yes', 'on'}
    )
    updated_clock_nodes = _set_use_sim_time(document, use_sim_time)
    if updated_clock_nodes == 0:
        raise RuntimeError(f'No ros__parameters sections in {base_path}')

    planner = _find_node_parameters(document, 'planner_server')
    if planner is None:
        raise RuntimeError(
            f'No planner_server.ros__parameters in {base_path}'
        )
    planner_plugins = planner.get('planner_plugins', ['GridBased'])
    if 'GridBased' not in planner_plugins:
        planner_plugins.append('GridBased')
    planner['planner_plugins'] = planner_plugins
    grid_based = planner.setdefault('GridBased', {})
    grid_based['plugin'] = 'nav2_navfn_planner/NavfnPlanner'
    grid_based['allow_unknown'] = False
    grid_based.setdefault('tolerance', 0.35)

    global_costmap = _find_node_parameters(document, 'global_costmap')
    if global_costmap is None:
        raise RuntimeError(
            f'No global_costmap.ros__parameters in {base_path}'
        )
    global_costmap['track_unknown_space'] = True
    static_layer = global_costmap.setdefault('static_layer', {})
    static_layer['map_subscribe_transient_local'] = True
    static_layer['subscribe_to_updates'] = True

    behavior = _find_node_parameters(document, 'behavior_server')
    if behavior is not None:
        for behavior_id, plugin_type in HUMBLE_BEHAVIOR_PLUGINS.items():
            plugin_parameters = behavior.get(behavior_id)
            if isinstance(plugin_parameters, dict):
                plugin_parameters['plugin'] = plugin_type

    generated = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='tb3_frontier_nav2_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    with generated:
        yaml.safe_dump(document, generated, sort_keys=False)

    nav2_share = get_package_share_directory('nav2_bringup')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': generated.name,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            OpaqueFunction(
                function=_remove_generated_file,
                args=[generated.name],
            )
        ])
    )
    return [
        LogInfo(msg=(
            f'Live-map Nav2 parameters generated at {generated.name}; '
            f'use_sim_time={use_sim_time} injected into '
            f'{updated_clock_nodes} nodes'
        )),
        navigation,
        cleanup,
    ]


def generate_launch_description():
    """Build the isolated launch for optional challenge four."""
    configured_model = os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    if configured_model != 'burger':
        raise RuntimeError(
            'Autonomous exploration is calibrated for TurtleBot3 Burger. '
            'Run: export TURTLEBOT3_MODEL=burger'
        )

    assessment_share = get_package_share_directory('tb3_nav_assessment')
    slam_share = get_package_share_directory('slam_toolbox')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                assessment_share,
                'launch',
                'simulation.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('start_simulation')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_share,
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )
    explorer = Node(
        package='tb3_nav_assessment',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[
            LaunchConfiguration('explorer_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_save_path': LaunchConfiguration('map_save_path'),
            },
        ],
    )
    delayed_explorer = TimerAction(
        period=LaunchConfiguration('explorer_start_delay_sec'),
        actions=[explorer],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_simulation', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                assessment_share,
                'config',
                'slam_toolbox.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                assessment_share,
                'config',
                'frontier_explorer.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'base_nav2_params_file',
            default_value=_default_nav2_params(),
            description=(
                'Known-good TurtleBot3 Nav2 YAML; live-map settings are '
                'overlaid without changing the file.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=_default_rviz_config(),
        ),
        DeclareLaunchArgument(
            'explorer_start_delay_sec',
            default_value='8.0',
        ),
        DeclareLaunchArgument(
            'map_save_path',
            default_value='~/.ros/maps/autonomous_exploration',
            description='Output stem for the final PGM and YAML map.',
        ),
        simulation,
        slam,
        OpaqueFunction(function=_navigation_setup),
        delayed_explorer,
        rviz,
    ])
