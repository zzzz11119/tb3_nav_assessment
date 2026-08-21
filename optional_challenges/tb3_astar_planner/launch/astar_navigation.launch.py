#!/usr/bin/env python3
#
# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Launch Nav2 with the custom A* planner injected into stable TB3 params."""

from pathlib import Path
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


PLUGIN_PARAMETERS = {
    'expected_planner_frequency': 5.0,
    'planner_plugins': ['AStar', 'GridBased'],
    'AStar': {
        'plugin': 'tb3_astar_planner/AStarPlanner',
        'allow_unknown': False,
        'use_eight_connected': True,
        'tolerance': 0.35,
        'cost_penalty': 2.0,
        'max_iterations': 1000000,
        'lethal_cost': 253,
    },
    # TurtleBot3's default NavigateToPose behavior tree requests GridBased.
    # Keep that stable ID, but route it to this challenge's A* implementation.
    'GridBased': {
        'plugin': 'tb3_astar_planner/AStarPlanner',
        'allow_unknown': False,
        'use_eight_connected': True,
        'tolerance': 0.35,
        'cost_penalty': 2.0,
        'max_iterations': 1000000,
        'lethal_cost': 253,
    },
}

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
    """Inject use_sim_time even when newer TB3 YAML files omit the key."""
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


def _launch_setup(context):
    base_path = Path(
        LaunchConfiguration('base_params_file').perform(context)
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
    planner_parameters = _find_node_parameters(document, 'planner_server')
    if planner_parameters is None:
        raise RuntimeError(
            f'No planner_server.ros__parameters in {base_path}'
        )
    planner_parameters.update(PLUGIN_PARAMETERS)

    behavior_parameters = _find_node_parameters(document, 'behavior_server')
    if behavior_parameters is None:
        raise RuntimeError(
            f'No behavior_server.ros__parameters in {base_path}'
        )
    for behavior_id, plugin_type in HUMBLE_BEHAVIOR_PLUGINS.items():
        plugin_parameters = behavior_parameters.setdefault(behavior_id, {})
        plugin_parameters['plugin'] = plugin_type

    generated = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='tb3_astar_nav2_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    with generated:
        yaml.safe_dump(document, generated, sort_keys=False)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': generated.name,
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn': LaunchConfiguration('use_respawn'),
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
            f'Nav2 A* parameters generated at {generated.name}; '
            f'use_sim_time={use_sim_time} injected into '
            f'{updated_clock_nodes} nodes'
        )),
        nav2_launch,
        cleanup,
    ]


def generate_launch_description():
    """Build the isolated Nav2 launch for optional challenge two."""
    default_params = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        'burger.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the saved occupancy-map YAML.',
        ),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=default_params,
            description=(
                'Known-good Nav2 YAML; only planner_server settings are '
                'overlaid for the A* challenge.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
