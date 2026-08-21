# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Static acceptance tests for optional challenge two: Nav2 A*."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]


def test_manifest_and_plugin_description_are_valid():
    manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    assert manifest.findtext('name') == 'tb3_astar_planner'
    dependencies = {
        element.text
        for tag in ('depend', 'exec_depend')
        for element in manifest.findall(tag)
    }
    assert {
        'nav2_core',
        'nav2_costmap_2d',
        'nav2_msgs',
        'nav2_util',
        'pluginlib',
        'rclcpp',
    } <= dependencies

    plugin = ET.parse(PACKAGE_ROOT / 'plugins.xml').getroot()
    exported_class = plugin.find('class')
    assert plugin.attrib['path'] == 'astar_planner'
    assert exported_class.attrib['name'] == (
        'tb3_astar_planner/AStarPlanner'
    )
    assert exported_class.attrib['base_class_type'] == (
        'nav2_core::GlobalPlanner'
    )


def test_astar_source_implements_nav2_lifecycle_and_search_contract():
    header = (
        PACKAGE_ROOT
        / 'include/tb3_astar_planner/astar_planner.hpp'
    ).read_text(encoding='utf-8')
    source = (PACKAGE_ROOT / 'src/astar_planner.cpp').read_text(
        encoding='utf-8'
    )
    for method in (
        'configure(',
        'cleanup()',
        'activate()',
        'deactivate()',
        'createPlan(',
    ):
        assert method in header
    for required in (
        'std::priority_queue',
        'g_score',
        'heuristic(',
        'traversalCost(',
        'diagonalMoveIsSafe(',
        'worldToMap(',
        'mapToWorld(',
        'PLUGINLIB_EXPORT_CLASS',
    ):
        assert required in source


def test_planner_checks_bounds_obstacles_unknown_and_no_corner_cutting():
    source = (PACKAGE_ROOT / 'src/astar_planner.cpp').read_text(
        encoding='utf-8'
    )
    assert 'start lies outside the costmap' in source
    assert 'goal lies outside the costmap' in source
    assert 'start lies in an occupied costmap cell' in source
    assert 'goal lies in an occupied costmap cell' in source
    assert 'NO_INFORMATION' in source
    assert 'isTraversable(static_cast<unsigned int>(side_x), y)' in source
    assert 'isTraversable(x, static_cast<unsigned int>(side_y))' in source


def test_cmake_builds_and_exports_nav2_plugin():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'add_library(astar_planner SHARED' in cmake
    assert 'pluginlib_export_plugin_description_file(nav2_core plugins.xml)' in cmake
    assert 'scripts/astar_smoke_check' in cmake
    for directory in ('config', 'docs', 'launch'):
        assert directory in cmake


def test_isolated_launch_overlays_only_planner_configuration():
    launch_path = PACKAGE_ROOT / 'launch/astar_navigation.launch.py'
    compile(launch_path.read_text(encoding='utf-8'), str(launch_path), 'exec')
    launch_text = launch_path.read_text(encoding='utf-8')
    assert "FindPackageShare('nav2_bringup')" in launch_text
    assert "FindPackageShare('turtlebot3_navigation2')" in launch_text
    assert "'planner_plugins': ['AStar', 'GridBased']" in launch_text
    assert "'plugin': 'tb3_astar_planner/AStarPlanner'" in launch_text
    assert "'GridBased': {" in launch_text
    assert "'spin': 'nav2_behaviors/Spin'" in launch_text
    assert "'backup': 'nav2_behaviors/BackUp'" in launch_text
    assert "parameters['use_sim_time'] = enabled" in launch_text
    assert "updated_clock_nodes = _set_use_sim_time" in launch_text
    assert "'base_params_file'" in launch_text


def test_runtime_checker_requests_named_planner_and_validates_detour():
    checker_path = PACKAGE_ROOT / 'scripts/astar_smoke_check'
    compile(checker_path.read_text(encoding='utf-8'), str(checker_path), 'exec')
    checker = checker_path.read_text(encoding='utf-8')
    assert 'ComputePathToPose' in checker
    assert "'planner_id': 'AStar'" in checker
    assert 'path avoids occupied map cells' in checker
    assert 'direct route crosses an obstacle' in checker
    assert 'AStar route makes a real detour' in checker
    assert 'A* OPTIONAL CHALLENGE CHECK: {summary}' in checker
