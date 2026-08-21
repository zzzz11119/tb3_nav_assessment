# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Acceptance tests for optional challenge four: autonomous exploration."""

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tb3_nav_assessment.frontier import find_frontiers  # noqa: E402
from tb3_nav_assessment.frontier import GridMap  # noqa: E402
from tb3_nav_assessment.frontier import is_exploration_complete  # noqa: E402
from tb3_nav_assessment.frontier import save_grid_map  # noqa: E402


def _grid(width, height, values, resolution=0.25, yaw=0.0):
    return GridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=-1.0,
        origin_y=-2.0,
        origin_yaw=yaw,
        data=tuple(values),
    )


def _set_rectangle(values, width, rows, columns, value):
    for row in rows:
        for column in columns:
            values[row * width + column] = value


def test_grid_coordinate_round_trip_supports_rotated_map_origins():
    grid = _grid(12, 8, [-1] * 96, yaw=math.pi / 3.0)
    for cell in ((0, 0), (3, 5), (7, 11)):
        x, y = grid.cell_to_world(cell)
        assert grid.world_to_cell(x, y) == cell


def test_frontiers_are_clustered_reachable_safe_and_facing_unknown():
    width = 24
    height = 16
    values = [-1] * (width * height)
    _set_rectangle(values, width, range(3, 13), range(3, 15), 0)
    _set_rectangle(values, width, range(6, 10), range(8, 10), 100)
    grid = _grid(width, height, values, resolution=0.10)
    robot_x, robot_y = grid.cell_to_world((5, 5))

    result = find_frontiers(
        grid,
        robot_x,
        robot_y,
        min_cluster_size=4,
        min_goal_clearance_m=0.10,
    )

    assert result.frontier_cells
    assert result.cluster_count >= 1
    assert result.candidates
    candidate = result.candidates[0]
    assert candidate.goal_cell in candidate.cells
    assert grid.value(candidate.goal_cell) == 0
    assert candidate.path_distance >= 0.80
    assert candidate.information_gain > 0.0
    assert math.isfinite(candidate.score)
    assert math.isfinite(candidate.yaw)


def test_disconnected_frontier_is_rejected_as_unreachable():
    width = 30
    height = 14
    values = [-1] * (width * height)
    _set_rectangle(values, width, range(3, 11), range(2, 9), 0)
    _set_rectangle(values, width, range(3, 11), range(20, 27), 0)
    grid = _grid(width, height, values, resolution=0.10)
    robot_x, robot_y = grid.cell_to_world((6, 5))

    result = find_frontiers(
        grid,
        robot_x,
        robot_y,
        min_cluster_size=4,
        min_goal_clearance_m=0.0,
        min_goal_distance_m=0.0,
    )

    assert result.cluster_count == 2
    assert len(result.candidates) == 1
    assert result.rejected_unreachable_clusters == 1
    assert result.candidates[0].goal_cell[1] < 10


def test_no_unknown_boundary_means_exploration_is_complete_candidate_set():
    width = 18
    height = 12
    values = [0] * (width * height)
    for row in range(height):
        values[row * width] = 100
        values[row * width + width - 1] = 100
    for column in range(width):
        values[column] = 100
        values[(height - 1) * width + column] = 100
    grid = _grid(width, height, values)
    robot_x, robot_y = grid.cell_to_world((5, 5))

    result = find_frontiers(grid, robot_x, robot_y)

    assert result.frontier_cells == ()
    assert result.candidates == ()


def test_nearby_frontier_hole_is_rejected_to_prevent_goal_thrashing():
    width = 25
    height = 25
    values = [0] * (width * height)
    for row in range(11, 14):
        for column in range(11, 14):
            values[row * width + column] = -1
    grid = _grid(width, height, values, resolution=0.05)
    robot_x, robot_y = grid.cell_to_world((12, 10))

    result = find_frontiers(
        grid,
        robot_x,
        robot_y,
        min_cluster_size=4,
        min_goal_clearance_m=0.0,
        min_goal_distance_m=0.80,
    )

    assert result.frontier_cells
    assert result.candidates == ()
    assert result.rejected_near_clusters == 1


def test_high_coverage_unusable_residual_frontiers_complete_exploration():
    assert is_exploration_complete(
        frontier_cell_count=33,
        eligible_candidate_count=0,
        known_area_m2=91.485,
    )
    assert not is_exploration_complete(
        frontier_cell_count=33,
        eligible_candidate_count=1,
        known_area_m2=91.485,
    )
    assert not is_exploration_complete(
        frontier_cell_count=41,
        eligible_candidate_count=0,
        known_area_m2=91.485,
    )
    assert not is_exploration_complete(
        frontier_cell_count=33,
        eligible_candidate_count=0,
        known_area_m2=79.99,
    )
    assert is_exploration_complete(
        frontier_cell_count=0,
        eligible_candidate_count=0,
        known_area_m2=0.0,
    )


def test_completed_grid_saves_nav2_compatible_yaml_and_binary_pgm(tmp_path):
    values = [
        -1, 0, 100,
        0, 20, 65,
    ]
    grid = GridMap(
        width=3,
        height=2,
        resolution=0.05,
        origin_x=-4.0,
        origin_y=-3.0,
        origin_yaw=0.25,
        data=tuple(values),
    )
    yaml_path, image_path = save_grid_map(
        grid,
        tmp_path / 'completed_map.yaml',
    )

    assert yaml_path.is_file()
    assert image_path.is_file()
    yaml_text = yaml_path.read_text(encoding='utf-8')
    assert 'image: completed_map.pgm' in yaml_text
    assert 'mode: trinary' in yaml_text
    assert 'resolution: 0.05' in yaml_text
    assert 'origin: [-4, -3, 0.25]' in yaml_text
    assert 'occupied_thresh: 0.65' in yaml_text
    assert 'free_thresh: 0.2' in yaml_text

    payload = image_path.read_bytes()
    assert payload.startswith(b'P5\n# CREATOR:')
    assert payload.endswith(bytes([254, 254, 0, 205, 254, 0]))


def test_launch_and_runtime_nodes_compile_and_expose_full_pipeline():
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'autonomous_exploration.launch.py'
    )
    explorer_path = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'frontier_explorer.py'
    )
    checker_path = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'exploration_smoke_check.py'
    )
    for source in (launch_path, explorer_path, checker_path):
        compile(source.read_text(encoding='utf-8'), str(source), 'exec')

    launch_text = launch_path.read_text(encoding='utf-8')
    for required in (
        'simulation.launch.py',
        'online_async_launch.py',
        'navigation_launch.py',
        "executable='frontier_explorer'",
        "static_layer['subscribe_to_updates'] = True",
        "grid_based['plugin'] = 'nav2_navfn_planner/NavfnPlanner'",
        "grid_based['allow_unknown'] = False",
        "parameters['use_sim_time'] = enabled",
    ):
        assert required in launch_text
    assert 'bringup_launch.py' not in launch_text


def test_explorer_uses_nav2_action_feedback_blacklist_and_auto_map_save():
    explorer = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'frontier_explorer.py'
    ).read_text(encoding='utf-8')
    for required in (
        'ActionClient(',
        'NavigateToPose',
        'feedback_callback=self._on_feedback',
        'cancel_goal_async()',
        "self._state = 'complete'",
        'No frontier remains',
        'save_grid_map(',
        'AUTONOMOUS EXPLORATION COMPLETE',
        "'/exploration/status'",
        "'/exploration/frontiers'",
    ):
        assert required in explorer

    checker = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'exploration_smoke_check.py'
    ).read_text(encoding='utf-8')
    assert 'require_complete' in checker
    assert 'minimum successful goals reached' in checker
    assert 'AUTONOMOUS EXPLORATION OPTIONAL CHALLENGE CHECK' in checker


def test_manifest_setup_and_configs_install_exploration_requirements():
    package_root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text for element in package_root.findall('exec_depend')
    }
    assert {
        'action_msgs',
        'geometry_msgs',
        'nav2_bringup',
        'nav2_msgs',
        'python3-yaml',
        'slam_toolbox',
        'visualization_msgs',
    } <= dependencies

    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert 'frontier_explorer:main' in setup_text
    assert 'exploration_smoke_check:main' in setup_text
    assert (
        "data_files('config/*.yaml')" in setup_text
        or "glob('config/*.yaml')" in setup_text
    )
    assert (
        "data_files('launch/*.launch.py')" in setup_text
        or "glob('launch/*.launch.py')" in setup_text
    )

    slam = (PACKAGE_ROOT / 'config' / 'slam_toolbox.yaml').read_text(
        encoding='utf-8'
    )
    explorer = (
        PACKAGE_ROOT / 'config' / 'frontier_explorer.yaml'
    ).read_text(encoding='utf-8')
    assert 'mode: mapping' in slam
    assert 'map_update_interval: 2.0' in slam
    assert 'auto_save_map: true' in explorer
    assert 'min_frontier_size: 8' in explorer
    assert 'min_goal_distance_m: 0.80' in explorer
    assert 'max_residual_frontier_cells: 40' in explorer
    assert 'min_completion_known_area_m2: 80.0' in explorer
