# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Static acceptance tests for week-one package assets."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'assessment_world.world'


def _world_models():
    root = ET.parse(WORLD_PATH).getroot()
    return {
        model.attrib['name']: model
        for model in root.findall('./world/model')
    }


def test_package_manifest_is_valid_and_declares_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    assert root.findtext('name') == 'tb3_nav_assessment'
    dependencies = {
        element.text
        for element in root.findall('exec_depend')
    }
    assert {
        'gazebo_ros',
        'rclpy',
        'tf2_ros',
        'turtlebot3_gazebo',
    } <= dependencies


def test_world_has_closed_outer_boundary():
    models = _world_models()
    separate_walls = {
        'outer_wall_north',
        'outer_wall_south',
        'outer_wall_east',
        'outer_wall_west',
    }
    assert separate_walls <= models.keys() or 'outer_walls' in models


def test_world_has_two_regions_with_wide_central_doorway():
    models = _world_models()
    if 'partition_walls' in models:
        grouped_walls = models['partition_walls']
        assert len(grouped_walls.findall('.//collision')) >= 2
        assert len(grouped_walls.findall('.//visual')) >= 2
        return

    north = models['partition_wall_north']
    south = models['partition_wall_south']
    north_y = float(north.findtext('pose').split()[1])
    south_y = float(south.findtext('pose').split()[1])
    north_length = float(
        north.findtext('./link/collision/geometry/box/size').split()[1]
    )
    south_length = float(
        south.findtext('./link/collision/geometry/box/size').split()[1]
    )
    doorway_width = (
        north_y - north_length / 2
        - (south_y + south_length / 2)
    )
    assert doorway_width >= 1.0


def test_world_has_three_different_obstacles():
    models = _world_models()
    obstacles = [
        model
        for name, model in models.items()
        if name.startswith('obstacle_')
    ]
    assert len(obstacles) >= 3

    geometry_signatures = []
    for obstacle in obstacles:
        geometry = obstacle.find('./link/collision/geometry')
        geometry_signatures.append(ET.tostring(geometry, encoding='unicode'))
    assert len(set(geometry_signatures)) == len(geometry_signatures)


def test_every_wall_and_obstacle_has_collision_and_visual_geometry():
    for name, model in _world_models().items():
        is_assessment_geometry = (
            name.startswith(('outer_wall_', 'partition_wall_', 'obstacle_'))
            or name in {'outer_walls', 'partition_walls'}
        )
        if is_assessment_geometry:
            assert model.findtext('static') == 'true'
            links = model.findall('./link')
            assert links
            for link in links:
                assert link.find('./collision/geometry') is not None
                assert link.find('./visual/geometry') is not None


def test_launch_and_python_nodes_compile():
    sources = [
        PACKAGE_ROOT / 'launch' / 'simulation.launch.py',
        (
            PACKAGE_ROOT
            / 'tb3_nav_assessment'
            / 'simulation_smoke_check.py'
        ),
    ]
    for source in sources:
        compile(
            source.read_text(encoding='utf-8'),
            str(source),
            'exec',
        )


def test_setup_installs_all_required_resource_directories():
    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    for pattern in (
        'launch/*.launch.py',
        'worlds/*.world',
        'config/*.yaml',
        'maps/*',
        'rviz/*.rviz',
        'docs/*.md',
        'docs/**/*.md',
    ):
        assert pattern in setup_text


def test_setup_registers_waypoint_navigator_console_script():
    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert 'waypoint_navigator = ' in setup_text
    assert 'tb3_nav_assessment.waypoint_navigator:main' in setup_text
