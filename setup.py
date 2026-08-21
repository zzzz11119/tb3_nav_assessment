# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'tb3_nav_assessment'


def data_files(pattern):
    """Return install tuples for every file matched by a package-relative glob."""
    matches = [path for path in glob(pattern) if os.path.isfile(path)]
    grouped = {}
    for path in matches:
        destination = os.path.join('share', package_name, os.path.dirname(path))
        grouped.setdefault(destination, []).append(path)
    return list(grouped.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        *data_files('launch/*.launch.py'),
        *data_files('urdf/*.urdf'),
        *data_files('urdf/*.xacro'),
        *data_files('worlds/*.world'),
        *data_files('config/*.yaml'),
        *data_files('maps/*'),
        *data_files('rviz/*.rviz'),
        *data_files('docs/*.md'),
        *data_files('models/*.md'),
        *data_files('models/yolo/*.onnx'),
        *data_files('models/*/*.config'),
        *data_files('models/*/*.sdf'),
        *data_files('models/*/materials/scripts/*'),
        *data_files('models/*/materials/textures/*'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Assessment Student',
    maintainer_email='student@example.com',
    description=(
        'TurtleBot3 Burger simulation assets for the stage-one '
        'autonomous-navigation assessment.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'simulation_smoke_check = '
                'tb3_nav_assessment.simulation_smoke_check:main'
            ),
            (
                'rgbd_smoke_check = '
                'tb3_nav_assessment.rgbd_smoke_check:main'
            ),
            (
                'yolo_object_detector = '
                'tb3_nav_assessment.yolo_object_detector:main'
            ),
            (
                'perception_smoke_check = '
                'tb3_nav_assessment.perception_smoke_check:main'
            ),
            (
                'frontier_explorer = '
                'tb3_nav_assessment.frontier_explorer:main'
            ),
            (
                'exploration_smoke_check = '
                'tb3_nav_assessment.exploration_smoke_check:main'
            ),
        ],
    },
)
