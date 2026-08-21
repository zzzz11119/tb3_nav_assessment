# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Acceptance tests for optional challenge three: visual perception."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from tb3_nav_assessment.yolo_inference import annotate_detections  # noqa: E402
from tb3_nav_assessment.yolo_inference import OpenCVDnnYolo  # noqa: E402


MODEL_PATH = PACKAGE_ROOT / 'models' / 'yolo' / 'yolov5n.onnx'
DEMO_IMAGE_PATH = (
    PACKAGE_ROOT
    / 'models'
    / 'perception_demo_poster'
    / 'materials'
    / 'textures'
    / 'bus.jpg'
)


def test_bundled_pretrained_model_detects_official_demo_target():
    assert MODEL_PATH.stat().st_size > 7_000_000
    assert DEMO_IMAGE_PATH.stat().st_size > 100_000
    image = cv2.imread(str(DEMO_IMAGE_PATH))
    assert image is not None

    detector = OpenCVDnnYolo(
        MODEL_PATH,
        confidence_threshold=0.25,
        class_filter=['person', 'bus'],
    )
    detections = detector.detect(image)
    labels = {item.label for item in detections}
    assert {'person', 'bus'} <= labels
    assert all(item.width > 0 and item.height > 0 for item in detections)
    assert max(item.confidence for item in detections) > 0.8

    annotated = annotate_detections(image, detections, latency_ms=10.0)
    assert annotated.shape == image.shape
    assert (annotated != image).any()

    simulated_camera = np.full((480, 640, 3), 185, dtype=np.uint8)
    projected_poster = cv2.resize(image, (197, 263))
    simulated_camera[100:363, 222:419] = projected_poster
    projected_detections = detector.detect(simulated_camera)
    assert 'bus' in {item.label for item in projected_detections}


def test_launch_exposes_original_and_detection_modes():
    launch_path = PACKAGE_ROOT / 'launch' / 'visual_perception.launch.py'
    detector_path = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'yolo_object_detector.py'
    )
    checker_path = (
        PACKAGE_ROOT
        / 'tb3_nav_assessment'
        / 'perception_smoke_check.py'
    )
    for source in (launch_path, detector_path, checker_path):
        compile(source.read_text(encoding='utf-8'), str(source), 'exec')

    launch_text = launch_path.read_text(encoding='utf-8')
    assert "choices=['original', 'detection']" in launch_text
    assert 'rgbd_simulation.launch.py' in launch_text
    assert 'yolo_object_detector' in launch_text
    assert 'spawn_perception_demo_poster' in launch_text
    assert 'condition=_mode_is_detection(mode)' in launch_text


def test_detector_preserves_raw_topic_and_uses_separate_outputs():
    config_text = (
        PACKAGE_ROOT / 'config' / 'visual_perception.yaml'
    ).read_text(encoding='utf-8')
    assert 'input_image_topic: /camera/color/image_raw' in config_text
    assert (
        'annotated_image_topic: /perception/yolo/annotated_image'
        in config_text
    )
    assert 'detections_topic: /perception/yolo/detections' in config_text
    assert 'use_depth: true' in config_text


def test_demo_poster_is_static_physical_and_textured():
    model_root = ET.parse(
        PACKAGE_ROOT / 'models' / 'perception_demo_poster' / 'model.sdf'
    ).getroot()
    model = model_root.find('./model')
    assert model.findtext('static') == 'true'
    assert model.find('./link/collision/geometry/box') is not None
    script = model.find('./link/visual/material/script')
    assert script is not None
    assert script.findtext('name') == 'Perception/DemoPoster'


def test_manifest_and_setup_install_perception_runtime_assets():
    package_root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text for element in package_root.findall('exec_depend')
    }
    assert {
        'cv_bridge',
        'python3-numpy',
        'python3-opencv',
        'rqt_image_view',
        'std_msgs',
    } <= dependencies

    setup_text = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    for required in (
        'config/*.yaml',
        'models/yolo/*.onnx',
        'models/*/*.sdf',
        'models/*/materials/scripts/*',
        'models/*/materials/textures/*',
        'yolo_object_detector:main',
        'perception_smoke_check:main',
    ):
        assert required in setup_text
