#!/usr/bin/env python3
#
# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 node for pretrained YOLO detection on the Burger RGB-D stream."""

import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .yolo_inference import annotate_detections
from .yolo_inference import OpenCVDnnYolo


class YoloObjectDetector(Node):
    """Detect COCO objects without changing the original camera topic."""

    def __init__(self):
        super().__init__('yolo_object_detector')
        self._declare_parameters()

        model_path = Path(self.get_parameter('model_path').value)
        if not model_path.is_file():
            raise RuntimeError(
                f'YOLO ONNX model not found: {model_path}. Rebuild the '
                'workspace so package model assets are installed.'
            )

        self.max_processing_hz = float(
            self.get_parameter('max_processing_hz').value
        )
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.max_depth_age_sec = float(
            self.get_parameter('max_depth_age_sec').value
        )
        self.bridge = CvBridge()
        self.last_processed_at = None
        self.last_depth = None
        self.last_depth_stamp = None
        self.last_error_log_at = 0.0
        self.last_detection_log_at = 0.0

        self.detector = OpenCVDnnYolo(
            model_path=model_path,
            confidence_threshold=float(
                self.get_parameter('confidence_threshold').value
            ),
            iou_threshold=float(
                self.get_parameter('iou_threshold').value
            ),
            image_size=int(self.get_parameter('image_size').value),
            max_detections=int(
                self.get_parameter('max_detections').value
            ),
            class_filter=list(self.get_parameter('class_filter').value),
        )

        input_topic = self.get_parameter('input_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        annotated_topic = self.get_parameter('annotated_image_topic').value
        detections_topic = self.get_parameter('detections_topic').value

        self.annotated_publisher = self.create_publisher(
            Image,
            annotated_topic,
            10,
        )
        self.detections_publisher = self.create_publisher(
            String,
            detections_topic,
            10,
        )
        self.image_subscription = self.create_subscription(
            Image,
            input_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.depth_subscription = None
        if self.use_depth:
            self.depth_subscription = self.create_subscription(
                Image,
                depth_topic,
                self._on_depth,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            'Pretrained YOLO detector ready. Input remains unchanged at '
            f'{input_topic}; annotated output: {annotated_topic}'
        )

    def _declare_parameters(self):
        self.declare_parameter('model_path', '')
        self.declare_parameter(
            'input_image_topic',
            '/camera/color/image_raw',
        )
        self.declare_parameter(
            'depth_image_topic',
            '/camera/depth/image_raw',
        )
        self.declare_parameter(
            'annotated_image_topic',
            '/perception/yolo/annotated_image',
        )
        self.declare_parameter(
            'detections_topic',
            '/perception/yolo/detections',
        )
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('max_detections', 50)
        self.declare_parameter('max_processing_hz', 5.0)
        self.declare_parameter('class_filter', ['*'])
        self.declare_parameter('use_depth', True)
        self.declare_parameter('max_depth_age_sec', 0.5)

    def _on_depth(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='passthrough',
            )
            depth = np.asarray(depth)
            if message.encoding == '16UC1':
                depth = depth.astype(np.float32) / 1000.0
            elif message.encoding == '32FC1':
                depth = depth.astype(np.float32, copy=False)
            else:
                self._log_error_throttled(
                    f'Unsupported depth encoding: {message.encoding}'
                )
                return
            self.last_depth = depth
            self.last_depth_stamp = self._stamp_seconds(
                message.header.stamp
            )
        except (CvBridgeError, ValueError) as error:
            self._log_error_throttled(f'Depth conversion failed: {error}')

    def _on_image(self, message):
        now = time.monotonic()
        minimum_period = 1.0 / max(self.max_processing_hz, 0.1)
        if self.last_processed_at is not None:
            if now - self.last_processed_at < minimum_period:
                return
        self.last_processed_at = now

        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
            started_at = time.perf_counter()
            detections = self.detector.detect(image)
            detections = self._attach_depth(detections, message)
            latency_ms = (time.perf_counter() - started_at) * 1000.0

            annotated = annotate_detections(
                image,
                detections,
                latency_ms,
            )
            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding='bgr8',
            )
            annotated_message.header = message.header
            self.annotated_publisher.publish(annotated_message)
            self.detections_publisher.publish(String(
                data=json.dumps(
                    self._summary(message, detections, latency_ms),
                    ensure_ascii=True,
                    separators=(',', ':'),
                )
            ))
            self._log_detections(detections, latency_ms)
        except (cv2.error, CvBridgeError, RuntimeError, ValueError) as error:
            self._log_error_throttled(f'YOLO frame processing failed: {error}')

    def _attach_depth(self, detections, image_message):
        if not self.use_depth or self.last_depth is None:
            return detections

        image_stamp = self._stamp_seconds(image_message.header.stamp)
        if self.last_depth_stamp is not None and image_stamp > 0.0:
            depth_age = abs(image_stamp - self.last_depth_stamp)
            if depth_age > self.max_depth_age_sec:
                return detections

        enriched = []
        depth_height, depth_width = self.last_depth.shape[:2]
        image_width = max(1, int(image_message.width))
        image_height = max(1, int(image_message.height))
        scale_x = depth_width / image_width
        scale_y = depth_height / image_height
        for detection in detections:
            x1 = max(0, round(detection.x1 * scale_x))
            y1 = max(0, round(detection.y1 * scale_y))
            x2 = min(depth_width, round(detection.x2 * scale_x))
            y2 = min(depth_height, round(detection.y2 * scale_y))
            region = self.last_depth[y1:y2, x1:x2]
            finite = region[
                np.isfinite(region) & (region >= 0.12) & (region <= 5.0)
            ]
            distance = None
            if finite.size:
                distance = float(np.median(finite))
            enriched.append(detection.with_distance(distance))
        return enriched

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    @staticmethod
    def _summary(message, detections, latency_ms):
        return {
            'model': 'YOLOv5n-COCO-ONNX',
            'frame_id': message.header.frame_id,
            'stamp': {
                'sec': message.header.stamp.sec,
                'nanosec': message.header.stamp.nanosec,
            },
            'latency_ms': round(latency_ms, 2),
            'count': len(detections),
            'detections': [
                {
                    'class_id': item.class_id,
                    'label': item.label,
                    'confidence': round(item.confidence, 4),
                    'bbox_xyxy': [item.x1, item.y1, item.x2, item.y2],
                    'distance_m': (
                        None
                        if item.distance_m is None
                        else round(item.distance_m, 3)
                    ),
                }
                for item in detections
            ],
        }

    def _log_detections(self, detections, latency_ms):
        now = time.monotonic()
        if now - self.last_detection_log_at < 1.0:
            return
        self.last_detection_log_at = now
        labels = ', '.join(item.label for item in detections) or 'none'
        self.get_logger().info(
            f'YOLO: {len(detections)} objects [{labels}], '
            f'{latency_ms:.1f} ms'
        )

    def _log_error_throttled(self, message):
        now = time.monotonic()
        if now - self.last_error_log_at >= 2.0:
            self.last_error_log_at = now
            self.get_logger().error(message)


def main(args=None):
    """Run the pretrained detector until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloObjectDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
