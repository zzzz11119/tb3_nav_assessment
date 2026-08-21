#!/usr/bin/env python3
#
# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Runtime acceptance check for optional challenge three."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PerceptionSmokeCheck(Node):
    """Check raw preservation, annotated output and model detections."""

    def __init__(self):
        super().__init__('perception_smoke_check')
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('min_annotated_hz', 1.0)
        self.declare_parameter('require_detection', True)

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.min_annotated_hz = float(
            self.get_parameter('min_annotated_hz').value
        )
        self.require_detection = bool(
            self.get_parameter('require_detection').value
        )
        self.started_at = time.monotonic()
        self.first_annotated_at = None
        self.last_annotated_at = None
        self.raw_count = 0
        self.annotated_count = 0
        self.summary_count = 0
        self.valid_summary_count = 0
        self.detected_labels = set()
        self.done = False
        self.exit_code = 1

        self._challenge_subscriptions = [
            self.create_subscription(
                Image,
                '/camera/color/image_raw',
                self._on_raw,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                '/perception/yolo/annotated_image',
                self._on_annotated,
                10,
            ),
            self.create_subscription(
                String,
                '/perception/yolo/detections',
                self._on_summary,
                10,
            ),
        ]
        self.timer = self.create_timer(0.2, self._evaluate_at_deadline)
        self.get_logger().info(
            f'Collecting visual-perception evidence for '
            f'{self.timeout_sec:.1f} s...'
        )

    def _on_raw(self, message):
        if message.width > 0 and message.height > 0 and message.data:
            self.raw_count += 1

    def _on_annotated(self, message):
        if not (message.width > 0 and message.height > 0 and message.data):
            return
        now = time.monotonic()
        self.annotated_count += 1
        if self.first_annotated_at is None:
            self.first_annotated_at = now
        self.last_annotated_at = now

    def _on_summary(self, message):
        self.summary_count += 1
        try:
            payload = json.loads(message.data)
            detections = payload['detections']
            if (
                payload.get('model') == 'YOLOv5n-COCO-ONNX'
                and payload.get('count') == len(detections)
                and isinstance(payload.get('latency_ms'), (int, float))
            ):
                self.valid_summary_count += 1
                self.detected_labels.update(
                    item['label']
                    for item in detections
                    if item.get('label')
                )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    def _annotated_frequency(self):
        if (
            self.annotated_count < 2
            or self.first_annotated_at is None
            or self.last_annotated_at is None
            or self.last_annotated_at <= self.first_annotated_at
        ):
            return 0.0
        return (
            (self.annotated_count - 1)
            / (self.last_annotated_at - self.first_annotated_at)
        )

    def _evaluate_at_deadline(self):
        if self.done or time.monotonic() - self.started_at < self.timeout_sec:
            return

        self.done = True
        annotated_hz = self._annotated_frequency()
        checks = {
            'original RGB stream preserved': self.raw_count >= 3,
            'annotated image stream': self.annotated_count >= 3,
            'structured detection summaries': self.valid_summary_count >= 3,
            f'annotated frequency >= {self.min_annotated_hz:.1f} Hz': (
                annotated_hz >= self.min_annotated_hz
            ),
        }
        if self.require_detection:
            checks['at least one pretrained-model detection'] = bool(
                self.detected_labels
            )

        self.get_logger().info(
            f'Observed raw={self.raw_count}, '
            f'annotated={self.annotated_count}, '
            f'summaries={self.summary_count}, '
            f'valid_summaries={self.valid_summary_count}, '
            f'annotated_hz={annotated_hz:.2f}'
        )
        self.get_logger().info(
            'Observed labels: '
            + (', '.join(sorted(self.detected_labels)) or 'none')
        )
        for label, passed in checks.items():
            if passed:
                self.get_logger().info(f'[PASS] {label}')
            else:
                self.get_logger().error(f'[FAIL] {label}')

        self.exit_code = 0 if all(checks.values()) else 1
        summary = 'PASS' if self.exit_code == 0 else 'FAIL'
        if self.exit_code == 0:
            self.get_logger().info(
                f'VISUAL PERCEPTION OPTIONAL CHALLENGE CHECK: {summary}'
            )
        else:
            self.get_logger().error(
                f'VISUAL PERCEPTION OPTIONAL CHALLENGE CHECK: {summary}'
            )
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    """Run the acceptance check and return a shell-friendly status."""
    rclpy.init(args=args)
    node = PerceptionSmokeCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning('Perception check interrupted.')
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
