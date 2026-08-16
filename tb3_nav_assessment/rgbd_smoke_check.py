#!/usr/bin/env python3
#
# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Verify the RGB-D topics and camera TF required by optional challenge one."""

import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer
from tf2_ros import TransformListener


class RgbdSmokeCheck(Node):
    """Collect camera samples and return a shell-friendly acceptance result."""

    def __init__(self):
        super().__init__('rgbd_smoke_check')
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('min_image_hz', 5.0)

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.min_image_hz = float(self.get_parameter('min_image_hz').value)
        self.started_at = time.monotonic()
        self.done = False
        self.exit_code = 1

        self.counts = {
            'scan': 0,
            'odom': 0,
            'color_image': 0,
            'color_info': 0,
            'depth_image': 0,
            'depth_info': 0,
            'points': 0,
        }
        self.valid = {key: False for key in self.counts}
        self.first_color_at = None
        self.last_color_at = None

        self._challenge_subscriptions = [
            self.create_subscription(
                LaserScan,
                '/scan',
                self._on_scan,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                '/odom',
                self._on_odom,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                '/camera/color/image_raw',
                self._on_color,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                '/camera/color/camera_info',
                lambda msg: self._on_camera_info('color_info', msg),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                '/camera/depth/image_raw',
                self._on_depth,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                '/camera/depth/camera_info',
                lambda msg: self._on_camera_info('depth_info', msg),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2,
                '/camera/depth/points',
                self._on_points,
                qos_profile_sensor_data,
            ),
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self._evaluate_at_deadline)

        self.get_logger().info(
            f'Collecting RGB-D evidence for {self.timeout_sec:.1f} s...'
        )

    def _on_scan(self, msg):
        self.counts['scan'] += 1
        self.valid['scan'] = bool(msg.ranges and msg.header.frame_id)

    def _on_odom(self, msg):
        self.counts['odom'] += 1
        self.valid['odom'] = bool(
            msg.header.frame_id and msg.child_frame_id
        )

    def _on_color(self, msg):
        now = time.monotonic()
        self.counts['color_image'] += 1
        if self.first_color_at is None:
            self.first_color_at = now
        self.last_color_at = now
        self.valid['color_image'] = bool(
            msg.width > 0
            and msg.height > 0
            and msg.encoding
            and msg.data
            and msg.header.frame_id
        )

    def _on_camera_info(self, key, msg):
        self.counts[key] += 1
        self.valid[key] = bool(
            msg.width > 0
            and msg.height > 0
            and msg.k[0] > 0.0
            and msg.k[4] > 0.0
            and msg.header.frame_id
        )

    def _on_depth(self, msg):
        self.counts['depth_image'] += 1
        self.valid['depth_image'] = bool(
            msg.width > 0
            and msg.height > 0
            and msg.encoding in {'16UC1', '32FC1'}
            and msg.data
            and msg.header.frame_id
        )

    def _on_points(self, msg):
        self.counts['points'] += 1
        self.valid['points'] = bool(
            msg.width > 0
            and msg.height > 0
            and msg.point_step > 0
            and msg.data
            and msg.header.frame_id
        )

    def _color_frequency(self):
        if (
            self.counts['color_image'] < 2
            or self.first_color_at is None
            or self.last_color_at is None
            or self.last_color_at <= self.first_color_at
        ):
            return 0.0
        return (
            (self.counts['color_image'] - 1)
            / (self.last_color_at - self.first_color_at)
        )

    def _camera_tf_available(self):
        return self.tf_buffer.can_transform(
            'base_link',
            'camera_rgb_optical_frame',
            Time(),
            timeout=Duration(seconds=0.0),
        )

    def _evaluate_at_deadline(self):
        if self.done or time.monotonic() - self.started_at < self.timeout_sec:
            return

        self.done = True
        color_hz = self._color_frequency()
        checks = {
            'stock LaserScan preserved': (
                self.counts['scan'] >= 3 and self.valid['scan']
            ),
            'stock odometry preserved': (
                self.counts['odom'] >= 3 and self.valid['odom']
            ),
            'RGB image stream': (
                self.counts['color_image'] >= 3
                and self.valid['color_image']
            ),
            'RGB camera_info': (
                self.counts['color_info'] >= 1
                and self.valid['color_info']
            ),
            'depth image stream': (
                self.counts['depth_image'] >= 3
                and self.valid['depth_image']
            ),
            'depth camera_info': (
                self.counts['depth_info'] >= 1
                and self.valid['depth_info']
            ),
            'organized point cloud': (
                self.counts['points'] >= 1
                and self.valid['points']
            ),
            f'RGB frequency >= {self.min_image_hz:.1f} Hz': (
                color_hz >= self.min_image_hz
            ),
            'base_link -> camera_rgb_optical_frame TF': (
                self._camera_tf_available()
            ),
        }

        self.get_logger().info(
            'Observed counts: '
            + ', '.join(f'{key}={value}' for key, value in self.counts.items())
        )
        self.get_logger().info(f'Observed RGB frequency: {color_hz:.2f} Hz')
        for label, passed in checks.items():
            log = self.get_logger().info if passed else self.get_logger().error
            log(f'[{"PASS" if passed else "FAIL"}] {label}')

        self.exit_code = 0 if all(checks.values()) else 1
        summary = 'PASS' if self.exit_code == 0 else 'FAIL'
        log = self.get_logger().info if self.exit_code == 0 else self.get_logger().error
        log(f'RGB-D OPTIONAL CHALLENGE CHECK: {summary}')

        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    """Run the RGB-D check and return its result as a process exit code."""
    rclpy.init(args=args)
    node = RgbdSmokeCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning('RGB-D check interrupted.')
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
