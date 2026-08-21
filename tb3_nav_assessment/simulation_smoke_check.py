#!/usr/bin/env python3
#
# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Verify the minimum ROS graph required by the week-one simulation."""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from tf2_ros import TransformListener


class SimulationSmokeCheck(Node):
    """Collect topic samples and report whether the simulation is usable."""

    def __init__(self):
        super().__init__('simulation_smoke_check')
        self.declare_parameter('timeout_sec', 12.0)
        self.declare_parameter('min_scan_hz', 3.0)

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.min_scan_hz = float(self.get_parameter('min_scan_hz').value)
        self.started_at = time.monotonic()
        self.exit_code = 1
        self.done = False

        self.counts = {
            '/scan': 0,
            '/odom': 0,
            '/clock': 0,
            '/tf': 0,
            '/tf_static': 0,
        }
        self.scan_samples_with_ranges = 0
        self.first_scan_at = None
        self.last_scan_at = None

        transient_local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscriptions = [
            self.create_subscription(
                LaserScan,
                '/scan',
                self._on_scan,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                '/odom',
                lambda _msg: self._count('/odom'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Clock,
                '/clock',
                lambda _msg: self._count('/clock'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                TFMessage,
                '/tf',
                lambda msg: self._count_transforms('/tf', msg),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                TFMessage,
                '/tf_static',
                lambda msg: self._count_transforms('/tf_static', msg),
                transient_local_qos,
            ),
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self._evaluate_at_deadline)

        self.get_logger().info(
            f'Collecting simulation evidence for {self.timeout_sec:.1f} s...'
        )

    def _count(self, topic):
        self.counts[topic] += 1

    def _count_transforms(self, topic, msg):
        self.counts[topic] += len(msg.transforms)

    def _on_scan(self, msg):
        now = time.monotonic()
        self.counts['/scan'] += 1
        if self.first_scan_at is None:
            self.first_scan_at = now
        self.last_scan_at = now
        if msg.ranges and any(math.isfinite(value) for value in msg.ranges):
            self.scan_samples_with_ranges += 1

    def _scan_frequency(self):
        if (
            self.counts['/scan'] < 2
            or self.first_scan_at is None
            or self.last_scan_at is None
            or self.last_scan_at <= self.first_scan_at
        ):
            return 0.0
        return (
            (self.counts['/scan'] - 1)
            / (self.last_scan_at - self.first_scan_at)
        )

    def _transform_available(self, target, source):
        return self.tf_buffer.can_transform(
            target,
            source,
            Time(),
            timeout=Duration(seconds=0.0),
        )

    def _evaluate_at_deadline(self):
        if self.done or time.monotonic() - self.started_at < self.timeout_sec:
            return

        self.done = True
        scan_hz = self._scan_frequency()
        checks = {
            '/scan messages': self.counts['/scan'] >= 3,
            '/scan contains finite ranges': self.scan_samples_with_ranges >= 1,
            f'/scan frequency >= {self.min_scan_hz:.1f} Hz': (
                scan_hz >= self.min_scan_hz
            ),
            '/odom messages': self.counts['/odom'] >= 3,
            '/clock messages': self.counts['/clock'] >= 3,
            '/tf transforms': self.counts['/tf'] >= 1,
            '/tf_static transforms': self.counts['/tf_static'] >= 1,
            'odom -> base_footprint TF': self._transform_available(
                'odom',
                'base_footprint',
            ),
            'base_footprint -> base_scan TF': self._transform_available(
                'base_footprint',
                'base_scan',
            ),
        }

        self.get_logger().info(
            'Observed counts: '
            + ', '.join(f'{key}={value}' for key, value in self.counts.items())
        )
        self.get_logger().info(f'Observed /scan frequency: {scan_hz:.2f} Hz')
        for label, passed in checks.items():
            log = self.get_logger().info if passed else self.get_logger().error
            log(f'[{"PASS" if passed else "FAIL"}] {label}')

        self.exit_code = 0 if all(checks.values()) else 1
        if self.exit_code == 0:
            self.get_logger().info('WEEK01 SIMULATION SMOKE CHECK: PASS')
        else:
            self.get_logger().error('WEEK01 SIMULATION SMOKE CHECK: FAIL')

        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    """Run the smoke check and return a shell-friendly status code."""
    rclpy.init(args=args)
    node = SimulationSmokeCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning('Smoke check interrupted.')
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
