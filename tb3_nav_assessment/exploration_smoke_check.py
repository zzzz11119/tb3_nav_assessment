# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Runtime acceptance check for autonomous frontier exploration."""

import json
from pathlib import Path
import sys
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


class ExplorationSmokeCheck(Node):
    """Validate live SLAM data, progress and optional completion."""

    def __init__(self):
        super().__init__('exploration_smoke_check')
        defaults = {
            'timeout_sec': 45.0,
            'require_complete': False,
            'min_known_area_m2': 8.0,
            'min_successful_goals': 1,
            'require_saved_map': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._status = None
        self._map = None
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_subscription = self.create_subscription(
            String,
            '/exploration/status',
            self._on_status,
            latched_qos,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            latched_qos,
        )

    def _on_status(self, message):
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self._status = payload

    def _on_map(self, message):
        self._map = message

    def _ready(self):
        if self._status is None or self._map is None:
            return False
        if bool(self.get_parameter('require_complete').value):
            return self._status.get('state') in {'complete', 'error'}
        return (
            int(self._status.get('map_revision', 0)) > 0
            and self._status.get('state') not in {
                'waiting_for_map',
                'warming_up',
                'waiting_for_tf',
                'waiting_for_nav2',
            }
        )

    def run(self):
        timeout = float(self.get_parameter('timeout_sec').value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._ready():
            rclpy.spin_once(self, timeout_sec=0.2)

        checks = {
            'exploration status received': self._status is not None,
            'SLAM occupancy map received': self._map is not None,
        }
        if self._status is None or self._map is None:
            return self._report(checks)

        map_values = [int(value) for value in self._map.data]
        resolution = float(self._map.info.resolution)
        known_cells = sum(1 for value in map_values if value >= 0)
        known_area = known_cells * resolution * resolution
        checks.update({
            'map contains known free space': any(
                0 <= value <= 20 for value in map_values
            ),
            'map contains occupied structure': any(
                value >= 65 for value in map_values
            ),
            'known map area meets threshold': known_area >= float(
                self.get_parameter('min_known_area_m2').value
            ),
            'explorer processed a map revision': int(
                self._status.get('map_revision', 0)
            ) > 0,
            'explorer is not in error state': (
                self._status.get('state') != 'error'
            ),
        })

        require_complete = bool(
            self.get_parameter('require_complete').value
        )
        if require_complete:
            checks['frontier exploration completed'] = (
                self._status.get('state') == 'complete'
            )
            checks['minimum successful goals reached'] = int(
                self._status.get('goals_succeeded', 0)
            ) >= int(self.get_parameter('min_successful_goals').value)
            if bool(self.get_parameter('require_saved_map').value):
                yaml_path = Path(
                    str(self._status.get('saved_map_yaml', ''))
                ).expanduser()
                image_path = Path(
                    str(self._status.get('saved_map_image', ''))
                ).expanduser()
                checks['completed map YAML exists'] = (
                    bool(str(yaml_path))
                    and yaml_path.is_file()
                    and yaml_path.stat().st_size > 0
                )
                checks['completed map image exists'] = (
                    bool(str(image_path))
                    and image_path.is_file()
                    and image_path.stat().st_size > 0
                )
        else:
            checks['frontier processing is active'] = (
                int(self._status.get('frontier_cells', 0)) > 0
                or int(self._status.get('goals_sent', 0)) > 0
                or self._status.get('state') in {
                    'confirming_complete',
                    'complete',
                }
            )

        self.get_logger().info(
            f"State={self._status.get('state')}, "
            f'known_area={known_area:.2f} m^2, '
            f"frontiers={self._status.get('frontier_cells', 0)}, "
            f"goals={self._status.get('goals_sent', 0)}, "
            f"success={self._status.get('goals_succeeded', 0)}, "
            f"failed={self._status.get('goals_failed', 0)}"
        )
        return self._report(checks)

    def _report(self, checks):
        for label, passed in checks.items():
            if passed:
                self.get_logger().info(f'[PASS] {label}')
            else:
                self.get_logger().error(f'[FAIL] {label}')
        success = bool(checks) and all(checks.values())
        summary = 'PASS' if success else 'FAIL'
        if success:
            self.get_logger().info(
                f'AUTONOMOUS EXPLORATION OPTIONAL CHALLENGE CHECK: {summary}'
            )
        else:
            self.get_logger().error(
                f'AUTONOMOUS EXPLORATION OPTIONAL CHALLENGE CHECK: {summary}'
            )
        return 0 if success else 1


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationSmokeCheck()
    try:
        return node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('Exploration check interrupted.')
        return 130
    except Exception as error:
        node.get_logger().error(f'Exploration check failed: {error}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
