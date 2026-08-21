# Copyright 2026 Assessment Student
# SPDX-License-Identifier: Apache-2.0

"""Autonomous frontier exploration node for ROS 2 Humble and Nav2."""

import json
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from .frontier import find_frontiers
from .frontier import GridMap
from .frontier import is_exploration_complete
from .frontier import save_grid_map


class FrontierExplorer(Node):
    """Continuously select frontier goals and execute them through Nav2."""

    def __init__(self):
        super().__init__('frontier_explorer')
        defaults = {
            'map_topic': '/map',
            'action_name': 'navigate_to_pose',
            'global_frame': 'map',
            'robot_frame': 'base_footprint',
            'planning_period_sec': 2.0,
            'initial_wait_sec': 8.0,
            'completion_timeout_sec': 15.0,
            'max_residual_frontier_cells': 40,
            'min_completion_known_area_m2': 80.0,
            'stalled_timeout_sec': 90.0,
            'goal_timeout_sec': 120.0,
            'blacklist_radius_m': 0.55,
            'blacklist_expiry_sec': 180.0,
            'min_frontier_size': 8,
            'min_goal_clearance_m': 0.22,
            'min_goal_distance_m': 0.80,
            'free_threshold': 20,
            'occupied_threshold': 65,
            'information_gain_weight': 2.0,
            'distance_weight': 1.0,
            'max_goals': 100,
            'initial_goals_sent': 0,
            'initial_goals_succeeded': 0,
            'initial_goals_failed': 0,
            'auto_save_map': True,
            'map_save_path': '~/.ros/maps/autonomous_exploration',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._grid = None
        self._map_message = None
        self._map_revision = 0
        self._required_map_revision = 0
        self._latest_result = None
        self._state = 'waiting_for_map'
        self._completion_reason = ''
        self._start_monotonic = time.monotonic()
        self._no_frontier_since = None
        self._stalled_since = None
        self._active_goal = None
        self._active_goal_handle = None
        self._goal_started_monotonic = None
        self._cancel_requested = False
        self._blacklist = []
        self._goals_sent = int(
            self.get_parameter('initial_goals_sent').value
        )
        self._goals_succeeded = int(
            self.get_parameter('initial_goals_succeeded').value
        )
        self._goals_failed = int(
            self.get_parameter('initial_goals_failed').value
        )
        if (
            min(
                self._goals_sent,
                self._goals_succeeded,
                self._goals_failed,
            ) < 0
            or self._goals_succeeded + self._goals_failed
            > self._goals_sent
        ):
            raise ValueError('initial goal counters are inconsistent')
        self._last_remaining_distance = None
        self._map_saved = False
        self._saved_map_yaml = ''
        self._saved_map_image = ''

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            map_qos,
        )
        self._status_publisher = self.create_publisher(
            String,
            '/exploration/status',
            map_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            '/exploration/frontiers',
            10,
        )
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('action_name').value),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        period = float(self.get_parameter('planning_period_sec').value)
        if period <= 0.0:
            raise ValueError('planning_period_sec must be positive')
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            'Frontier explorer started; waiting for SLAM map and Nav2.'
        )

    def _on_map(self, message):
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        try:
            self._grid = GridMap(
                width=int(message.info.width),
                height=int(message.info.height),
                resolution=float(message.info.resolution),
                origin_x=float(message.info.origin.position.x),
                origin_y=float(message.info.origin.position.y),
                origin_yaw=yaw,
                data=tuple(int(value) for value in message.data),
            )
        except ValueError as error:
            self._state = 'error'
            self._completion_reason = str(error)
            self.get_logger().error(f'Invalid occupancy grid: {error}')
            return
        self._map_message = message
        self._map_revision += 1

    def _robot_position(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('global_frame').value),
                str(self.get_parameter('robot_frame').value),
                Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for map-to-robot TF: {error}',
                throttle_duration_sec=5.0,
            )
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _tick(self):
        if self._state in {'complete', 'error'}:
            self._publish_status()
            return
        if self._grid is None:
            self._state = 'waiting_for_map'
            self._publish_status()
            return
        if (
            time.monotonic() - self._start_monotonic
            < float(self.get_parameter('initial_wait_sec').value)
        ):
            self._state = 'warming_up'
            self._publish_status()
            return
        if self._active_goal_handle is not None:
            self._check_goal_timeout()
            self._state = 'navigating'
            self._publish_status()
            return
        if self._active_goal is not None:
            self._state = 'sending_goal'
            self._publish_status()
            return
        if self._goals_sent >= int(self.get_parameter('max_goals').value):
            self._state = 'stalled'
            self._completion_reason = 'maximum goal count reached'
            self._publish_status()
            return
        if self._map_revision < self._required_map_revision:
            self._state = 'waiting_for_map_update'
            self._publish_status()
            return
        self._required_map_revision = 0

        robot_position = self._robot_position()
        if robot_position is None:
            self._state = 'waiting_for_tf'
            self._publish_status()
            return
        self._latest_result = find_frontiers(
            self._grid,
            robot_position[0],
            robot_position[1],
            free_threshold=int(self.get_parameter('free_threshold').value),
            occupied_threshold=int(
                self.get_parameter('occupied_threshold').value
            ),
            min_cluster_size=int(
                self.get_parameter('min_frontier_size').value
            ),
            min_goal_clearance_m=float(
                self.get_parameter('min_goal_clearance_m').value
            ),
            min_goal_distance_m=float(
                self.get_parameter('min_goal_distance_m').value
            ),
            information_gain_weight=float(
                self.get_parameter('information_gain_weight').value
            ),
            distance_weight=float(
                self.get_parameter('distance_weight').value
            ),
        )
        self._publish_markers()
        now = time.monotonic()
        frontier_count = len(self._latest_result.frontier_cells)
        eligible_count = len(self._latest_result.candidates)
        known_area = self._known_map_area()
        completion_candidate = is_exploration_complete(
            frontier_cell_count=frontier_count,
            eligible_candidate_count=eligible_count,
            known_area_m2=known_area,
            max_residual_frontier_cells=int(
                self.get_parameter(
                    'max_residual_frontier_cells'
                ).value
            ),
            min_known_area_m2=float(
                self.get_parameter(
                    'min_completion_known_area_m2'
                ).value
            ),
        )
        if completion_candidate:
            self._stalled_since = None
            if self._no_frontier_since is None:
                self._no_frontier_since = now
                if frontier_count == 0:
                    self.get_logger().info(
                        'No frontier remains; starting completion hold timer.'
                    )
                else:
                    self.get_logger().info(
                        'High map coverage reached and only unreachable or '
                        f'unsafe residual frontiers remain ({frontier_count} '
                        'cells); starting completion hold timer.'
                    )
            hold = float(
                self.get_parameter('completion_timeout_sec').value
            )
            if now - self._no_frontier_since >= hold:
                if frontier_count == 0:
                    reason = 'no frontier remained during hold time'
                else:
                    reason = (
                        f'high coverage ({known_area:.2f} m^2); '
                        f'{frontier_count} residual frontier cells had no '
                        'reachable and safe goal'
                    )
                self._finish_complete(reason)
            else:
                self._state = 'confirming_complete'
            self._publish_status()
            return

        self._no_frontier_since = None
        self._expire_blacklist(now)
        candidate = next(
            (
                item for item in self._latest_result.candidates
                if not self._is_blacklisted(item.x, item.y)
            ),
            None,
        )
        if candidate is None:
            if self._stalled_since is None:
                self._stalled_since = now
            stalled_for = now - self._stalled_since
            expiry = float(
                self.get_parameter('blacklist_expiry_sec').value
            )
            if self._blacklist and stalled_for >= min(30.0, expiry):
                self.get_logger().warning(
                    'All frontier goals are blacklisted; clearing old '
                    'failures for one retry cycle.'
                )
                self._blacklist.clear()
                self._stalled_since = now
            elif stalled_for >= float(
                self.get_parameter('stalled_timeout_sec').value
            ):
                self._state = 'stalled'
                self._completion_reason = (
                    'frontiers remain but none is reachable and safe'
                )
            else:
                self._state = 'waiting_for_reachable_frontier'
            self._publish_status()
            return

        self._stalled_since = None
        self._send_goal(candidate)
        self._publish_status()

    def _send_goal(self, candidate):
        if not self._action_client.wait_for_server(timeout_sec=0.25):
            self._state = 'waiting_for_nav2'
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(
            self.get_parameter('global_frame').value
        )
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = candidate.x
        goal.pose.pose.position.y = candidate.y
        goal.pose.pose.orientation.z = math.sin(candidate.yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(candidate.yaw * 0.5)
        self._active_goal = candidate
        self._goal_started_monotonic = time.monotonic()
        self._cancel_requested = False
        self._goals_sent += 1
        self._state = 'sending_goal'
        self.get_logger().info(
            f'Goal {self._goals_sent}: frontier size={len(candidate.cells)}, '
            f'pose=({candidate.x:.2f}, {candidate.y:.2f}, '
            f'{candidate.yaw:.2f}), path estimate='
            f'{candidate.path_distance:.2f} m, score={candidate.score:.2f}'
        )
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to send frontier goal: {error}')
            self._mark_goal_failed()
            return
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected the frontier goal.')
            self._mark_goal_failed()
            return
        self._active_goal_handle = goal_handle
        self._state = 'navigating'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        remaining = getattr(feedback, 'distance_remaining', None)
        if remaining is not None and math.isfinite(float(remaining)):
            self._last_remaining_distance = float(remaining)

    def _on_navigation_result(self, future):
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self.get_logger().error(f'Navigation result failed: {error}')
            self._mark_goal_failed()
            return

        candidate = self._active_goal
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._goals_succeeded += 1
            self._required_map_revision = self._map_revision + 1
            if candidate is not None:
                self.get_logger().info(
                    f'Reached frontier ({candidate.x:.2f}, '
                    f'{candidate.y:.2f}); selecting from the updated map.'
                )
            self._clear_active_goal()
            self._state = 'selecting_frontier'
            return

        self.get_logger().warning(
            f'Frontier navigation ended with action status {status}; '
            'blacklisting this goal temporarily.'
        )
        self._mark_goal_failed()

    def _check_goal_timeout(self):
        if self._goal_started_monotonic is None or self._cancel_requested:
            return
        elapsed = time.monotonic() - self._goal_started_monotonic
        timeout = float(self.get_parameter('goal_timeout_sec').value)
        if elapsed < timeout:
            return
        self._cancel_requested = True
        self.get_logger().warning(
            f'Frontier goal exceeded {timeout:.1f} s; requesting cancel.'
        )
        self._active_goal_handle.cancel_goal_async()

    def _mark_goal_failed(self):
        candidate = self._active_goal
        if candidate is not None:
            self._blacklist.append((
                candidate.x,
                candidate.y,
                time.monotonic(),
            ))
        self._goals_failed += 1
        self._clear_active_goal()
        self._state = 'selecting_frontier'

    def _clear_active_goal(self):
        self._active_goal = None
        self._active_goal_handle = None
        self._goal_started_monotonic = None
        self._cancel_requested = False
        self._last_remaining_distance = None

    def _expire_blacklist(self, now):
        expiry = float(self.get_parameter('blacklist_expiry_sec').value)
        self._blacklist = [
            item for item in self._blacklist if now - item[2] < expiry
        ]

    def _is_blacklisted(self, x, y):
        radius = float(self.get_parameter('blacklist_radius_m').value)
        return any(
            math.hypot(x - item[0], y - item[1]) <= radius
            for item in self._blacklist
        )

    def _finish_complete(self, reason):
        self._state = 'complete'
        self._completion_reason = reason
        if bool(self.get_parameter('auto_save_map').value):
            try:
                self._save_map()
            except (OSError, ValueError) as error:
                self._state = 'error'
                self._completion_reason = f'map save failed: {error}'
                self.get_logger().error(self._completion_reason)
                return
        self.get_logger().info(
            'AUTONOMOUS EXPLORATION COMPLETE: '
            f'goals={self._goals_sent}, succeeded={self._goals_succeeded}, '
            f'failed={self._goals_failed}'
        )

    def _save_map(self):
        if self._grid is None:
            raise ValueError('no map is available to save')
        free_threshold = int(self.get_parameter('free_threshold').value)
        occupied_threshold = int(
            self.get_parameter('occupied_threshold').value
        )
        yaml_path, image_path = save_grid_map(
            self._grid,
            str(self.get_parameter('map_save_path').value),
            free_threshold=free_threshold,
            occupied_threshold=occupied_threshold,
        )
        self._map_saved = True
        self._saved_map_yaml = str(yaml_path)
        self._saved_map_image = str(image_path)
        self.get_logger().info(
            f'Saved completed map: {yaml_path} and {image_path}'
        )

    def _known_map_area(self):
        if self._grid is None:
            return 0.0
        known_cells = sum(1 for value in self._grid.data if value >= 0)
        return known_cells * self._grid.resolution * self._grid.resolution

    def _status_payload(self):
        result = self._latest_result
        candidate = self._active_goal
        return {
            'state': self._state,
            'completion_reason': self._completion_reason,
            'map_revision': self._map_revision,
            'required_map_revision': self._required_map_revision,
            'known_area_m2': round(self._known_map_area(), 3),
            'frontier_cells': (
                len(result.frontier_cells) if result is not None else 0
            ),
            'frontier_clusters': (
                result.cluster_count if result is not None else 0
            ),
            'eligible_clusters': (
                len(result.candidates) if result is not None else 0
            ),
            'completion_candidate': (
                is_exploration_complete(
                    frontier_cell_count=len(result.frontier_cells),
                    eligible_candidate_count=len(result.candidates),
                    known_area_m2=self._known_map_area(),
                    max_residual_frontier_cells=int(
                        self.get_parameter(
                            'max_residual_frontier_cells'
                        ).value
                    ),
                    min_known_area_m2=float(
                        self.get_parameter(
                            'min_completion_known_area_m2'
                        ).value
                    ),
                )
                if result is not None else False
            ),
            'rejected_near_clusters': (
                result.rejected_near_clusters if result is not None else 0
            ),
            'goals_sent': self._goals_sent,
            'goals_succeeded': self._goals_succeeded,
            'goals_failed': self._goals_failed,
            'blacklisted_goals': len(self._blacklist),
            'active_goal': (
                {
                    'x': round(candidate.x, 3),
                    'y': round(candidate.y, 3),
                    'yaw': round(candidate.yaw, 3),
                }
                if candidate is not None else None
            ),
            'distance_remaining': (
                round(self._last_remaining_distance, 3)
                if self._last_remaining_distance is not None else None
            ),
            'map_saved': self._map_saved,
            'saved_map_yaml': self._saved_map_yaml,
            'saved_map_image': self._saved_map_image,
        }

    def _publish_status(self):
        message = String()
        message.data = json.dumps(
            self._status_payload(),
            ensure_ascii=True,
            sort_keys=True,
        )
        self._status_publisher.publish(message)

    def _publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = str(
            self.get_parameter('global_frame').value
        )
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if self._latest_result is None or self._grid is None:
            self._marker_publisher.publish(markers)
            return

        points = Marker()
        points.header = clear.header
        points.ns = 'frontier_cells'
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = max(self._grid.resolution * 1.5, 0.06)
        points.scale.y = points.scale.x
        points.color.r = 0.10
        points.color.g = 0.65
        points.color.b = 1.0
        points.color.a = 0.9
        for cell in self._latest_result.frontier_cells:
            x, y = self._grid.cell_to_world(cell)
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.06
            points.points.append(point)
        markers.markers.append(points)

        for index, candidate in enumerate(
            self._latest_result.candidates[:20],
            start=1,
        ):
            goal = Marker()
            goal.header = clear.header
            goal.ns = 'candidate_goals'
            goal.id = index
            goal.type = Marker.ARROW
            goal.action = Marker.ADD
            goal.pose.position.x = candidate.x
            goal.pose.position.y = candidate.y
            goal.pose.position.z = 0.10
            goal.pose.orientation.z = math.sin(candidate.yaw * 0.5)
            goal.pose.orientation.w = math.cos(candidate.yaw * 0.5)
            goal.scale.x = 0.32 if index == 1 else 0.22
            goal.scale.y = 0.07
            goal.scale.z = 0.07
            goal.color.r = 0.15 if index == 1 else 0.95
            goal.color.g = 0.95 if index == 1 else 0.75
            goal.color.b = 0.20
            goal.color.a = 0.95
            markers.markers.append(goal)
        self._marker_publisher.publish(markers)

    def cancel_active_goal(self):
        """Request a safe Nav2 cancellation during process shutdown."""
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        node.get_logger().warning('Frontier exploration interrupted.')
        node.cancel_active_goal()
        return 130
    except Exception as error:
        node.get_logger().error(f'Frontier explorer failed: {error}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
