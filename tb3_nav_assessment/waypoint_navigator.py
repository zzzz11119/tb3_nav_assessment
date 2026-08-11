import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoint_names', ['point_a'])
        self.declare_parameter('waypoint_x', [0.0])
        self.declare_parameter('waypoint_y', [0.0])
        self.declare_parameter('waypoint_yaw', [0.0])
        self.declare_parameter('continue_on_failure', False)

        self.waypoint_names = list(
            self.get_parameter('waypoint_names').value
        )
        self.waypoint_x = list(
            self.get_parameter('waypoint_x').value
        )
        self.waypoint_y = list(
            self.get_parameter('waypoint_y').value
        )
        self.waypoint_yaw = list(
            self.get_parameter('waypoint_yaw').value
        )
        self.continue_on_failure = bool(
            self.get_parameter('continue_on_failure').value
        )

        lengths = {
            len(self.waypoint_names),
            len(self.waypoint_x),
            len(self.waypoint_y),
            len(self.waypoint_yaw),
        }

        if len(lengths) != 1:
            raise RuntimeError(
                'Waypoint name, x, y and yaw lists must have equal lengths.'
            )

        if len(self.waypoint_names) < 3:
            raise RuntimeError(
                'At least three waypoints are required.'
            )

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )

        self.current_index = 0
        self.success_count = 0
        self.failure_count = 0
        self.last_feedback_time_ns = 0
        self.shutdown_timer = None

        self.startup_timer = self.create_timer(
            1.0,
            self.start_navigation,
        )

        self.get_logger().info(
            f'Loaded {len(self.waypoint_names)} waypoints.'
        )

    def start_navigation(self):
        if self.get_clock().now().nanoseconds == 0:
            self.get_logger().info(
                'Waiting for Gazebo simulation clock...'
            )
            return

        if not self.action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().info(
                'Waiting for /navigate_to_pose action server...'
            )
            return

        self.startup_timer.cancel()
        self.send_current_goal()

    def send_current_goal(self):
        if self.current_index >= len(self.waypoint_names):
            self.finish_navigation()
            return

        name = self.waypoint_names[self.current_index]
        x = float(self.waypoint_x[self.current_index])
        y = float(self.waypoint_y[self.current_index])
        yaw = float(self.waypoint_yaw[self.current_index])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        number = self.current_index + 1
        total = len(self.waypoint_names)

        self.get_logger().info(
            f'Sending waypoint {number}/{total}: '
            f'{name}, x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        self.last_feedback_time_ns = 0

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Waypoint {self.current_name()} was REJECTED.'
            )
            self.handle_failure('REJECTED')
            return

        self.get_logger().info(
            f'Waypoint {self.current_name()} was ACCEPTED.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_feedback_time_ns < 1_000_000_000:
            return

        self.last_feedback_time_ns = now_ns
        feedback = feedback_message.feedback

        eta = (
            feedback.estimated_time_remaining.sec
            + feedback.estimated_time_remaining.nanosec / 1e9
        )
        elapsed = (
            feedback.navigation_time.sec
            + feedback.navigation_time.nanosec / 1e9
        )

        self.get_logger().info(
            f'Navigating to {self.current_name()}: '
            f'distance={feedback.distance_remaining:.2f} m, '
            f'ETA={eta:.1f} s, elapsed={elapsed:.1f} s, '
            f'recoveries={feedback.number_of_recoveries}'
        )

    def result_callback(self, future):
        response = future.result()
        status = response.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.success_count += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_name()} successfully.'
            )
            self.current_index += 1
            self.send_current_goal()
            return

        status_name = self.status_to_text(status)
        self.get_logger().error(
            f'Waypoint {self.current_name()} finished with '
            f'status {status_name}.'
        )
        self.handle_failure(status_name)

    def handle_failure(self, status_name):
        self.failure_count += 1

        if self.continue_on_failure:
            self.get_logger().warning(
                f'Continuing after {status_name}.'
            )
            self.current_index += 1
            self.send_current_goal()
            return

        self.get_logger().error(
            'Stopping waypoint sequence because '
            'continue_on_failure is false.'
        )
        self.schedule_shutdown()

    def finish_navigation(self):
        total = len(self.waypoint_names)

        self.get_logger().info(
            'Waypoint navigation complete: '
            f'{self.success_count}/{total} succeeded, '
            f'{self.failure_count} failed.'
        )
        self.schedule_shutdown()

    def schedule_shutdown(self):
        if self.shutdown_timer is None:
            self.shutdown_timer = self.create_timer(
                0.2,
                self.shutdown,
            )

    def shutdown(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()

        if rclpy.ok():
            rclpy.shutdown()

    def current_name(self):
        if self.current_index < len(self.waypoint_names):
            return self.waypoint_names[self.current_index]

        return 'unknown'

    @staticmethod
    def status_to_text(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'CODE_{status}')


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = WaypointNavigator()
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
