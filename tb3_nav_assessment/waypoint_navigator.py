import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )
        self._last_feedback_second = -1

        self._startup_timer = self.create_timer(
            1.0,
            self.start_navigation,
        )

    def start_navigation(self):
        if self.get_clock().now().nanoseconds == 0:
            self.get_logger().info(
                'Waiting for Gazebo simulation clock...'
            )
            return

        self._startup_timer.cancel()
        self.send_goal()

    def send_goal(self):
        x = float(self.get_parameter('goal_x').value)
        y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)

        self.get_logger().info(
            'Waiting for /navigate_to_pose action server...'
        )

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server is not ready yet...')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal: x={x:.2f}, y={y:.2f}, '
            f'yaw={yaw:.2f} rad'
        )

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal was ACCEPTED')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        elapsed = feedback.navigation_time.sec

        if elapsed == self._last_feedback_second:
            return

        self._last_feedback_second = elapsed

        eta = (
            feedback.estimated_time_remaining.sec
            + feedback.estimated_time_remaining.nanosec / 1e9
        )

        self.get_logger().info(
            f'Feedback: distance={feedback.distance_remaining:.2f} m, '
            f'ETA={eta:.1f} s, elapsed={elapsed} s, '
            f'recoveries={feedback.number_of_recoveries}'
        )

    def result_callback(self, future):
        wrapped_result = future.result()
        status = wrapped_result.status
        status_name = STATUS_NAMES.get(status, f'CODE_{status}')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation result: SUCCEEDED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('Navigation result: CANCELED')
        else:
            self.get_logger().error(
                f'Navigation result: {status_name}'
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning('Navigation interrupted by user')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
