#!/usr/bin/env python3
"""
Single-Robot Choke Point Traversal Client
Sends a programmatic NavigateToPose action goal across the central 1.15m choke point
to verify single-robot unattended navigation and path following.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class ChokePointNavigator(Node):
    """
    Action client wrapper for dispatching and monitoring NavigateToPose goals.
    """
    def __init__(self):
        super().__init__(
            'choke_point_navigator',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x=4.0, y=0.0):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/navigate_to_pose action server not available after 10s!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Dispatching goal: ({x}, {y}) through the 1.15m bottleneck...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2!')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted by Nav2! Tracking navigation progress...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m | '
            f'ETA: {feedback.estimated_time_remaining.sec}s'
        )

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal SUCCEEDED! AMR successfully cleared the bottleneck!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('Goal was CANCELED!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal was ABORTED!')
        else:
            self.get_logger().info(f'Goal finished with status code: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = ChokePointNavigator()
    navigator.send_goal(4.0, 0.0)
    try:
        rclpy.spin(navigator)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
