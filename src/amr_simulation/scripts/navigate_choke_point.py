#!/usr/bin/env python3
"""
Single-Robot Choke Point Traversal Client
------------------------------------------
Sends a programmatic NavigateToPose action goal across the central 1.15m choke point
to verify single-robot unattended navigation and path following.

Workflow Overview:
1. Initializes a ROS 2 node ('choke_point_navigator') using simulation time.
2. Connects to the Nav2 action server ('/amr1/navigate_to_pose').
3. Constructs and dispatches a target PoseStamped goal in the 'map' frame across the choke point.
4. Listens asynchronously for goal acceptance, progress feedback (distance/ETA), and final result.
5. Shuts down ROS 2 execution once navigation succeeds or fails.

Note on Route Calculation:
- Route calculation (global path planning and local trajectory control) is delegated to Nav2.
- The script itself does not calculate trajectories; it acts purely as a ROS 2 Action Client.
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
            # Enable simulation time to sync with Gazebo/clock publisher
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )
        # Create an Action Client targeting Nav2's NavigateToPose interface under namespace /amr1
        self._action_client = ActionClient(self, NavigateToPose, '/amr1/navigate_to_pose')

    def send_goal(self, x=4.0, y=0.0):
        """
        Defines the target goal pose and dispatches it asynchronously to Nav2.
        
        Goal Definition:
        - Position: (x=4.0, y=0.0) relative to 'map' frame (across the 1.15m bottleneck).
        - Orientation: Quaternion (0, 0, 0, 1) representing 0-degree yaw heading.
        """
        self.get_logger().info('Waiting for /amr1/navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/amr1/navigate_to_pose action server not available after 10s!')
            return

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Dispatching goal: ({x}, {y}) through the 1.15m bottleneck...')
        # Send goal asynchronously with feedback callback registered
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback triggered when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2!')
            if rclpy.ok():
                rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted by Nav2! Tracking navigation progress...')
        # Register callback for ultimate navigation result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback invoked periodically by Nav2 while robot is en route."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m | '
            f'ETA: {feedback.estimated_time_remaining.sec}s'
        )

    def get_result_callback(self, future):
        """Callback triggered when navigation completes (Success, Aborted, Canceled)."""
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
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = ChokePointNavigator()
    # Define choke point traversal target pose (x=4.0, y=0.0)
    navigator.send_goal(4.0, 0.0)
    try:
        # Spin event loop to handle feedback and result callbacks asynchronously
        rclpy.spin(navigator)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()

