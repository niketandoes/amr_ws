#!/usr/bin/env python3
"""
Coordinated Choke Point Navigation Test
Dispatches opposing goals to AMR 1 and AMR 2 across the 1.15m narrow corridor
while P2P intent sharing and decentralized coordination are active. Demonstrates
deadlock-free traversal via spatial conflict arbitration.

Target Goals:
  - amr1: (4.0, -1.5)  [well clear of choke zone exit at x=1.5m]
  - amr2: (-4.0, 1.5)   [well clear of choke zone exit at x=-1.5m]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.time import Time
import sys
import time

class CoordinatedChokeTest(Node):
    """
    Action client node that issues simultaneous opposing navigation goals
    to evaluate decentralized collision avoidance and right-of-way yielding.
    """
    def __init__(self):
        super().__init__(
            'coordinated_choke_test',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.amr1_client = ActionClient(self, NavigateToPose, '/amr1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, '/amr2/navigate_to_pose')
        
    def wait_for_servers(self, timeout_sec=60.0):
        self.get_logger().info('Waiting for /amr1 and /amr2 action servers...')
        s1 = self.amr1_client.wait_for_server(timeout_sec=timeout_sec)
        s2 = self.amr2_client.wait_for_server(timeout_sec=timeout_sec)
        if not s1 or not s2:
            self.get_logger().error('Action servers failed to become available within timeout.')
            return False
        self.get_logger().info('Action servers are ONLINE. Waiting 5s for Nav2 lifecycle activation...')
        import time
        time.sleep(5.0)
        self.get_logger().info('Nav2 stack fully active. Ready.')
        return True
        
    def dispatch_head_to_head_goals(self):
        goal1 = NavigateToPose.Goal()
        goal1.pose.header.frame_id = 'map'
        goal1.pose.header.stamp = Time().to_msg()
        goal1.pose.pose.position.x = 4.0
        goal1.pose.pose.position.y = -1.5
        goal1.pose.pose.orientation.w = 1.0

        goal2 = NavigateToPose.Goal()
        goal2.pose.header.frame_id = 'map'
        goal2.pose.header.stamp = Time().to_msg()
        goal2.pose.pose.position.x = -4.0
        goal2.pose.pose.position.y = 1.5
        goal2.pose.pose.orientation.z = 1.0
        goal2.pose.pose.orientation.w = 0.0

        self.get_logger().info('[COORDINATED DISPATCH] Dispatching opposing goals with P2P intent sharing active...')
        f1 = self.amr1_client.send_goal_async(goal1)
        f1.add_done_callback(lambda future: self.goal_response_callback(future, 'amr1'))

        f2 = self.amr2_client.send_goal_async(goal2)
        f2.add_done_callback(lambda future: self.goal_response_callback(future, 'amr2'))

    def goal_response_callback(self, future, robot_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'[{robot_id}] Goal REJECTED by Nav2 stack.')
            return
        self.get_logger().info(f'[{robot_id}] Goal ACCEPTED. Monitoring P2P bottleneck traversal...')
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(lambda f: self.goal_result_callback(f, robot_id))

    def goal_result_callback(self, future, robot_id):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{robot_id}] Mission SUCCEEDED! Traversed choke point successfully.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f'[{robot_id}] Mission CANCELED.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'[{robot_id}] Mission ABORTED!')
        else:
            self.get_logger().info(f'[{robot_id}] Goal finished with status code: {status}')

def main():
    rclpy.init()
    node = CoordinatedChokeTest()
    if not node.wait_for_servers():
        sys.exit(1)
        
    time.sleep(2.0)
    node.dispatch_head_to_head_goals()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
