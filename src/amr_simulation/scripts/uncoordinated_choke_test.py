#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys
import time

class ChokeTest(Node):
    def __init__(self):
        super().__init__('choke_test')
        self.amr1_client = ActionClient(self, NavigateToPose, '/amr1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, '/amr2/navigate_to_pose')
        
    def wait_for_servers(self):
        self.get_logger().info('Waiting for action servers...')
        self.amr1_client.wait_for_server()
        self.amr2_client.wait_for_server()
        self.get_logger().info('Action servers available.')
        
    def send_goals(self):
        # AMR 1 goal: from x=-5 to x=4 through choke at x=0
        goal_msg1 = NavigateToPose.Goal()
        goal_msg1.pose.header.frame_id = 'map'
        goal_msg1.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg1.pose.pose.position.x = 4.0
        goal_msg1.pose.pose.position.y = 0.0
        goal_msg1.pose.pose.orientation.w = 1.0

        # AMR 2 goal: from x=5 to x=-4 through choke at x=0
        goal_msg2 = NavigateToPose.Goal()
        goal_msg2.pose.header.frame_id = 'map'
        goal_msg2.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg2.pose.pose.position.x = -4.0
        goal_msg2.pose.pose.position.y = 0.0
        # point amr2 to the west
        goal_msg2.pose.pose.orientation.z = 1.0
        goal_msg2.pose.pose.orientation.w = 0.0

        self.get_logger().info('Sending goal to AMR 1 (target: 4.0, 0.0)...')
        self.amr1_client.send_goal_async(goal_msg1)
        
        self.get_logger().info('Sending goal to AMR 2 (target: -4.0, 0.0)...')
        self.amr2_client.send_goal_async(goal_msg2)

def main():
    rclpy.init()
    node = ChokeTest()
    node.wait_for_servers()
    
    # Wait a bit for simulation to settle
    time.sleep(2)
    node.send_goals()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
