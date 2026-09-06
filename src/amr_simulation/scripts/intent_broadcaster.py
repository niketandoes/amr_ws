#!/usr/bin/env python3
"""
P2P Fleet Intent Broadcaster Node
Extracts local robot odometry and downsamples Nav2 global path waypoints,
publishing a continuous 5 Hz heartbeat of robot state and trajectory intent
onto /fleet/intent via DDS multicast with BEST_EFFORT QoS.

Inputs:
  - odom (nav_msgs/msg/Odometry)
  - plan (nav_msgs/msg/Path)

Outputs:
  - /fleet/intent (amr_interfaces/msg/FleetIntent)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from amr_interfaces.msg import FleetIntent

class IntentBroadcaster(Node):
    """
    Heartbeat generator publishing the robot's future spatial occupancy
    to peer AMRs without an intermediary broker.
    """
    def __init__(self):
        super().__init__('intent_broadcaster')
        
        # Resolve namespace to get robot_id (e.g., 'amr1')
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'default_amr'

        # P2P QoS: Best Effort prevents network locking if packets drop
        p2p_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.publisher = self.create_publisher(FleetIntent, '/fleet/intent', p2p_qos)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(Path, 'plan', self.plan_callback, 10)

        self.current_pose = None
        self.current_velocity = None
        self.planned_waypoints = []
        
        # 5 Hz Heartbeat
        self.timer = self.create_timer(0.2, self.broadcast_intent)
        self.get_logger().info(f'[{self.robot_id}] Intent Broadcaster online, publishing to /fleet/intent')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def plan_callback(self, msg):
        # Downsample the dense global plan to 3 future waypoints
        step = max(1, len(msg.poses) // 10)
        self.planned_waypoints = [p.pose for p in msg.poses[step:step*4:step]][:3]

    def broadcast_intent(self):
        if not self.current_pose:
            return
            
        msg = FleetIntent()
        msg.robot_id = self.robot_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.current_pose = self.current_pose
        if self.current_velocity is not None:
            msg.current_velocity = self.current_velocity
        msg.planned_waypoints = self.planned_waypoints
        msg.current_state = 0 # NORMAL_NAV
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = IntentBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
