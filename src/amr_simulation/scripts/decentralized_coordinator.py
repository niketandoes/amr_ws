#!/usr/bin/env python3
"""
Decentralized Fleet Coordinator Node
Implements peer-to-peer (P2P) right-of-way arbitration without central dispatchers.
Monitors distances to the narrow 1.15m bottleneck and commands yielding AMRs
to hold position when approaching lower-ID / higher-priority peers.

Inputs:
  - /fleet/intent (amr_interfaces/msg/FleetIntent)

Outputs:
  - cmd_vel (geometry_msgs/msg/TwistStamped) [Override during yielding]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from amr_interfaces.msg import FleetIntent
import math

class DecentralizedCoordinator(Node):
    """
    Evaluates peer trajectories and bottleneck reservations at 50 Hz.
    If multiple AMRs are within the choke buffer zone, the agent with
    higher string ID yields until the corridor is cleared.
    """
    def __init__(self):
        super().__init__('decentralized_coordinator')
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'default_amr'
        
        p2p_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.intent_sub = self.create_subscription(FleetIntent, '/fleet/intent', self.intent_callback, p2p_qos)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        self.peer_intents = {}
        self.my_intent = None
        self.choke_center = (0.0, 0.0)
        self.choke_radius_enter = 3.5
        self.choke_radius_exit = 3.5
        self.yielding_to = None
        
        self.create_timer(0.02, self.evaluate_conflicts)
        self.get_logger().info(f'[{self.robot_id}] Decentralized Coordinator online (P2P Mesh Arbitration active)')

    def intent_callback(self, msg):
        if msg.robot_id == self.robot_id:
            self.my_intent = msg
        else:
            self.peer_intents[msg.robot_id] = msg

    def evaluate_conflicts(self):
        if not self.my_intent:
            return
            
        my_x = self.my_intent.current_pose.position.x
        my_y = self.my_intent.current_pose.position.y
        my_dist = math.hypot(my_x - self.choke_center[0], my_y - self.choke_center[1])
        
        # Check if we are currently yielding to someone
        if self.yielding_to:
            if self.yielding_to in self.peer_intents:
                peer_intent = self.peer_intents[self.yielding_to]
                peer_x = peer_intent.current_pose.position.x
                peer_y = peer_intent.current_pose.position.y
                peer_dist = math.hypot(peer_x - self.choke_center[0], peer_y - self.choke_center[1])
                
                # Wait until they are completely clear of the exit radius
                if peer_dist > self.choke_radius_exit:
                    self.get_logger().info(f'[{self.robot_id}] {self.yielding_to} has cleared the zone. Resuming.')
                    self.yielding_to = None
                else:
                    self._halt(self.yielding_to)
                    return
            else:
                self.yielding_to = None # Peer disappeared, assume clear
                
        # If we are not yielding, check if we need to start yielding
        if my_dist > self.choke_radius_enter:
            return # Too far to initiate conflict
            
        # Check peers in the enter zone
        conflicting_peers = []
        for peer_id, intent in self.peer_intents.items():
            peer_x = intent.current_pose.position.x
            peer_y = intent.current_pose.position.y
            peer_dist = math.hypot(peer_x - self.choke_center[0], peer_y - self.choke_center[1])
            
            # If peer is anywhere in the relevant simulation area (buffer distance)
            if peer_dist <= 6.0:
                conflicting_peers.append((peer_id, peer_dist))
                
        if not conflicting_peers:
            return # Clear corridor, proceed normally
            
        # Deterministic arbitration: lowest string ID gets right-of-way
        all_candidates = conflicting_peers + [(self.robot_id, my_dist)]
        all_candidates.sort(key=lambda x: x[0]) 
        
        winner_id = all_candidates[0][0]
        
        if self.robot_id != winner_id:
            self.yielding_to = winner_id
            self._halt(winner_id)

    def _halt(self, winner_id):
        self.get_logger().info(f'[{self.robot_id}] Yielding to {winner_id} at choke point.', throttle_duration_sec=1.5)
        halt_msg = TwistStamped()
        halt_msg.header.stamp = self.get_clock().now().to_msg()
        halt_msg.header.frame_id = f'{self.robot_id}/base_footprint'
        self.cmd_vel_pub.publish(halt_msg)

def main():
    rclpy.init()
    node = DecentralizedCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
