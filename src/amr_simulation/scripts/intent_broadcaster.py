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
from tf2_ros import Buffer, TransformListener, TransformException

class IntentBroadcaster(Node):
    """
    Heartbeat generator publishing the robot's future spatial occupancy
    to peer AMRs without an intermediary broker.
    """

    # Critical corridor reservation bounding box (from warehouse.sdf geometry)
    # choke_wall_top: center (0, 3.36), size 0.4x5.4  -> y spans [0.66, 6.06]
    # choke_wall_bottom: center (0, -3.36), size 0.4x5.4 -> y spans [-6.06, -0.66]
    # Gap: y in [-0.66, 0.66], wall thickness x: 0.2 on each side
    # Extended by ~0.8m safety buffer on x-axis, ~0.54m on y-axis
    CHOKE_X_MIN = -1.5
    CHOKE_X_MAX = 1.5
    CHOKE_Y_MIN = -1.2
    CHOKE_Y_MAX = 1.2

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_velocity = None
        self.planned_waypoints = []
        self.tf_ready = False
        
        # Start with a 1 Hz startup check that waits for TF readiness
        self.startup_timer = self.create_timer(1.0, self._wait_for_tf)
        self.broadcast_timer = None  # Created once TF is ready
        self.get_logger().info(f'[{self.robot_id}] Intent Broadcaster initializing, waiting for TF...')

    def _wait_for_tf(self):
        """Startup phase: wait for map -> base_footprint TF to become available."""
        try:
            self.tf_buffer.lookup_transform('map', f'{self.robot_id}/base_footprint', rclpy.time.Time())
            self.tf_ready = True
            self.startup_timer.cancel()
            # Now start the real 5 Hz heartbeat
            self.broadcast_timer = self.create_timer(0.2, self.broadcast_intent)
            self.get_logger().info(
                f'[{self.robot_id}] TF ready! Intent Broadcaster online, publishing to /fleet/intent at 5 Hz'
            )
        except TransformException:
            self.get_logger().info(
                f'[{self.robot_id}] Waiting for TF (map -> {self.robot_id}/base_footprint)...',
                throttle_duration_sec=5.0
            )

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist

    def plan_callback(self, msg):
        # Guard: during recovery behaviors the planner may clear the path
        if not msg.poses:
            self.planned_waypoints = []
            return
        # Downsample the dense global plan to 3 future waypoints
        step = max(1, len(msg.poses) // 10)
        self.planned_waypoints = [p.pose for p in msg.poses[step:step*4:step]][:3]

    def _is_in_choke_zone(self, x, y):
        """Check if robot's global position is inside the critical corridor bounding box."""
        return (self.CHOKE_X_MIN <= x <= self.CHOKE_X_MAX and
                self.CHOKE_Y_MIN <= y <= self.CHOKE_Y_MAX)

    def broadcast_intent(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', f'{self.robot_id}/base_footprint', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(
                f'[{self.robot_id}] TF lookup failed during broadcast: {e}',
                throttle_duration_sec=5.0
            )
            return
            
        msg = FleetIntent()
        msg.robot_id = self.robot_id
        msg.stamp = self.get_clock().now().to_msg()
        
        msg.current_pose.position.x = trans.transform.translation.x
        msg.current_pose.position.y = trans.transform.translation.y
        msg.current_pose.orientation = trans.transform.rotation
        
        if self.current_velocity is not None:
            msg.current_velocity = self.current_velocity
        msg.planned_waypoints = self.planned_waypoints
        msg.current_state = 0 # NORMAL_NAV
        msg.is_in_choke_zone = self._is_in_choke_zone(trans.transform.translation.x, trans.transform.translation.y)
        
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
