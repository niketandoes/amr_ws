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
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster
import time


class IntentBroadcaster(Node):
    """
    Heartbeat generator publishing the robot's future spatial occupancy
    to peer AMRs without an intermediary broker.
    """

    CHOKE_X_MIN = -1.0
    CHOKE_X_MAX = 1.0
    CHOKE_Y_MIN = -0.80
    CHOKE_Y_MAX = 0.80

    APPROACH_X_LIMIT = 4.0
    APPROACH_Y_LIMIT = 2.5

    def __init__(self):
        super().__init__('intent_broadcaster')

        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'default_amr'

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
        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_velocity = None
        self.planned_waypoints = []
        self.tf_ready = False

        self._approach_entry_time = 0.0
        self.last_halt_time = 0.0
        self.cmd_vel_coord_sub = self.create_subscription(Twist, 'cmd_vel_coord', self.cmd_vel_coord_callback, 10)

        self.startup_timer = self.create_timer(1.0, self._wait_for_tf)
        self.broadcast_timer = None
        self.get_logger().info(f'[{self.robot_id}] Intent Broadcaster initializing, waiting for TF...')

    def _wait_for_tf(self):
        try:
            self.tf_buffer.lookup_transform('map', f'{self.robot_id}/base_footprint', rclpy.time.Time())
            self.tf_ready = True
            self.startup_timer.cancel()
            self.broadcast_timer = self.create_timer(0.2, self.broadcast_intent)
            self.get_logger().info(
                f'[{self.robot_id}] TF ready! Intent Broadcaster online, publishing to /fleet/intent at 5 Hz'
            )
        except TransformException:
            self.get_logger().info(
                f'[{self.robot_id}] Waiting for TF (map -> {self.robot_id}/base_footprint)...',
                throttle_duration_sec=5.0
            )

    def cmd_vel_coord_callback(self, msg):
        self.last_halt_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist
        
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else f'{self.robot_id}/odom'
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else f'{self.robot_id}/base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def plan_callback(self, msg):
        step = max(1, len(msg.poses) // 10)
        self.planned_waypoints = [p.pose for p in msg.poses[step:step * 4:step]][:3]

    def _is_in_choke_zone(self, x, y):
        return (self.CHOKE_X_MIN <= x <= self.CHOKE_X_MAX and
                self.CHOKE_Y_MIN <= y <= self.CHOKE_Y_MAX)

    def _is_in_approach_zone(self, x, y):
        return abs(x) <= self.APPROACH_X_LIMIT and abs(y) <= self.APPROACH_Y_LIMIT

    def broadcast_intent(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', f'{self.robot_id}/base_footprint', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(
                f'[{self.robot_id}] TF lookup failed during broadcast: {e}',
                throttle_duration_sec=5.0
            )
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        in_choke = self._is_in_choke_zone(x, y)
        in_approach = self._is_in_approach_zone(x, y)

        now = self.get_clock().now().nanoseconds / 1e9

        # Track first-entry timestamp into the approach zone for fair
        # request-order tie-breaking (reset once we leave the zone).
        if in_approach and self._approach_entry_time == 0.0:
            self._approach_entry_time = now
        elif not in_approach:
            self._approach_entry_time = 0.0

        msg = FleetIntent()
        msg.robot_id = self.robot_id
        msg.stamp = self.get_clock().now().to_msg()

        msg.current_pose.position.x = x
        msg.current_pose.position.y = y
        msg.current_pose.orientation = trans.transform.rotation

        if self.current_velocity is not None:
            msg.current_velocity = self.current_velocity
        msg.planned_waypoints = self.planned_waypoints

        is_yielding = (now - self.last_halt_time) < 0.5

        if is_yielding:
            msg.current_state = 3   # YIELDING
        elif in_choke:
            msg.current_state = 2   # IN_CHOKE
        elif in_approach:
            msg.current_state = 1   # APPROACHING_CHOKE
        else:
            msg.current_state = 0   # NORMAL_NAV

        msg.is_in_choke_zone = in_choke
        msg.approach_request_time = self._approach_entry_time

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = IntentBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

