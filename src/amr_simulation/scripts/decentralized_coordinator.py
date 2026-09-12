#!/usr/bin/env python3
"""
Decentralized Fleet Coordinator Node — Zone Reservation Protocol
Implements peer-to-peer (P2P) right-of-way arbitration without central dispatchers.

Protocol:
  1. A bounding-box defines the critical corridor zone (from warehouse.sdf geometry).
  2. Each robot broadcasts ground-truth zone occupancy via FleetIntent.is_in_choke_zone.
  3. Occupancy ALWAYS gates entry: if any peer is inside the zone (or clearing with
     active hysteresis), all approaching peers yield — regardless of string-ID priority.
  4. String-ID comparison is used ONLY as a tie-breaker when multiple robots are
     waiting in the approach zone with an empty critical corridor.
  5. Exit hysteresis requires a peer to report is_in_choke_zone=False for N
     consecutive ticks (~300ms) before the corridor lock is released.
  6. Stale peer intents (>1.5s without update) are pruned to prevent ghost lockups.

Velocity Architecture:
  - Nav2 controller_server publishes to cmd_vel_nav (priority 10)
  - This coordinator publishes halt commands to cmd_vel_coord (priority 20)
  - twist_mux merges both onto cmd_vel for the Gazebo bridge
  - On yield: cancel Nav2 goal + publish zero-twist to cmd_vel_coord
  - On resume: re-dispatch cached goal to Nav2

Inputs:
  - /fleet/intent (amr_interfaces/msg/FleetIntent)
  - navigate_to_pose action feedback (for goal caching)

Outputs:
  - cmd_vel_coord (geometry_msgs/msg/TwistStamped) [Override during yielding via twist_mux]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from amr_interfaces.msg import FleetIntent
import math
import time

class DecentralizedCoordinator(Node):
    """
    Evaluates peer zone occupancy and bottleneck reservations at 50 Hz.
    Uses bounding-box zone reservation with exit hysteresis for robust
    collision-free choke point traversal.
    """

    # Critical corridor reservation bounding box (matches intent_broadcaster)
    CHOKE_X_MIN = -1.5
    CHOKE_X_MAX = 1.5
    CHOKE_Y_MIN = -1.2
    CHOKE_Y_MAX = 1.2

    # Approach zone: robots within BOTH x and y ranges are candidates for conflict checks
    APPROACH_X_LIMIT = 3.5
    APPROACH_Y_LIMIT = 1.5

    # Exit hysteresis: number of consecutive ticks peer must report zone-free
    # At 10 Hz evaluation, 5 ticks ≈ 500ms
    HYSTERESIS_TICKS_REQUIRED = 5

    # Stale intent expiry: prune peer data older than this (seconds)
    INTENT_EXPIRY_SEC = 1.5

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

        # Publish halt commands to cmd_vel_coord (twist_mux priority 20 overrides Nav2)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel_coord', 10)
        
        # Nav2 action client for clean goal cancel/re-dispatch during yielding
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cached_goal = None       # NavigateToPose.Goal cached for re-dispatch
        self._active_goal_handle = None  # Current goal handle for cancellation
        self._is_nav_paused = False     # True when we have cancelled Nav2's goal

        self.peer_intents = {}       # {peer_id: FleetIntent msg}
        self.peer_timestamps = {}    # {peer_id: time.monotonic() of last received msg}
        self.my_intent = None

        # Track which peer we are yielding to (or None)
        self.yielding_to = None

        # Hysteresis counters: how many consecutive ticks each peer has been OUT of zone
        self.peer_clear_ticks = {}
        
        self.create_timer(0.1, self.evaluate_conflicts)  # 10 Hz (reduces cmd_vel jitter vs Nav2 20 Hz)
        self.get_logger().info(
            f'[{self.robot_id}] Decentralized Coordinator online '
            f'(Zone Reservation Protocol — box [{self.CHOKE_X_MIN},{self.CHOKE_X_MAX}] x '
            f'[{self.CHOKE_Y_MIN},{self.CHOKE_Y_MAX}], '
            f'hysteresis={self.HYSTERESIS_TICKS_REQUIRED} ticks, '
            f'twist_mux output on cmd_vel_coord)'
        )

    def intent_callback(self, msg):
        if msg.robot_id == self.robot_id:
            self.my_intent = msg
        else:
            self.peer_intents[msg.robot_id] = msg
            self.peer_timestamps[msg.robot_id] = time.monotonic()

    def _self_in_approach_zone(self):
        """Check if this robot is within the approach zone (both X and Y limits)."""
        if not self.my_intent:
            return False
        x = self.my_intent.current_pose.position.x
        y = self.my_intent.current_pose.position.y
        return abs(x) <= self.APPROACH_X_LIMIT and abs(y) <= self.APPROACH_Y_LIMIT

    def _self_in_choke_zone(self):
        """Check if this robot's odometry is inside the critical corridor box."""
        if not self.my_intent:
            return False
        x = self.my_intent.current_pose.position.x
        y = self.my_intent.current_pose.position.y
        return (self.CHOKE_X_MIN <= x <= self.CHOKE_X_MAX and
                self.CHOKE_Y_MIN <= y <= self.CHOKE_Y_MAX)

    def _peer_confirmed_clear(self, peer_id):
        """Return True only if the peer has been continuously out-of-zone for
        the required hysteresis duration."""
        return self.peer_clear_ticks.get(peer_id, 0) >= self.HYSTERESIS_TICKS_REQUIRED

    def _prune_stale_intents(self):
        """Remove peer intents that haven't been refreshed within INTENT_EXPIRY_SEC.
        Prevents ghost robot lockup when a peer crashes inside the choke zone."""
        now = time.monotonic()
        stale_peers = [
            pid for pid, ts in self.peer_timestamps.items()
            if (now - ts) > self.INTENT_EXPIRY_SEC
        ]
        for pid in stale_peers:
            self.get_logger().warn(
                f'[{self.robot_id}] Pruning stale intent from {pid} '
                f'(no update for >{self.INTENT_EXPIRY_SEC}s). Assuming clear.',
                throttle_duration_sec=5.0
            )
            self.peer_intents.pop(pid, None)
            self.peer_timestamps.pop(pid, None)
            self.peer_clear_ticks.pop(pid, None)
            # If we were yielding to this stale peer, resume immediately
            if self.yielding_to == pid:
                self.yielding_to = None
                self._resume_navigation()

    def evaluate_conflicts(self):
        if not self.my_intent:
            return

        # ── Prune stale peer data to prevent ghost lockups ──
        self._prune_stale_intents()

        # ── Update hysteresis counters for all peers ──
        for peer_id, intent in self.peer_intents.items():
            if intent.is_in_choke_zone:
                self.peer_clear_ticks[peer_id] = 0
            else:
                self.peer_clear_ticks[peer_id] = self.peer_clear_ticks.get(peer_id, 0) + 1

        # ── If currently yielding, check if we can resume ──
        if self.yielding_to:
            if self.yielding_to in self.peer_intents:
                peer = self.peer_intents[self.yielding_to]
                # Require BOTH: peer reports out-of-zone AND hysteresis is satisfied
                if not peer.is_in_choke_zone and self._peer_confirmed_clear(self.yielding_to):
                    self.get_logger().info(
                        f'[{self.robot_id}] {self.yielding_to} has confirmed clear of zone '
                        f'(hysteresis satisfied). Resuming navigation.'
                    )
                    self.yielding_to = None
                    self._resume_navigation()
                else:
                    self._halt(self.yielding_to)
                    return
            else:
                # Peer disappeared from mesh — assume clear
                self.get_logger().info(
                    f'[{self.robot_id}] {self.yielding_to} disappeared from mesh. Resuming.'
                )
                self.yielding_to = None
                self._resume_navigation()

        # ── If we are not in the approach zone, nothing to do ──
        if not self._self_in_approach_zone():
            return

        i_am_in_zone = self._self_in_choke_zone()

        # ── RULE 1: Occupancy gate — if any peer is inside the zone (or still in
        # hysteresis window), we must yield regardless of ID priority ──
        for peer_id, intent in self.peer_intents.items():
            peer_in_zone = intent.is_in_choke_zone
            peer_clearing = not peer_in_zone and not self._peer_confirmed_clear(peer_id)

            if peer_in_zone or peer_clearing:
                # A peer physically occupies (or is still clearing) the corridor
                if not i_am_in_zone:
                    # We are approaching but not inside — yield unconditionally
                    self.yielding_to = peer_id
                    self._halt(peer_id)
                    return
                # If WE are also in the zone, we don't halt ourselves (we need to
                # exit). The peer's coordinator will handle its own yielding logic.

        # ── RULE 2: Tie-break — both robots are in approach zone, corridor is empty.
        # Use deterministic string-ID comparison: lowest ID proceeds first. ──
        approaching_peers = []
        for peer_id, intent in self.peer_intents.items():
            peer_x = intent.current_pose.position.x
            peer_y = intent.current_pose.position.y
            if abs(peer_x) <= self.APPROACH_X_LIMIT and abs(peer_y) <= self.APPROACH_Y_LIMIT:
                approaching_peers.append(peer_id)

        if not approaching_peers:
            return  # No peers in approach zone — proceed freely

        # Deterministic arbitration: lowest string ID gets right-of-way
        all_candidates = approaching_peers + [self.robot_id]
        all_candidates.sort()
        winner_id = all_candidates[0]

        if self.robot_id != winner_id:
            self.yielding_to = winner_id
            self._halt(winner_id)

    def _halt(self, winner_id):
        """Halt the robot by publishing zero-twist to cmd_vel_coord (twist_mux overrides
        Nav2's cmd_vel_nav) and cancelling the active Nav2 navigation goal."""
        self.get_logger().info(
            f'[{self.robot_id}] Yielding to {winner_id} at choke zone.',
            throttle_duration_sec=1.5
        )
        # Immediate velocity suppression via twist_mux priority override
        halt_msg = TwistStamped()
        halt_msg.header.stamp = self.get_clock().now().to_msg()
        halt_msg.header.frame_id = f'{self.robot_id}/base_footprint'
        self.cmd_vel_pub.publish(halt_msg)

        # Cancel Nav2 goal to prevent progress checker timeout and recovery triggers
        self._cancel_navigation()

    def _cancel_navigation(self):
        """Cancel the active Nav2 NavigateToPose goal and cache it for re-dispatch.
        Prevents Nav2's progress checker from timing out during the yield hold."""
        if self._is_nav_paused:
            return  # Already paused, no-op

        if not self._nav_action_client.server_is_ready():
            self.get_logger().warn(
                f'[{self.robot_id}] NavigateToPose action server not ready, '
                f'cannot cancel goal. twist_mux halt still active.',
                throttle_duration_sec=5.0
            )
            self._is_nav_paused = True
            return

        # Cancel the tracked goal handle if we have one from a re-dispatch
        if self._active_goal_handle is not None:
            self.get_logger().info(
                f'[{self.robot_id}] Cancelling tracked Nav2 goal for yield hold.',
                throttle_duration_sec=2.0
            )
            self._active_goal_handle.cancel_goal_async()
        else:
            # No tracked handle — the goal was dispatched externally (e.g., by task_allocator_cnp).
            # twist_mux zero-twist override is still effective via cmd_vel_coord priority.
            # The external dispatcher retains its own goal handle.
            self.get_logger().info(
                f'[{self.robot_id}] No tracked goal handle to cancel. '
                f'twist_mux halt active on cmd_vel_coord.',
                throttle_duration_sec=2.0
            )
        self._is_nav_paused = True

    def _resume_navigation(self):
        """Re-dispatch the cached NavigateToPose goal after the corridor clears.
        If no cached goal exists, Nav2 will simply remain idle until the next
        external goal dispatch (e.g., from task_allocator_cnp)."""
        if not self._is_nav_paused:
            return  # Not paused, no-op

        self._is_nav_paused = False

        if self._cached_goal is None:
            self.get_logger().info(
                f'[{self.robot_id}] Nav2 resumed (no cached goal to re-dispatch, '
                f'awaiting next goal from task allocator).',
                throttle_duration_sec=2.0
            )
            return

        if not self._nav_action_client.server_is_ready():
            self.get_logger().warn(
                f'[{self.robot_id}] NavigateToPose action server not ready, '
                f'cannot re-dispatch cached goal.',
                throttle_duration_sec=5.0
            )
            return

        self.get_logger().info(
            f'[{self.robot_id}] Corridor clear — re-dispatching cached Nav2 goal '
            f'({self._cached_goal.pose.pose.position.x:.2f}, '
            f'{self._cached_goal.pose.pose.position.y:.2f}).'
        )
        send_future = self._nav_action_client.send_goal_async(
            self._cached_goal,
            feedback_callback=self._nav_feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Track the goal handle from re-dispatched goals for future cancellation."""
        goal_handle = future.result()
        if goal_handle is not None and goal_handle.accepted:
            self._active_goal_handle = goal_handle
            self.get_logger().info(
                f'[{self.robot_id}] Re-dispatched Nav2 goal accepted.',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().warn(
                f'[{self.robot_id}] Re-dispatched Nav2 goal was rejected!',
                throttle_duration_sec=2.0
            )

    def _nav_feedback_callback(self, feedback_msg):
        """Feedback callback for re-dispatched goals. Goal is already cached."""
        pass  # Goal is already cached from the original dispatch

    def cache_goal(self, goal_msg):
        """Public method for external nodes (e.g., task_allocator_cnp) to register
        the current navigation goal with the coordinator for yield/resume caching.
        
        Can also be called via a ROS 2 service or topic subscription if needed."""
        self._cached_goal = goal_msg
        self.get_logger().debug(
            f'[{self.robot_id}] Cached Nav2 goal: '
            f'({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})'
        )

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
