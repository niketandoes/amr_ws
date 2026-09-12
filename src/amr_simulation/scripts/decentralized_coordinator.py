#!/usr/bin/env python3
"""
Decentralized Fleet Coordinator Node — Zone Reservation Protocol (v2)
Implements peer-to-peer (P2P) right-of-way arbitration without central dispatchers.

Protocol:
  1. A bounding-box defines the critical corridor zone (from warehouse.sdf geometry).
  2. Each robot broadcasts ground-truth zone occupancy via FleetIntent.is_in_choke_zone.
  3. Occupancy ALWAYS gates entry: if any peer is inside the zone (or clearing with
     active hysteresis), all approaching peers yield.
  4. Fail-safe cold start: a robot entering the approach zone with an incomplete
     peer picture (hasn't yet heard from every expected fleet member) holds
     briefly rather than assuming "no data = clear." Bounded by
     APPROACH_PEER_GRACE_SEC so a genuinely dead peer can't deadlock the fleet.
  5. Tie-break when the corridor is empty and multiple robots are waiting:
     earliest approach_request_time wins (fair, starvation-free), string-ID
     only breaks an exact timestamp tie.
  6. Exit hysteresis requires a peer to report is_in_choke_zone=False for N
     consecutive ticks (~500ms) before the corridor lock is released.
  7. Stale peer intents (>1.5s without update) are pruned to prevent ghost lockups.

Velocity Architecture:
  - Nav2 controller_server publishes to cmd_vel_nav (twist_mux priority 10)
  - This coordinator publishes halt commands to cmd_vel_coord (priority 20)
  - twist_mux merges both onto cmd_vel for the Gazebo bridge
  - Resume relies on twist_mux's 0.5s per-topic timeout: once this node stops
    publishing halts, twist_mux falls back to cmd_vel_nav automatically. Nav2's
    own goal was never cancelled, so it keeps executing the moment its output
    isn't being overridden — no cross-node goal-handle plumbing needed.

Inputs:
  - /fleet/intent (amr_interfaces/msg/FleetIntent)

Outputs:
  - cmd_vel_coord (geometry_msgs/msg/TwistStamped) [Override during yielding via twist_mux]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from amr_interfaces.msg import FleetIntent
import time


class DecentralizedCoordinator(Node):

    CHOKE_X_MIN = -1.0
    CHOKE_X_MAX = 1.0
    CHOKE_Y_MIN = -0.80
    CHOKE_Y_MAX = 0.80

    APPROACH_X_LIMIT = 4.0
    APPROACH_Y_LIMIT = 2.5

    HYSTERESIS_TICKS_REQUIRED = 5      # ~500ms at 10Hz
    INTENT_EXPIRY_SEC = 1.5

    APPROACH_PEER_GRACE_SEC = 2.0

    def __init__(self):
        super().__init__('decentralized_coordinator')
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'default_amr'

        self.declare_parameter('fleet_robot_ids', ['amr1', 'amr2', 'amr3'])
        fleet_ids = self.get_parameter('fleet_robot_ids').value
        self.expected_peers = [r for r in fleet_ids if r != self.robot_id]

        p2p_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.intent_sub = self.create_subscription(FleetIntent, '/fleet/intent', self.intent_callback, p2p_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_coord', 10)

        self.peer_intents = {}
        self.peer_timestamps = {}
        self.my_intent = None
        self.yielding_to = None
        self.peer_clear_ticks = {}
        self.peer_has_entered_zone = {}
        self.peer_has_passed = {}
        self._warned_incomplete_peers = False

        self.create_timer(0.1, self.evaluate_conflicts)  # 10 Hz
        self.get_logger().info(
            f'[{self.robot_id}] Decentralized Coordinator online '
            f'(expecting peers: {self.expected_peers}, '
            f'box [{self.CHOKE_X_MIN},{self.CHOKE_X_MAX}] x [{self.CHOKE_Y_MIN},{self.CHOKE_Y_MAX}], '
            f'hysteresis={self.HYSTERESIS_TICKS_REQUIRED} ticks)'
        )

    def intent_callback(self, msg):
        if msg.robot_id == self.robot_id:
            self.my_intent = msg
        else:
            self.peer_intents[msg.robot_id] = msg
            self.peer_timestamps[msg.robot_id] = time.monotonic()

    def _self_in_approach_zone(self):
        if not self.my_intent:
            return False
        x = self.my_intent.current_pose.position.x
        y = self.my_intent.current_pose.position.y
        return abs(x) <= self.APPROACH_X_LIMIT and abs(y) <= self.APPROACH_Y_LIMIT

    def _self_in_choke_zone(self):
        if not self.my_intent:
            return False
        x = self.my_intent.current_pose.position.x
        y = self.my_intent.current_pose.position.y
        return (self.CHOKE_X_MIN <= x <= self.CHOKE_X_MAX and
                self.CHOKE_Y_MIN <= y <= self.CHOKE_Y_MAX)

    def _is_peer_in_approach_or_choke(self, peer_intent):
        px = peer_intent.current_pose.position.x
        py = peer_intent.current_pose.position.y
        in_choke = (self.CHOKE_X_MIN <= px <= self.CHOKE_X_MAX and self.CHOKE_Y_MIN <= py <= self.CHOKE_Y_MAX)
        in_approach = (abs(px) <= self.APPROACH_X_LIMIT and abs(py) <= self.APPROACH_Y_LIMIT)
        return in_choke or in_approach

    def _peer_confirmed_clear(self, peer_id):
        return self.peer_clear_ticks.get(peer_id, 0) >= self.HYSTERESIS_TICKS_REQUIRED

    def _all_expected_peers_seen(self):
        return all(pid in self.peer_intents for pid in self.expected_peers)

    def _prune_stale_intents(self):
        now = time.monotonic()
        stale_peers = [pid for pid, ts in self.peer_timestamps.items()
                       if (now - ts) > self.INTENT_EXPIRY_SEC]
        for pid in stale_peers:
            self.get_logger().warn(
                f'[{self.robot_id}] Pruning stale intent from {pid} '
                f'(no update for >{self.INTENT_EXPIRY_SEC}s). Assuming clear.',
                throttle_duration_sec=5.0
            )
            self.peer_intents.pop(pid, None)
            self.peer_timestamps.pop(pid, None)
            self.peer_clear_ticks.pop(pid, None)
            self.peer_has_entered_zone.pop(pid, None)
            self.peer_has_passed.pop(pid, None)
            if self.yielding_to == pid:
                self.yielding_to = None

    def evaluate_conflicts(self):
        if not self.my_intent:
            return

        self._prune_stale_intents()

        for peer_id, intent in self.peer_intents.items():
            if intent.is_in_choke_zone:
                self.peer_clear_ticks[peer_id] = 0
                self.peer_has_entered_zone[peer_id] = True
            else:
                self.peer_clear_ticks[peer_id] = self.peer_clear_ticks.get(peer_id, 0) + 1

            if self.peer_has_passed.get(peer_id, False) and not self._is_peer_in_approach_or_choke(intent):
                self.peer_has_passed[peer_id] = False

        if self.yielding_to and self.yielding_to != 'UNKNOWN_PEER':
            if self.yielding_to in self.peer_intents:
                peer = self.peer_intents[self.yielding_to]
                has_entered = self.peer_has_entered_zone.get(self.yielding_to, False)
                if has_entered and not peer.is_in_choke_zone and self._peer_confirmed_clear(self.yielding_to):
                    self.get_logger().info(
                        f'[{self.robot_id}] {self.yielding_to} completed choke transit & confirmed clear. Resuming.'
                    )
                    self.peer_has_passed[self.yielding_to] = True
                    self.yielding_to = None
                elif not self._is_peer_in_approach_or_choke(peer):
                    self.get_logger().info(
                        f'[{self.robot_id}] {self.yielding_to} left approach/choke zone. Resuming.'
                    )
                    self.yielding_to = None
                else:
                    self._halt(self.yielding_to)
                    return
            else:
                self.get_logger().info(f'[{self.robot_id}] {self.yielding_to} disappeared from mesh. Resuming.')
                self.yielding_to = None

        if not self._self_in_approach_zone():
            self._warned_incomplete_peers = False
            return

        if not self._all_expected_peers_seen():
            my_wait_start = self.my_intent.approach_request_time
            elapsed = (time.monotonic() - my_wait_start) if my_wait_start > 0 else 0.0
            if elapsed < self.APPROACH_PEER_GRACE_SEC:
                self.get_logger().warn(
                    f'[{self.robot_id}] Incomplete peer picture on approach '
                    f'(missing: {[p for p in self.expected_peers if p not in self.peer_intents]}) — '
                    f'holding for up to {self.APPROACH_PEER_GRACE_SEC}s before proceeding.',
                    throttle_duration_sec=1.0
                )
                self.yielding_to = 'UNKNOWN_PEER'
                self._halt('an unseen peer (grace period)')
                return
            elif not self._warned_incomplete_peers:
                self.get_logger().warn(
                    f'[{self.robot_id}] Grace period expired, still missing peers. Proceeding.'
                )
                self._warned_incomplete_peers = True

        i_am_in_zone = self._self_in_choke_zone()

        for peer_id, intent in self.peer_intents.items():
            if self.peer_has_passed.get(peer_id, False):
                continue

            peer_in_zone = intent.is_in_choke_zone
            peer_clearing = not peer_in_zone and self.peer_has_entered_zone.get(peer_id, False) and not self._peer_confirmed_clear(peer_id)
            if peer_in_zone or peer_clearing:
                if not i_am_in_zone:
                    self.yielding_to = peer_id
                    self._halt(peer_id)
                    return

        if i_am_in_zone:
            return

        approaching_peers = []
        for peer_id, intent in self.peer_intents.items():
            if self.peer_has_passed.get(peer_id, False):
                continue

            px, py = intent.current_pose.position.x, intent.current_pose.position.y
            if abs(px) <= self.APPROACH_X_LIMIT and abs(py) <= self.APPROACH_Y_LIMIT:
                req_time = intent.approach_request_time if intent.approach_request_time > 0 else float('inf')
                approaching_peers.append((req_time, peer_id))

        if not approaching_peers:
            return

        my_req_time = self.my_intent.approach_request_time if self.my_intent.approach_request_time > 0 else float('inf')
        all_candidates = approaching_peers + [(my_req_time, self.robot_id)]
        all_candidates.sort()
        winner_id = all_candidates[0][1]

        if self.robot_id != winner_id:
            self.yielding_to = winner_id
            self._halt(winner_id)

    def _halt(self, reason):
        self.get_logger().info(f'[{self.robot_id}] Yielding to {reason} at choke zone.', throttle_duration_sec=1.5)
        halt_msg = Twist()
        self.cmd_vel_pub.publish(halt_msg)


def main():
    rclpy.init()
    node = DecentralizedCoordinator()
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
