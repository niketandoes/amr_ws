#!/usr/bin/env python3
"""
Contract Net Protocol (CNP) Dynamic Task Allocator for AMRs
Detects blockages and orchestrates decentralized task auctions, bidding, and awards.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from amr_interfaces.msg import TaskAuction, TaskBid, TaskAward


class TaskAllocatorCNP(Node):
    def __init__(self):
        super().__init__('task_allocator_cnp')

        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'amr1'

        self.current_pose = None
        self.current_speed = 0.0
        self.current_battery_pct = 100.0
        self.is_navigating = False
        self.active_goal_pose = None
        self.stalled_start_time = None
        self.is_blocked = False

        # Active auction state
        self.active_auction = None
        self.collected_bids = []
        self.auction_timer = None

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )

        # Fleet CNP Topics
        self.auction_pub = self.create_publisher(TaskAuction, '/fleet/task_auction', 10)
        self.bid_pub = self.create_publisher(TaskBid, '/fleet/task_bids', 10)
        self.award_pub = self.create_publisher(TaskAward, '/fleet/task_award', 10)

        self.auction_sub = self.create_subscription(
            TaskAuction,
            '/fleet/task_auction',
            self.auction_callback,
            10
        )
        self.bid_sub = self.create_subscription(
            TaskBid,
            '/fleet/task_bids',
            self.bid_callback,
            10
        )
        self.award_sub = self.create_subscription(
            TaskAward,
            '/fleet/task_award',
            self.award_callback,
            10
        )

        # Trigger blockage hook (useful for interactive simulation & testing)
        self.blockage_sub = self.create_subscription(
            Bool,
            'trigger_blockage',
            self.trigger_blockage_callback,
            10
        )

        # Goal dispatch subscription
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10
        )

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Periodic check for stall/blockage
        self.monitor_timer = self.create_timer(0.5, self.monitor_blockage)

        self.get_logger().info(f"TaskAllocatorCNP initialized for {self.robot_id}")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def battery_callback(self, msg: BatteryState):
        self.current_battery_pct = msg.percentage * 100.0

    def goal_pose_callback(self, msg: PoseStamped):
        self.active_goal_pose = msg.pose
        self.is_navigating = True
        self.is_blocked = False
        self.stalled_start_time = None
        self.get_logger().info(f"[{self.robot_id}] Tracking active goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

    def trigger_blockage_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warning(f"[{self.robot_id}] Synthetic blockage triggered!")
            self.initiate_task_auction()

    def monitor_blockage(self):
        """Monitors velocity when actively navigating to detect stuck/blocked condition."""
        if not self.is_navigating or self.is_blocked or self.active_goal_pose is None:
            return

        now = time.time()
        # If moving below 0.05 m/s
        if self.current_speed < 0.05:
            if self.stalled_start_time is None:
                self.stalled_start_time = now
            elif now - self.stalled_start_time > 4.0:
                self.get_logger().warning(
                    f"[{self.robot_id}] Blockage detected! Stalled for {now - self.stalled_start_time:.1f}s while navigating."
                )
                self.initiate_task_auction()
        else:
            self.stalled_start_time = None

    def initiate_task_auction(self):
        """Flags task blocked, broadcasts TaskAuction, and begins 500ms bidding window."""
        self.is_blocked = True
        self.is_navigating = False
        self.stalled_start_time = None

        if self.active_goal_pose is None:
            # Fallback default target if none set
            self.active_goal_pose = Pose()
            self.active_goal_pose.position.x = 3.5
            self.active_goal_pose.position.y = -1.5

        task_id = f"task_{self.robot_id}_{int(time.time() * 1000) % 100000}"
        self.active_auction = {
            'task_id': task_id,
            'auctioneer_id': self.robot_id,
            'target_pose': self.active_goal_pose,
            'start_time': time.time()
        }
        self.collected_bids = []

        auction_msg = TaskAuction()
        auction_msg.task_id = task_id
        auction_msg.auctioneer_id = self.robot_id
        auction_msg.target_pose = self.active_goal_pose
        auction_msg.timeout_seconds = 0.5
        auction_msg.stamp = self.get_clock().now().to_msg()

        self.auction_pub.publish(auction_msg)
        self.get_logger().info(f"[{self.robot_id}] Published TaskAuction: {task_id}")

        # Set 500ms timer to close auction
        if self.auction_timer:
            self.auction_timer.cancel()
        self.auction_timer = self.create_timer(0.5, self.close_auction)

    def auction_callback(self, msg: TaskAuction):
        """Evaluates peer auctions and places a bid if available."""
        if msg.auctioneer_id == self.robot_id:
            return  # Ignore own auction

        if self.is_blocked:
            self.get_logger().info(f"[{self.robot_id}] Ignoring auction {msg.task_id} (I am blocked)")
            return

        if self.current_pose is None:
            return

        # Calculate heuristic bid cost
        # Cost = (1.0 * Distance) + (0.5 * (100 - Battery))
        dx = msg.target_pose.position.x - self.current_pose.position.x
        dy = msg.target_pose.position.y - self.current_pose.position.y
        dist = math.hypot(dx, dy)
        battery_penalty = max(0.0, 100.0 - self.current_battery_pct)

        # If currently navigating, add busy penalty of 2.0
        busy_penalty = 2.0 if self.is_navigating else 0.0
        bid_cost = float((1.0 * dist) + (0.5 * battery_penalty) + busy_penalty)

        bid_msg = TaskBid()
        bid_msg.task_id = msg.task_id
        bid_msg.bidder_id = self.robot_id
        bid_msg.bid_cost = bid_cost
        bid_msg.stamp = self.get_clock().now().to_msg()

        # Slight jitter to avoid simultaneous publish collisions
        self.bid_pub.publish(bid_msg)
        self.get_logger().info(
            f"[{self.robot_id}] Placed bid for {msg.task_id}: Cost={bid_cost:.2f} (Dist={dist:.2f}m, Batt={self.current_battery_pct:.1f}%)"
        )

    def bid_callback(self, msg: TaskBid):
        """Auctioneer collects incoming peer bids."""
        if self.active_auction and msg.task_id == self.active_auction['task_id']:
            self.collected_bids.append(msg)
            self.get_logger().info(f"[{self.robot_id}] Received bid from {msg.bidder_id}: {msg.bid_cost:.2f}")

    def close_auction(self):
        """Selects lowest bid, publishes award, and reroutes blocked robot."""
        if self.auction_timer:
            self.auction_timer.cancel()
            self.auction_timer = None

        if not self.active_auction:
            return

        task_id = self.active_auction['task_id']
        if self.collected_bids:
            self.collected_bids.sort(key=lambda b: b.bid_cost)
            winner = self.collected_bids[0]
            winner_id = winner.bidder_id

            award_msg = TaskAward()
            award_msg.task_id = task_id
            award_msg.winner_id = winner_id
            award_msg.stamp = self.get_clock().now().to_msg()
            self.award_pub.publish(award_msg)

            self.get_logger().info(
                f"[{self.robot_id}] Auction {task_id} CLOSED. Winner: {winner_id} with bid {winner.bid_cost:.2f}"
            )
        else:
            self.get_logger().warning(f"[{self.robot_id}] Auction {task_id} received 0 bids. Retrying in 2.0s...")
            self.active_auction = None
            return

        self.active_auction = None

        # Reroute blocked robot to standby holding spot
        self.reroute_to_standby()

    def reroute_to_standby(self):
        """Dispatches goal to safe pull-over spot to clear the lane."""
        self.get_logger().info(f"[{self.robot_id}] Rerouting to standby holding zone...")
        standby_pose = PoseStamped()
        standby_pose.header.frame_id = 'map'
        standby_pose.header.stamp = self.get_clock().now().to_msg()

        # Standby coordinates outside main lanes
        if self.robot_id == 'amr1':
            standby_pose.pose.position.x = -3.5
            standby_pose.pose.position.y = 2.0
        elif self.robot_id == 'amr2':
            standby_pose.pose.position.x = 3.5
            standby_pose.pose.position.y = 2.0
        else:
            standby_pose.pose.position.x = 0.0
            standby_pose.pose.position.y = -3.5
        standby_pose.pose.orientation.w = 1.0

        self.dispatch_nav2_goal(standby_pose)

    def award_callback(self, msg: TaskAward):
        """Processes task awards; winner executes the delivery goal."""
        if msg.winner_id == self.robot_id:
            self.get_logger().info(f"🎉 [{self.robot_id}] I WON TASK AUCTION {msg.task_id}! Executing delivery.")
            self.is_navigating = True
            self.is_blocked = False

            target = PoseStamped()
            target.header.frame_id = 'map'
            target.header.stamp = self.get_clock().now().to_msg()
            # Standard mission delivery waypoint
            target.pose.position.x = 3.5
            target.pose.position.y = -1.5
            target.pose.orientation.w = 1.0

            self.dispatch_nav2_goal(target)

    def dispatch_nav2_goal(self, pose_stamped: PoseStamped):
        """Sends goal to Nav2 action server."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning(f"[{self.robot_id}] Nav2 navigate_to_pose action server not ready.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.nav_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocatorCNP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
