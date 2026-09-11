#!/usr/bin/env python3
"""
Fleet Dashboard Server & Real-time Telemetry Bridge
Serves HTML5/Canvas UI on http://localhost:8080 and streams 10 Hz ROS 2 telemetry via Server-Sent Events (SSE).
"""

import os
import sys
import json
import math
import time
import socket
import threading
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse

class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            try:
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass
        super().server_bind()


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

try:
    from amr_interfaces.msg import FleetIntent, TaskAuction, TaskBid, TaskAward
except ImportError:
    FleetIntent = None
    TaskAuction = None
    TaskBid = None
    TaskAward = None


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FleetDashboardNode(Node):
    def __init__(self, static_dir, host='0.0.0.0', port=8080):
        super().__init__('fleet_dashboard_server')
        self.static_dir = static_dir
        self.host = host
        self.port = port

        # Telemetry State Store
        self.fleet_state = {
            'amr1': {'x': -4.0, 'y': 0.0, 'yaw': 0.0, 'speed': 0.0, 'battery': 98.0, 'voltage': 25.1, 'state': 'NORMAL_NAV'},
            'amr2': {'x': 4.0, 'y': 0.0, 'yaw': 3.1415, 'speed': 0.0, 'battery': 86.0, 'voltage': 24.6, 'state': 'NORMAL_NAV'},
            'amr3': {'x': 1.0, 'y': -4.0, 'yaw': 1.5708, 'speed': 0.0, 'battery': 92.0, 'voltage': 24.9, 'state': 'IDLE'}
        }
        self.state_lock = threading.Lock()
        self.event_queue = []
        self.queue_lock = threading.Lock()

        # Odometry and Battery Subscriptions
        for robot in ['amr1', 'amr2', 'amr3']:
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(r, msg),
                10
            )
            self.create_subscription(
                BatteryState,
                f'/{robot}/battery_state',
                lambda msg, r=robot: self.battery_callback(r, msg),
                10
            )

        # Fleet Intent (QoS Best Effort)
        if FleetIntent:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            self.create_subscription(FleetIntent, '/fleet/intent', self.intent_callback, qos)

        # CNP Subscriptions
        if TaskAuction:
            self.create_subscription(TaskAuction, '/fleet/task_auction', self.auction_callback, 10)
        if TaskBid:
            self.create_subscription(TaskBid, '/fleet/task_bids', self.bid_callback, 10)
        if TaskAward:
            self.create_subscription(TaskAward, '/fleet/task_award', self.award_callback, 10)

        # Publishers for Dispatching & Blockage
        self.pub_amr1_goal = self.create_publisher(PoseStamped, '/amr1/goal_pose', 10)
        self.pub_amr2_goal = self.create_publisher(PoseStamped, '/amr2/goal_pose', 10)
        self.pub_amr3_goal = self.create_publisher(PoseStamped, '/amr3/goal_pose', 10)
        self.pub_blockage = self.create_publisher(Bool, '/amr1/trigger_blockage', 10)

        # TF2 listener for accurate global positioning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback)

        # Nav2 Action Clients for direct goal dispatch (no CNP dependency)
        self.nav_clients = {}
        for robot in ['amr1', 'amr2', 'amr3']:
            self.nav_clients[robot] = ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')

        self.get_logger().info(f"FleetDashboardNode initialized. Dashboard files from: {self.static_dir}")

    def tf_timer_callback(self):
        with self.state_lock:
            for robot_id in self.fleet_state.keys():
                try:
                    trans = self.tf_buffer.lookup_transform('map', f'{robot_id}/base_footprint', rclpy.time.Time())
                    self.fleet_state[robot_id]['x'] = round(float(trans.transform.translation.x), 2)
                    self.fleet_state[robot_id]['y'] = round(float(trans.transform.translation.y), 2)
                    yaw = quaternion_to_yaw(trans.transform.rotation)
                    self.fleet_state[robot_id]['yaw'] = round(float(yaw), 3)
                except TransformException:
                    pass

    def odom_callback(self, robot_id, msg: Odometry):
        with self.state_lock:
            if robot_id in self.fleet_state:
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                speed = math.hypot(vx, vy)
                self.fleet_state[robot_id]['speed'] = round(float(speed), 2)

    def battery_callback(self, robot_id, msg: BatteryState):
        with self.state_lock:
            if robot_id in self.fleet_state:
                self.fleet_state[robot_id]['battery'] = round(float(msg.percentage * 100.0), 1)
                self.fleet_state[robot_id]['voltage'] = round(float(msg.voltage), 1)

    def intent_callback(self, msg):
        state_map = {0: 'NORMAL_NAV', 1: 'APPROACHING_CHOKE', 2: 'IN_CHOKE', 3: 'YIELDING', 4: 'BLOCKED'}
        with self.state_lock:
            rid = msg.robot_id
            if rid in self.fleet_state:
                self.fleet_state[rid]['state'] = state_map.get(msg.current_state, 'UNKNOWN')

    def auction_callback(self, msg):
        time_str = time.strftime('%H:%M:%S')
        event = {
            'type': 'auction',
            'time': time_str,
            'message': f"<b>{msg.auctioneer_id.upper()}</b> initiated Task Auction <code>{msg.task_id}</code> at ({msg.target_pose.position.x:.1f}, {msg.target_pose.position.y:.1f})"
        }
        with self.queue_lock:
            self.event_queue.append(event)
        self.get_logger().info(f"CNP Event: Auction {msg.task_id} by {msg.auctioneer_id}")

    def bid_callback(self, msg):
        time_str = time.strftime('%H:%M:%S')
        event = {
            'type': 'bid',
            'time': time_str,
            'message': f"<b>{msg.bidder_id.upper()}</b> placed bid for <code>{msg.task_id}</code> &bull; Heuristic Cost = <b>{msg.bid_cost:.2f}</b>"
        }
        with self.queue_lock:
            self.event_queue.append(event)

    def award_callback(self, msg):
        time_str = time.strftime('%H:%M:%S')
        event = {
            'type': 'success',
            'time': time_str,
            'message': f"🏆 Auction <code>{msg.task_id}</code> AWARDED to <b>{msg.winner_id.upper()}</b>! Route dispatched."
        }
        with self.queue_lock:
            self.event_queue.append(event)

    def dispatch_opposing_test(self):
        """Dispatches opposing goals across choke point via Nav2 action servers."""
        self.get_logger().info("Dispatching opposing choke goals to AMR1 and AMR2...")
        # amr1 goal -> 4.0, -1.5 (well clear of choke zone exit at x=1.5m)
        g1 = PoseStamped()
        g1.header.frame_id = 'map'
        g1.header.stamp = self.get_clock().now().to_msg()
        g1.pose.position.x = 4.0
        g1.pose.position.y = -1.5
        g1.pose.orientation.w = 1.0
        self._send_nav2_goal('amr1', g1)

        # amr2 goal -> -4.0, 1.5
        g2 = PoseStamped()
        g2.header.frame_id = 'map'
        g2.header.stamp = self.get_clock().now().to_msg()
        g2.pose.position.x = -4.0
        g2.pose.position.y = 1.5
        g2.pose.orientation.w = 0.0
        g2.pose.orientation.z = 1.0
        self._send_nav2_goal('amr2', g2)

    def _send_nav2_goal(self, robot_id, pose_stamped):
        """Send goal directly to Nav2 action server for a specific robot."""
        client = self.nav_clients.get(robot_id)
        if client is None:
            self.get_logger().error(f"No Nav2 action client for {robot_id}")
            return
        if not client.server_is_ready():
            self.get_logger().warning(f"Nav2 action server for {robot_id} not ready, falling back to topic")
            # Fallback to topic-based dispatch
            pub = getattr(self, f'pub_{robot_id}_goal', None)
            if pub:
                pub.publish(pose_stamped)
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        client.send_goal_async(goal_msg)
        self.get_logger().info(f"[{robot_id}] Nav2 goal dispatched via action server")

    def trigger_blockage_action(self):
        """Sends synthetic blockage trigger to AMR1."""
        self.get_logger().warning("Triggering synthetic blockage on AMR 1...")
        msg = Bool()
        msg.data = True
        self.pub_blockage.publish(msg)


class DashboardHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, node=None, **kwargs):
        self.node = node
        super().__init__(*args, directory=node.static_dir, **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/events':
            # Server-Sent Events (SSE) stream
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            try:
                while True:
                    # Send telemetry snapshot
                    with self.node.state_lock:
                        data = json.dumps(self.node.fleet_state)
                    self.wfile.write(f"event: telemetry\ndata: {data}\n\n".encode('utf-8'))
                    self.wfile.flush()

                    # Send any queued CNP events
                    with self.node.queue_lock:
                        while self.node.event_queue:
                            ev = self.node.event_queue.pop(0)
                            ev_data = json.dumps(ev)
                            self.wfile.write(f"event: cnp_event\ndata: {ev_data}\n\n".encode('utf-8'))
                            self.wfile.flush()

                    time.sleep(0.1) # 10 Hz
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        elif parsed.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            with self.node.state_lock:
                self.wfile.write(json.dumps(self.node.fleet_state).encode('utf-8'))
            return

        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/dispatch_opposing':
            self.node.dispatch_opposing_test()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'Opposing goals successfully dispatched to AMR 1 and AMR 2'}).encode('utf-8'))
            return
        elif parsed.path == '/api/trigger_blockage':
            self.node.trigger_blockage_action()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'Synthetic blockage triggered on AMR 1'}).encode('utf-8'))
            return

        self.send_error(404, "Endpoint not found")

    def log_message(self, format, *args):
        # Suppress routine GET logging for cleaner console
        return


def main(args=None):
    rclpy.init(args=args)

    # Locate dashboard files: look in src first, fallback to install share directory
    ws_dir = '/home/niket/amr_ws'
    src_dashboard = os.path.join(ws_dir, 'src', 'amr_simulation', 'dashboard')
    if os.path.isdir(src_dashboard):
        static_dir = src_dashboard
    else:
        try:
            pkg_share = get_package_share_directory('amr_simulation')
            static_dir = os.path.join(pkg_share, 'dashboard')
        except Exception:
            static_dir = src_dashboard

    node = FleetDashboardNode(static_dir=static_dir, port=8080)

    # Launch HTTP Server in daemon thread
    def run_server():
        handler = lambda *hargs, **hkwargs: DashboardHTTPRequestHandler(*hargs, node=node, **hkwargs)
        server = None
        for p in [8080, 8081, 8082]:
            try:
                server = ReusableThreadingHTTPServer(('0.0.0.0', p), handler)
                break
            except OSError:
                continue
        if server is None:
            print("❌ Error: Could not bind HTTP Server to port 8080 or alternate ports.")
            return

        port = server.server_address[1]
        print("=" * 70)
        print(f"  🚀 AMR FLEET DASHBOARD LIVE AT: http://localhost:{port}")
        print("=" * 70)
        server.serve_forever()

    http_thread = threading.Thread(target=run_server, daemon=True)
    http_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
