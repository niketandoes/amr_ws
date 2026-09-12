#!/usr/bin/env python3
"""
Fleet Dashboard Server & Real-time Telemetry Bridge
Serves HTML5/Canvas UI on http://localhost:8080 and streams 10 Hz ROS 2 telemetry via Server-Sent Events (SSE).
Dynamically discovers AMRs via ROS 2 topics and serves map definitions.
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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

DEFAULT_INITIAL_POSES = {
    'amr1': {'x': -4.0, 'y': 0.0, 'yaw': 0.0},
    'amr2': {'x': 4.0, 'y': 0.0, 'yaw': 3.14159},
    'amr3': {'x': 1.0, 'y': -4.0, 'yaw': 1.5708}
}

class FleetDashboardNode(Node):
    def __init__(self, static_dir, map_dir, host='0.0.0.0', port=8080):
        super().__init__('fleet_dashboard_server')
        self.static_dir = static_dir
        self.map_dir = map_dir
        self.host = host
        self.port = port

        # Dynamic Telemetry State Store
        self.fleet_state = {}
        self.known_robots = set()
        self.subscribed_robots = set()
        self.state_lock = threading.Lock()
        self.event_queue = []
        self.queue_lock = threading.Lock()

        # TF2 listener for accurate global positioning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback)
        
        # Discovery timer
        self.discovery_timer = self.create_timer(2.0, self.discover_robots)
        
        self.nav_clients = {}

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

        # Blockage publisher (still hardcoded to amr1 for the demo test)
        self.pub_blockage = self.create_publisher(Bool, '/amr1/trigger_blockage', 10)

        self.get_logger().info(f"FleetDashboardNode initialized. Dashboard files from: {self.static_dir}")
        self.get_logger().info(f"Map directory: {self.map_dir}")

    def discover_robots(self):
        topics_and_types = self.get_topic_names_and_types()
        active_discovered = set()
        for topic_name, _ in topics_and_types:
            if topic_name.endswith('/odom'):
                parts = topic_name.split('/')
                if len(parts) >= 3:
                    robot_id = parts[1]
                    active_discovered.add(robot_id)
                    if robot_id not in self.known_robots:
                        self.get_logger().info(f"Dynamically discovered new robot: {robot_id}")
                        self.known_robots.add(robot_id)
                        with self.state_lock:
                            self.fleet_state[robot_id] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'speed': 0.0, 'battery': 100.0, 'voltage': 24.0, 'state': 'UNKNOWN', 'last_seen': time.time()}
                        
                        if robot_id not in self.subscribed_robots:
                            # Create subscriptions for this robot
                            self.create_subscription(Odometry, f'/{robot_id}/odom', lambda msg, r=robot_id: self.odom_callback(r, msg), 10)
                            self.create_subscription(BatteryState, f'/{robot_id}/battery_state', lambda msg, r=robot_id: self.battery_callback(r, msg), 10)
                            self.nav_clients[robot_id] = ActionClient(self, NavigateToPose, f'/{robot_id}/navigate_to_pose')
                            setattr(self, f'pub_{robot_id}_goal', self.create_publisher(PoseStamped, f'/{robot_id}/goal_pose', 10))
                            setattr(self, f'pub_{robot_id}_initialpose', self.create_publisher(PoseWithCovarianceStamped, f'/{robot_id}/initialpose', 10))
                            setattr(self, f'pub_{robot_id}_cmd_vel', self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10))
                            self.subscribed_robots.add(robot_id)

        # Purge disconnected robots (no odom for 5 seconds)
        current_time = time.time()
        with self.state_lock:
            stale_robots = []
            for r_id, state in self.fleet_state.items():
                if current_time - state.get('last_seen', current_time) > 5.0:
                    stale_robots.append(r_id)
            
            for r_id in stale_robots:
                self.get_logger().warning(f"Robot {r_id} disconnected. Purging from dashboard.")
                self.known_robots.discard(r_id)
                del self.fleet_state[r_id]

    def tf_timer_callback(self):
        with self.state_lock:
            for robot_id in self.fleet_state.keys():
                try:
                    trans = self.tf_buffer.lookup_transform('map', f'{robot_id}/base_footprint', rclpy.time.Time())
                    self.fleet_state[robot_id]['x'] = round(float(trans.transform.translation.x), 2)
                    self.fleet_state[robot_id]['y'] = round(float(trans.transform.translation.y), 2)
                    yaw = quaternion_to_yaw(trans.transform.rotation)
                    self.fleet_state[robot_id]['yaw'] = round(float(yaw), 3)
                    self.fleet_state[robot_id]['last_seen'] = time.time()
                except TransformException:
                    pass

    def odom_callback(self, robot_id, msg: Odometry):
        with self.state_lock:
            if robot_id in self.fleet_state:
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                speed = math.hypot(vx, vy)
                self.fleet_state[robot_id]['speed'] = round(float(speed), 2)
                self.fleet_state[robot_id]['last_seen'] = time.time()

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
        if 'amr1' not in self.known_robots or 'amr2' not in self.known_robots:
            return {'status': 'Failed: amr1 and amr2 must both be online for opposing test.'}

        self.get_logger().info("Dispatching opposing choke goals to AMR1 and AMR2...")
        # amr1 goal -> 4.0, -1.5
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
        return {'status': 'Opposing goals successfully dispatched to AMR 1 and AMR 2'}

    def _send_nav2_goal(self, robot_id, pose_stamped):
        client = self.nav_clients.get(robot_id)
        if client is None:
            return
        if not client.server_is_ready():
            self.get_logger().warning(f"Nav2 action server for {robot_id} not ready, falling back to topic")
            pub = getattr(self, f'pub_{robot_id}_goal', None)
            if pub:
                pub.publish(pose_stamped)
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        client.send_goal_async(goal_msg)

    def trigger_blockage_action(self):
        self.get_logger().warning("Triggering synthetic blockage on AMR 1...")
        msg = Bool()
        msg.data = True
        self.pub_blockage.publish(msg)

    def reset_fleet_poses(self):
        """Resets all active AMRs to their home spawn poses and clears active goals."""
        self.get_logger().info("Resetting fleet poses...")
        import subprocess

        target_robots = set(self.known_robots) | {'amr1', 'amr2', 'amr3'}

        for robot_id in target_robots:
            pose_cfg = DEFAULT_INITIAL_POSES.get(robot_id, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
            x = pose_cfg['x']
            y = pose_cfg['y']
            yaw = pose_cfg['yaw']

            # 1. Stop motion (zero velocity)
            pub_vel = getattr(self, f'pub_{robot_id}_cmd_vel', None)
            if pub_vel is None:
                pub_vel = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)
                setattr(self, f'pub_{robot_id}_cmd_vel', pub_vel)
            tw = Twist()
            pub_vel.publish(tw)

            # 2. Publish initialpose to ROS 2 localizer / Nav2
            pub_init = getattr(self, f'pub_{robot_id}_initialpose', None)
            if pub_init is None:
                pub_init = self.create_publisher(PoseWithCovarianceStamped, f'/{robot_id}/initialpose', 10)
                setattr(self, f'pub_{robot_id}_initialpose', pub_init)

            init_msg = PoseWithCovarianceStamped()
            init_msg.header.frame_id = 'map'
            init_msg.header.stamp = self.get_clock().now().to_msg()
            init_msg.pose.pose.position.x = x
            init_msg.pose.pose.position.y = y
            init_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            init_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            init_msg.pose.covariance[0] = 0.25
            init_msg.pose.covariance[7] = 0.25
            init_msg.pose.covariance[35] = 0.0685
            pub_init.publish(init_msg)

            # 3. Best-effort Ignition Gazebo entity pose reset
            try:
                qw = math.cos(yaw / 2.0)
                qz = math.sin(yaw / 2.0)
                cmd = [
                    'ign', 'service',
                    '-s', '/world/warehouse_world/set_pose',
                    '--reqtype', 'ign.msgs.Pose',
                    '--reptype', 'ign.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', f'name: "{robot_id}", position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{w: {qw}, x: 0.0, y: 0.0, z: {qz}}}'
                ]
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as e:
                self.get_logger().warning(f"Ignition set_pose service call failed for {robot_id}: {e}")

        return {'status': 'Fleet poses reset to home spawn positions.'}


class DashboardHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, node=None, **kwargs):
        self.node = node
        super().__init__(*args, directory=node.static_dir, **kwargs)

    def end_headers(self):
        # Prevent browser caching of static files (HTML, JS, CSS)
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            try:
                while True:
                    with self.node.state_lock:
                        data = json.dumps(self.node.fleet_state)
                    self.wfile.write(f"event: telemetry\ndata: {data}\n\n".encode('utf-8'))
                    self.wfile.flush()

                    with self.node.queue_lock:
                        while self.node.event_queue:
                            ev = self.node.event_queue.pop(0)
                            ev_data = json.dumps(ev)
                            self.wfile.write(f"event: cnp_event\ndata: {ev_data}\n\n".encode('utf-8'))
                            self.wfile.flush()

                    time.sleep(0.1)
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
        elif parsed.path == '/api/map.pgm':
            map_path = os.path.join(self.node.map_dir, 'warehouse_map.pgm')
            if os.path.exists(map_path):
                self.send_response(200)
                self.send_header('Content-Type', 'image/x-portable-graymap')
                self.end_headers()
                with open(map_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404, "Map PGM not found")
            return
        elif parsed.path == '/api/map.yaml':
            yaml_path = os.path.join(self.node.map_dir, 'warehouse_map.yaml')
            if os.path.exists(yaml_path):
                self.send_response(200)
                self.send_header('Content-Type', 'text/yaml')
                self.end_headers()
                with open(yaml_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404, "Map YAML not found")
            return

        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/dispatch_opposing':
            result = self.node.dispatch_opposing_test()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
            return
        elif parsed.path == '/api/trigger_blockage':
            self.node.trigger_blockage_action()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'Synthetic blockage triggered on AMR 1'}).encode('utf-8'))
            return
        elif parsed.path == '/api/reset_poses':
            result = self.node.reset_fleet_poses()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
            return

        self.send_error(404, "Endpoint not found")

    def log_message(self, format, *args):
        return

def main(args=None):
    rclpy.init(args=args)

    try:
        pkg_share = get_package_share_directory('amr_simulation')
        static_dir = os.path.join(pkg_share, 'dashboard')
        map_dir = os.path.join(pkg_share, 'maps')
    except Exception:
        # Fallback for development without source install
        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        static_dir = os.path.join(ws_dir, 'src', 'amr_simulation', 'dashboard')
        map_dir = os.path.join(ws_dir, 'src', 'amr_simulation', 'maps')

    node = FleetDashboardNode(static_dir=static_dir, map_dir=map_dir, port=8080)

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
        print("=" * 70, flush=True)
        print(f"  🚀 AMR FLEET DASHBOARD LIVE AT: http://localhost:{port}", flush=True)
        print("=" * 70, flush=True)
        server.serve_forever()

    http_thread = threading.Thread(target=run_server, daemon=True)
    http_thread.start()

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
