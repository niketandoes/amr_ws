#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray, GoalStatus
import math
import csv
import os

class BenchmarkLogger(Node):
    def __init__(self):
        super().__init__('benchmark_logger')
        
        self.robots = ['amr1', 'amr2', 'amr3']
        self.state = {r: {
            'x': 0.0, 'y': 0.0, 
            'v': 0.0, 
            'active_goal': False,
            'start_time': None,
            'distance_traveled': 0.0,
            'deadlock_start': None,
            'total_deadlock_time': 0.0,
            'last_odom_time': None
        } for r in self.robots}
        
        self.min_inter_robot_distance = float('inf')
        
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda msg, r=r: self.odom_callback(msg, r), 10)
            self.create_subscription(GoalStatusArray, f'/{r}/navigate_to_pose/_action/status', lambda msg, r=r: self.status_callback(msg, r), 10)
            
        self.timer = self.create_timer(1.0, self.check_deadlock)
        self.get_logger().info("Benchmark Logger Started")

    def odom_callback(self, msg, robot_id):
        s = self.state[robot_id]
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        new_v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        
        if s['last_odom_time'] is not None and s['active_goal']:
            dx = new_x - s['x']
            dy = new_y - s['y']
            s['distance_traveled'] += math.hypot(dx, dy)
            
        s['x'] = new_x
        s['y'] = new_y
        s['v'] = new_v
        s['last_odom_time'] = self.get_clock().now()
        
        # Check inter-robot distance
        for r2 in self.robots:
            if r2 != robot_id and self.state[r2]['last_odom_time'] is not None:
                d = math.hypot(new_x - self.state[r2]['x'], new_y - self.state[r2]['y'])
                if d < self.min_inter_robot_distance:
                    self.min_inter_robot_distance = d

    def status_callback(self, msg, robot_id):
        s = self.state[robot_id]
        if not msg.status_list:
            return
            
        # Get the latest goal status
        latest_status = msg.status_list[-1].status
        
        if latest_status == GoalStatus.STATUS_EXECUTING or latest_status == GoalStatus.STATUS_ACCEPTED:
            if not s['active_goal']:
                s['active_goal'] = True
                s['start_time'] = self.get_clock().now()
                s['distance_traveled'] = 0.0
                s['total_deadlock_time'] = 0.0
                s['deadlock_start'] = None
                self.min_inter_robot_distance = float('inf') # Reset when new goal starts
                self.get_logger().info(f"{robot_id} started goal")
        elif latest_status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
            if s['active_goal']:
                s['active_goal'] = False
                tct = (self.get_clock().now() - s['start_time']).nanoseconds / 1e9
                self.get_logger().info(f"{robot_id} finished goal. Status: {latest_status}, TCT: {tct:.2f}s, Deadlock: {s['total_deadlock_time']:.2f}s")
                self.export_results(robot_id, latest_status, tct)

    def check_deadlock(self):
        now = self.get_clock().now()
        for r, s in self.state.items():
            if s['active_goal']:
                if s['v'] < 0.02:
                    if s['deadlock_start'] is None:
                        s['deadlock_start'] = now
                    else:
                        deadlock_duration = (now - s['deadlock_start']).nanoseconds / 1e9
                        if deadlock_duration > 15.0:
                            s['total_deadlock_time'] += 1.0 
                else:
                    s['deadlock_start'] = None
                    
    def export_results(self, robot_id, status, tct):
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED"
        }.get(status, "UNKNOWN")
        
        s = self.state[robot_id]
        
        file_path = 'benchmark_results.csv'
        write_header = not os.path.exists(file_path)
        
        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['Robot_ID', 'Outcome', 'Task_Completion_Time_s', 'Deadlock_Duration_s', 'Distance_Traveled_m', 'Min_Inter_Robot_Dist_m'])
            
            writer.writerow([robot_id, status_str, f"{tct:.2f}", f"{s['total_deadlock_time']:.2f}", f"{s['distance_traveled']:.2f}", f"{self.min_inter_robot_distance:.2f}"])

def main():
    rclpy.init()
    node = BenchmarkLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
