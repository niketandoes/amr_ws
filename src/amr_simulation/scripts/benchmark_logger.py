#!/usr/bin/env python3
"""
Benchmark Logger Node for Multi-Robot Navigation
Logs individual robot odometry, goal status transitions, deadlock durations,
traveled distance, and minimum inter-robot distance into CSV format.

Inputs:
  - /{robot_id}/odom (nav_msgs/msg/Odometry)
  - /{robot_id}/navigate_to_pose/_action/status (action_msgs/msg/GoalStatusArray)

Outputs:
  - Appends performance metrics to benchmark_results.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray, GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
import math
import csv
import os

class BenchmarkLogger(Node):
    """
    Subscribes to fleet odometry and Nav2 goal status arrays to record
    task start/end times, distance traversed, and detect deadlocks.
    """
    def __init__(self):
        super().__init__('benchmark_logger')
        
        self.robots = ['amr1', 'amr2', 'amr3']
        self.state = {r: {
            'x': None, 'y': None, 
            'v': 0.0, 
            'active_goal': False,
            'start_time': None,
            'distance_traveled': 0.0,
            'deadlock_start': None,
            'total_deadlock_time': 0.0,
            'last_odom_time': None,
            'min_dist': float('inf'),
            'last_goal_id': None,
            'last_status': None,
            'has_exported': False
        } for r in self.robots}
        
        # TF2 listener for accurate global positioning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback)

        for r in self.robots:
            self.create_subscription(GoalStatusArray, f'/{r}/navigate_to_pose/_action/status', lambda msg, r=r: self.status_callback(msg, r), 10)

        self.timer = self.create_timer(0.5, self.check_deadlock)
        self.get_logger().info("Benchmark Logger Initialized and Monitoring TF and Goal Statuses")

    def tf_timer_callback(self):
        for r in self.robots:
            try:
                trans = self.tf_buffer.lookup_transform('map', f'{r}/base_footprint', rclpy.time.Time())
                self.process_position(r, trans.transform.translation.x, trans.transform.translation.y)
            except TransformException:
                pass

    def process_position(self, robot_id, new_x, new_y):
        s = self.state[robot_id]
        now = self.get_clock().now()
        
        if s['x'] is not None and s['active_goal']:
            dx = new_x - s['x']
            dy = new_y - s['y']
            dist = math.hypot(dx, dy)
            if dist > 0.005:
                s['distance_traveled'] += dist
            
            if s['last_odom_time'] is not None:
                dt = (now - s['last_odom_time']).nanoseconds / 1e9
                if dt > 0:
                    s['v'] = dist / dt
            
            self.get_logger().debug(f"[{robot_id}] Moving: v={s['v']:.3f}m/s, dist={s['distance_traveled']:.2f}m")
            
        s['x'] = new_x
        s['y'] = new_y
        s['last_odom_time'] = now
        
        # Calculate distance to other robots — only when both have active goals
        # and valid (non-null) odometry to avoid false 0.00m readings
        for r2 in self.robots:
            if r2 != robot_id:
                s2 = self.state[r2]
                if s2['x'] is not None and s['active_goal'] and s2['active_goal']:
                    d = math.hypot(new_x - s2['x'], new_y - s2['y'])
                    if d < s['min_dist']:
                        s['min_dist'] = d
                    if d < s2['min_dist']:
                        s2['min_dist'] = d

    def status_callback(self, msg, robot_id):
        s = self.state[robot_id]
        if not msg.status_list:
            return
            
        latest_status = msg.status_list[-1].status
        goal_id = bytes(msg.status_list[-1].goal_info.goal_id.uuid).hex()
        
        if s['last_goal_id'] == goal_id and s['last_status'] == latest_status:
            return
            
        s['last_goal_id'] = goal_id
        s['last_status'] = latest_status
        
        self.get_logger().info(f"[{robot_id}] Goal {goal_id[:8]} transitioned to status: {latest_status}")
        
        if latest_status in [GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED]:
            if not s['active_goal']:
                s['active_goal'] = True
                s['start_time'] = self.get_clock().now()
                s['distance_traveled'] = 0.0
                s['total_deadlock_time'] = 0.0
                s['deadlock_start'] = None
                s['min_dist'] = float('inf')
                s['has_exported'] = False
                self.get_logger().info(f"[{robot_id}] Mission started. Benchmark logging active.")
        elif latest_status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
            if s['active_goal'] and not s['has_exported']:
                s['active_goal'] = False
                s['has_exported'] = True
                tct = (self.get_clock().now() - s['start_time']).nanoseconds / 1e9
                self.get_logger().info(f"[{robot_id}] Mission Finished. Status: {latest_status}, TCT: {tct:.2f}s, Distance: {s['distance_traveled']:.2f}m, Deadlock: {s['total_deadlock_time']:.2f}s")
                self.export_results(robot_id, latest_status, tct)

    def check_deadlock(self):
        now = self.get_clock().now()
        for r, s in self.state.items():
            if s['active_goal']:
                if s['v'] < 0.02:
                    if s['deadlock_start'] is None:
                        s['deadlock_start'] = now
                    else:
                        deadlock_dur = (now - s['deadlock_start']).nanoseconds / 1e9
                        if deadlock_dur > 3.0:  # Count deadlock after 3 seconds of standstill
                            s['total_deadlock_time'] += 0.5
                else:
                    s['deadlock_start'] = None
                    
    def export_results(self, robot_id, status, tct):
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED"
        }.get(status, "UNKNOWN")
        
        s = self.state[robot_id]
        min_d_str = f"{s['min_dist']:.2f}" if s['min_dist'] != float('inf') else "N/A"
        
        file_path = os.path.expanduser('~/amr_ws/benchmark_results.csv')
        write_header = not os.path.exists(file_path)
        
        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['Robot_ID', 'Outcome', 'Task_Completion_Time_s', 'Deadlock_Duration_s', 'Distance_Traveled_m', 'Min_Inter_Robot_Dist_m'])
            
            writer.writerow([robot_id, status_str, f"{tct:.2f}", f"{s['total_deadlock_time']:.2f}", f"{s['distance_traveled']:.2f}", min_d_str])

def main():
    rclpy.init()
    node = BenchmarkLogger()
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
