#!/usr/bin/env python3
"""
Realistic Synthetic Battery Simulator for AMRs
Publishes sensor_msgs/msg/BatteryState based on actual movement and idle states.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        self.declare_parameter('initial_percentage', 100.0)
        self.declare_parameter('drain_rate_moving', 0.12)   # % per second at reference speed
        self.declare_parameter('drain_rate_idle', 0.015)    # % per second when stationary
        self.declare_parameter('voltage_full', 25.2)        # 6S LiPo full (4.2V/cell)
        self.declare_parameter('voltage_empty', 21.0)       # 6S LiPo cutoff (3.5V/cell)

        ns = self.get_namespace().strip('/')
        # Give diverse initial states if not overridden
        default_soc = 98.0
        if ns == 'amr2':
            default_soc = 86.0
        elif ns == 'amr3':
            default_soc = 92.0

        init_soc = self.get_parameter('initial_percentage').value
        if init_soc == 100.0 and default_soc != 100.0:
            self.battery_pct = default_soc
        else:
            self.battery_pct = float(init_soc)

        self.drain_moving = self.get_parameter('drain_rate_moving').value
        self.drain_idle = self.get_parameter('drain_rate_idle').value
        self.v_full = self.get_parameter('voltage_full').value
        self.v_empty = self.get_parameter('voltage_empty').value

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            'battery_state',
            10
        )

        self.timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f"BatterySimulator initialized for {ns} at {self.battery_pct:.1f}% SOC")

    def odom_callback(self, msg: Odometry):
        self.current_linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.current_angular_speed = abs(msg.twist.twist.angular.z)

    def timer_callback(self):
        # Calculate battery drain based on motion
        is_moving = (self.current_linear_speed > 0.02) or (self.current_angular_speed > 0.05)
        if is_moving:
            # Scale drain with movement intensity
            intensity = min(2.0, (self.current_linear_speed / 0.5) + (self.current_angular_speed / 1.0))
            delta_pct = (self.drain_moving * max(0.5, intensity)) * self.timer_period
        else:
            delta_pct = self.drain_idle * self.timer_period

        self.battery_pct = max(0.0, self.battery_pct - delta_pct)

        # Voltage mapping (linear approximation between empty and full)
        voltage = self.v_empty + (self.v_full - self.v_empty) * (self.battery_pct / 100.0)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.get_namespace().strip('/')}/base_footprint"
        msg.voltage = float(voltage)
        msg.current = -1.8 if is_moving else -0.2  # Discharge current (negative)
        msg.charge = float(self.battery_pct * 0.1) # Ah approx
        msg.capacity = 10.0
        msg.design_capacity = 10.0
        msg.percentage = float(self.battery_pct / 100.0)  # Standard ROS convention: 0.0 - 1.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
