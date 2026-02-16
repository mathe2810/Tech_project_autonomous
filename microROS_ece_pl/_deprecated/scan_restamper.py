#!/usr/bin/env python3
"""
Restamp LaserScan messages with current ROS2 time.
Fixes timestamp mismatch when LIDAR time is out of sync with ROS2 clock.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        
        self.get_logger().info('ScanRestamper started: /scan_raw → /scan (restamped)')
    
    def scan_callback(self, msg):
        # Copy the message
        restamped = LaserScan()
        restamped.header.frame_id = msg.header.frame_id
        restamped.header.stamp = self.get_clock().now().to_msg()  # Use current ROS2 time!
        
        restamped.angle_min = msg.angle_min
        restamped.angle_max = msg.angle_max
        restamped.angle_increment = msg.angle_increment
        restamped.time_increment = msg.time_increment
        restamped.scan_time = msg.scan_time
        restamped.range_min = msg.range_min
        restamped.range_max = msg.range_max
        restamped.ranges = msg.ranges
        restamped.intensities = msg.intensities
        
        self.publisher.publish(restamped)


def main():
    rclpy.init()
    node = ScanRestamper()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
