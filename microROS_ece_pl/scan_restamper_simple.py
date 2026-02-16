#!/usr/bin/env python3
"""
Simple Scan Restamper - Fix LIDAR timestamps
Converts QoS from BEST_EFFORT (ESP32) to RELIABLE (RF2O)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        
        # Match RF2O's EXACT QoS profile:
        # KeepLast(1) + BEST_EFFORT + VOLATILE
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher with matching QoS
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', 
            qos_profile=qos_profile
        )
        
        # Subscriber with matching QoS
        self.create_subscription(
            LaserScan, '/scan_raw', 
            self.scan_callback, 
            qos_profile=qos_profile
        )
        
        self.get_logger().info('Scan Restamper started - bridging /scan_raw → /scan')
        self.scan_count = 0

    def scan_callback(self, msg):
        """Retimestamp scan to ROS clock"""
        self.scan_count += 1
        
        # Log frame_id on first scan
        if self.scan_count == 1:
            self.get_logger().info(f'First scan - frame_id: {msg.header.frame_id}')
        
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)
        
        if self.scan_count % 50 == 0:
            self.get_logger().info(f'Published {self.scan_count} scans to /scan (frame_id={msg.header.frame_id})')

def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
