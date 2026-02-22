#!/usr/bin/env python3
"""
Simple Scan Restamper - Fix LIDAR timestamps
Bridges /scan_raw into:
- /scan (RELIABLE) for SLAM Toolbox / RViz
- /scan_rf2o (BEST_EFFORT) for RF2O
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        
        # Subscribe to ESP32 /scan_raw in BEST_EFFORT
        scan_raw_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publish /scan in RELIABLE so SLAM Toolbox + RViz are compatible
        scan_reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publish /scan_rf2o in BEST_EFFORT for RF2O compatibility
        scan_best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publisher QoS for SLAM/RViz
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', 
            qos_profile=scan_reliable_qos
        )

        self.scan_rf2o_pub = self.create_publisher(
            LaserScan, '/scan_rf2o',
            qos_profile=scan_best_effort_qos
        )
        
        # Subscriber QoS for ESP32 source
        self.create_subscription(
            LaserScan, '/scan_raw', 
            self.scan_callback, 
            qos_profile=scan_raw_qos
        )
        
        self.get_logger().info('Scan Restamper started - bridging /scan_raw → /scan (RELIABLE), /scan_rf2o (BEST_EFFORT)')
        self.scan_count = 0

    def scan_callback(self, msg):
        """Retimestamp scan to ROS clock"""
        if not rclpy.ok():
            return

        self.scan_count += 1
        
        # Log frame_id on first scan
        if self.scan_count == 1:
            self.get_logger().info(f'First scan - frame_id: {msg.header.frame_id}')
        
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            self.scan_pub.publish(msg)
            self.scan_rf2o_pub.publish(msg)
        except Exception:
            if not rclpy.ok():
                return
            raise
        
        if self.scan_count % 50 == 0:
            self.get_logger().info(
                f'Published {self.scan_count} scans to /scan + /scan_rf2o (frame_id={msg.header.frame_id})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
