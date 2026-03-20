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
import numpy as np

class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')

        self.declare_parameter('scan_yaw_offset', 0.0)
        self.scan_yaw_offset = float(self.get_parameter('scan_yaw_offset').value)
        
        self.scan_count = 0
        self.last_published_count = 0
        self.publish_interval = 4  # Publier 1 scan sur 4 (throttle maximal)
        
        # Subscribe to ESP32 /scan_raw in BEST_EFFORT
        scan_raw_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publish /scan in RELIABLE (SLAM needs RELIABLE)
        scan_reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
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
        
        self.get_logger().info(
            'Scan Restamper started - bridging /scan_raw → /scan (RELIABLE), '
            f'/scan_rf2o (BEST_EFFORT), scan_yaw_offset={self.scan_yaw_offset:.3f} rad'
        )
        self.scan_count = 0

    def scan_callback(self, msg):
        """Retimestamp scan to ROS clock - optimized"""
        if not rclpy.ok():
            return

        self.scan_count += 1
        
        # Throttle: publier juste 1 scan sur 2
        if self.scan_count % self.publish_interval != 0:
            return
        
        # Fast numpy filtering
        ranges_array = np.array(msg.ranges, dtype=np.float32)
        ranges_array[ranges_array > 4.0] = np.inf
        msg.ranges = ranges_array.tolist()

        if self.scan_yaw_offset != 0.0:
            msg.angle_min += self.scan_yaw_offset
            msg.angle_max += self.scan_yaw_offset

        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)
        self.scan_rf2o_pub.publish(msg)

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
