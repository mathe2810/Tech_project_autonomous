#!/usr/bin/env python3
"""Test script pour vérifier que RF2O reçoit les données du LIDAR simulé"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RFO2TestNode(Node):
    def __init__(self):
        super().__init__('rf2o_test_listener')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_raw_count = 0
        self.scan_count = 0
        self.odom_count = 0
        
        # Écoute les trois topics clés
        self.create_subscription(LaserScan, '/scan_raw', self.scan_raw_callback, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        
        self.get_logger().info('🔍 Démarrage du test - En écoute sur /scan_raw, /scan et /odom')

    def scan_raw_callback(self, msg):
        self.scan_raw_count += 1
        if self.scan_raw_count % 30 == 0:
            self.get_logger().info(f'✅ /scan_raw: {self.scan_raw_count} scans, {len(msg.ranges)} rayons')

    def scan_callback(self, msg):
        self.scan_count += 1
        if self.scan_count % 30 == 0:
            self.get_logger().info(f'✅ /scan: {self.scan_count} scans, angle_min={msg.angle_min:.4f}, max={msg.angle_max:.4f}')

    def odom_callback(self, msg):
        self.odom_count += 1
        if self.odom_count % 30 == 0:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            self.get_logger().info(f'✅ /odom: {self.odom_count} messages, position=({x:.2f}, {y:.2f})')

def main():
    rclpy.init()
    node = RFO2TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
