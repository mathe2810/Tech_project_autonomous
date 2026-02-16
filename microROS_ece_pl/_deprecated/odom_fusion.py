#!/usr/bin/env python3
"""
Odometry Fusion Node
Merges rf2o (LIDAR-based) + motor odometry (dead reckoning)
Publishes fused /odom for SLAM
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import numpy as np

class OdomFusion(Node):
    def __init__(self):
        super().__init__('odom_fusion')
        
        # Parameters
        self.declare_parameter('rf2o_weight', 0.8)  # rf2o is more reliable
        self.declare_parameter('motor_weight', 0.2)  # motor backup
        
        self.rf2o_weight = self.get_parameter('rf2o_weight').value
        self.motor_weight = self.get_parameter('motor_weight').value
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        
        # Subscribers
        self.rf2o_sub = self.create_subscription(Odometry, '/odom_rf2o', self.rf2o_callback, qos)
        self.motor_sub = self.create_subscription(Odometry, '/odom_motor', self.motor_callback, qos)
        
        # State
        self.rf2o_odom = None
        self.motor_odom = None
        
        # Timer
        self.create_timer(0.05, self.fusion_callback)  # 20Hz
        
        self.get_logger().info(f'Odometry Fusion started: rf2o={self.rf2o_weight}, motor={self.motor_weight}')
    
    def rf2o_callback(self, msg):
        self.rf2o_odom = msg
    
    def motor_callback(self, msg):
        self.motor_odom = msg
    
    def fusion_callback(self):
        """Fuse odometries"""
        if self.rf2o_odom is None or self.motor_odom is None:
            return
        
        # Weighted fusion
        fused = Odometry()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.header.frame_id = 'odom'
        fused.child_frame_id = 'base_link'
        
        # Position: RF2O primary (LIDAR-based is accurate), motor as backup
        fused.pose.pose.position.x = (
            self.rf2o_weight * self.rf2o_odom.pose.pose.position.x +
            self.motor_weight * self.motor_odom.pose.pose.position.x
        )
        fused.pose.pose.position.y = (
            self.rf2o_weight * self.rf2o_odom.pose.pose.position.y +
            self.motor_weight * self.motor_odom.pose.pose.position.y
        )
        
        # Orientation: Take from rf2o (more accurate for heading)
        fused.pose.pose.orientation = self.rf2o_odom.pose.pose.orientation
        
        # Velocity: Average
        fused.twist.twist.linear.x = (
            self.rf2o_weight * self.rf2o_odom.twist.twist.linear.x +
            self.motor_weight * self.motor_odom.twist.twist.linear.x
        )
        fused.twist.twist.angular.z = (
            self.rf2o_weight * self.rf2o_odom.twist.twist.angular.z +
            self.motor_weight * self.motor_odom.twist.twist.angular.z
        )
        
        # Covariance: Take rf2o's (it's computed from scan matching quality)
        fused.pose.covariance = self.rf2o_odom.pose.covariance
        fused.twist.covariance = self.rf2o_odom.twist.covariance
        
        self.odom_pub.publish(fused)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
