#!/usr/bin/env python3
"""
Diagnostic tool: Monitor SLAM mapping quality during movement
Shows:
  1. Odometry vs SLAM pose (to detect divergence)
  2. Covariance values (to see if SLAM is confident)
  3. LIDAR-to-map alignment quality
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class DiagnosticMonitor(Node):
    def __init__(self):
        super().__init__('diagnostic_monitor')
        
        self.odom_pose = None
        self.slam_pose = None
        self.odom_cov = None
        self.slam_cov = None
        self.cmd_vel = None
        self.scan_count = 0
        
        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odom_filtered', self.odom_filt_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 1)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        
        # Timer for periodic status
        self.create_timer(1.0, self.print_status)
        
        self.get_logger().info("Diagnostic Monitor started")
    
    def odom_cb(self, msg: Odometry):
        self.odom_pose = msg.pose.pose
        self.odom_cov = msg.pose.covariance[35]  # yaw covariance [5,5]
    
    def odom_filt_cb(self, msg: Odometry):
        self.slam_pose = msg.pose.pose
        self.slam_cov = msg.pose.covariance[35]  # yaw covariance [5,5]
    
    def scan_cb(self, msg: LaserScan):
        self.scan_count += 1
    
    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel = msg
    
    def print_status(self):
        if self.odom_pose is None or self.slam_pose is None:
            self.get_logger().warn("Waiting for data...")
            return
        
        odom_yaw = math.atan2(
            2 * (self.odom_pose.orientation.w * self.odom_pose.orientation.z),
            1 - 2 * (self.odom_pose.orientation.z ** 2)
        )
        
        slam_yaw = math.atan2(
            2 * (self.slam_pose.orientation.w * self.slam_pose.orientation.z),
            1 - 2 * (self.slam_pose.orientation.z ** 2)
        )
        
        yaw_diff = abs(math.atan2(math.sin(odom_yaw - slam_yaw), math.cos(odom_yaw - slam_yaw)))
        pos_diff = math.sqrt(
            (self.odom_pose.position.x - self.slam_pose.position.x) ** 2 +
            (self.odom_pose.position.y - self.slam_pose.position.y) ** 2
        )
        
        cmd_v = self.cmd_vel.linear.x if self.cmd_vel else 0.0
        cmd_w = self.cmd_vel.angular.z if self.cmd_vel else 0.0
        
        status = "IDLE"
        if abs(cmd_v) > 0.01:
            status = "MOVING"
        if abs(cmd_w) > 0.1:
            status = "ROTATING"
        
        print(f"""
┌─ DIAGNOSTIC STATUS {'[' + status + ']':>20}
│
│  Command:
│    Linear:  {cmd_v:+.2f} m/s
│    Angular: {cmd_w:+.2f} rad/s ({cmd_w*180/math.pi:+.1f}°/s)
│
│  Odometry (motor_odom):
│    Position: ({self.odom_pose.position.x:.3f}, {self.odom_pose.position.y:.3f})
│    Yaw:      {odom_yaw*180/math.pi:+.1f}°
│    Yaw Covariance: {self.odom_cov:.3f}  {'✓ HIGH (good!)' if self.odom_cov > 0.5 else '⚠️ LOW (bad!)'} 
│
│  Filtered (EKF):
│    Position: ({self.slam_pose.position.x:.3f}, {self.slam_pose.position.y:.3f})
│    Yaw:      {slam_yaw*180/math.pi:+.1f}°
│    Yaw Covariance: {self.slam_cov:.3f}
│
│  Divergence:
│    Position Error: {pos_diff:.3f}m  {'✓ OK' if pos_diff < 0.1 else '⚠️ DIVERGING'}
│    Yaw Error:      {yaw_diff*180/math.pi:.1f}°  {'✓ OK' if yaw_diff < 15*math.pi/180 else '⚠️ DIVERGING'}
│
│  LIDAR:
│    Scans received: {self.scan_count}
│
└────────────────────────────────────────────
        """)


def main():
    rclpy.init()
    monitor = DiagnosticMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
