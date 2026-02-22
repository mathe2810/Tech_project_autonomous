#!/usr/bin/env python3
"""Test for linear drift - move forward 2m and check accuracy"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class LinearDriftTest(Node):
    def __init__(self):
        super().__init__('linear_drift_test')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.start_pose = None
        self.current_pose = None
        self.total_distance = 0.0
        self.last_pose = None
        
        self.get_logger().info('➡️  Linear Drift Test - Starting in 3 seconds...')
        time.sleep(3)
        self.run_test()
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
        if self.last_pose is not None:
            dx = self.current_pose.position.x - self.last_pose.position.x
            dy = self.current_pose.position.y - self.last_pose.position.y
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        
        self.last_pose = self.current_pose
        
        if self.start_pose is None:
            self.start_pose = self.current_pose
    
    def run_test(self):
        """Move forward 1m at realistic speed and measure drift"""
        twist = Twist()
        twist.linear.x = 0.12  # 12 cm/s - conservative but realistic
        
        target_distance = 1.0  # 1 meter
        duration = target_distance / 0.12  # ~8.3 seconds
        
        self.get_logger().info(f'➡️  Moving forward {target_distance}m at 0.12 m/s...')
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub_cmd.publish(twist)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)
        
        # Stop
        twist.linear.x = 0.0
        self.pub_cmd.publish(twist)
        time.sleep(1)
        
        # Calculate accuracy
        if self.start_pose and self.current_pose:
            dx = self.current_pose.position.x - self.start_pose.position.x
            dy = self.current_pose.position.y - self.start_pose.position.y
            actual_distance = math.sqrt(dx*dx + dy*dy)
            
            # Check lateral drift (perpendicular to motion)
            yaw_start = math.atan2(dy, dx)
            lateral_drift = abs(math.sin(yaw_start) * actual_distance)
            
            error = actual_distance - target_distance
            error_percent = (error / target_distance) * 100
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('📊 LINEAR DRIFT TEST RESULTS:')
            self.get_logger().info(f'   Target distance: {target_distance:.2f} m')
            self.get_logger().info(f'   Actual distance: {actual_distance:.3f} m')
            self.get_logger().info(f'   Error: {error*100:.1f} cm ({error_percent:.1f}%)')
            self.get_logger().info(f'   Lateral drift: {lateral_drift*100:.1f} cm')
            self.get_logger().info(f'   Odometry total: {self.total_distance:.3f} m')
            
            if abs(error_percent) < 3:
                self.get_logger().info('   ✅ EXCELLENT - Error < 3%')
            elif abs(error_percent) < 7:
                self.get_logger().info('   ✅ GOOD - Error < 7%')
            elif abs(error_percent) < 15:
                self.get_logger().info('   ⚠️  ACCEPTABLE - Error < 15%')
            else:
                self.get_logger().info('   ❌ HIGH ERROR - Needs tuning')
            
            if lateral_drift < 0.05:
                self.get_logger().info('   ✅ Lateral drift < 5cm - Very straight')
            elif lateral_drift < 0.15:
                self.get_logger().info('   ⚠️  Lateral drift < 15cm - Some deviation')
            else:
                self.get_logger().info('   ❌ High lateral drift - Check alignment')
            
            self.get_logger().info('=' * 50)

def main():
    rclpy.init()
    test = LinearDriftTest()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
