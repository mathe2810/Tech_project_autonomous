#!/usr/bin/env python3
"""Test for rotation drift - rotate 360° and check if we return to same position"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RotationDriftTest(Node):
    def __init__(self):
        super().__init__('rotation_drift_test')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.start_pose = None
        self.current_pose = None
        self.total_rotation = 0.0
        self.last_yaw = None
        
        self.get_logger().info('🔄 Rotation Drift Test - Starting in 3 seconds...')
        time.sleep(3)
        self.run_test()
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        if self.last_yaw is not None:
            delta = yaw - self.last_yaw
            # Handle wrap-around
            if delta > math.pi:
                delta -= 2*math.pi
            elif delta < -math.pi:
                delta += 2*math.pi
            self.total_rotation += abs(delta)
        
        self.last_yaw = yaw
        
        if self.start_pose is None:
            self.start_pose = self.current_pose
    
    def run_test(self):
        """Rotate 360° at minimum motor speed and measure drift"""
        twist = Twist()
        
        # Rotate at 1.0 rad/s (~57°/s) - MINIMUM for your motors to turn
        twist.angular.z = 1.5  # Minimum speed where motors actually work
        
        self.get_logger().info('🔄 Rotating 360° at 1.5 rad/s (~100°/s) - 6.3 seconds...')
        
        # Rotate for ~6.3 seconds to complete 360° (2*pi radians)
        start_time = time.time()
        while time.time() - start_time < 6.3:
            self.pub_cmd.publish(twist)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)
        
        # Stop
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)
        time.sleep(1)
        
        # Calculate drift
        if self.start_pose and self.current_pose:
            dx = self.current_pose.position.x - self.start_pose.position.x
            dy = self.current_pose.position.y - self.start_pose.position.y
            drift_distance = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('📊 ROTATION DRIFT TEST RESULTS:')
            self.get_logger().info(f'   Total rotation: {math.degrees(self.total_rotation):.1f}°')
            self.get_logger().info(f'   Linear drift: {drift_distance*100:.2f} cm')
            self.get_logger().info(f'   Start: ({self.start_pose.position.x:.3f}, {self.start_pose.position.y:.3f})')
            self.get_logger().info(f'   End:   ({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f})')
            
            if drift_distance < 0.05:  # Less than 5cm
                self.get_logger().info('   ✅ EXCELLENT - Drift < 5cm')
            elif drift_distance < 0.15:  # Less than 15cm
                self.get_logger().info('   ✅ GOOD - Drift < 15cm')
            elif drift_distance < 0.30:  # Less than 30cm
                self.get_logger().info('   ⚠️  ACCEPTABLE - Drift < 30cm')
            else:
                self.get_logger().info('   ❌ HIGH DRIFT - Needs tuning')
            self.get_logger().info('=' * 50)

def main():
    rclpy.init()
    test = RotationDriftTest()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
