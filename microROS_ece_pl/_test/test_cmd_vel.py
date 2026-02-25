#!/usr/bin/env python3
"""
Test script: Send velocity commands to test odometry and SLAM
Publishes /cmd_vel with simple movement patterns
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("CmdVel Test Publisher started - publishing movement commands")
        
    def publish_forward(self, duration=2.0, speed=0.1):
        """Publish forward movement"""
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        start = time.time()
        while time.time() - start < duration:
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        # Stop
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(f"Moved forward {speed}m/s for {duration}s")
    
    def publish_rotate(self, duration=2.0, angular_speed=0.5):
        """Publish rotation"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        
        start = time.time()
        while time.time() - start < duration:
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        # Stop
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(f"Rotated at {angular_speed}rad/s for {duration}s")
    
    def test_pattern(self):
        """Execute test pattern: forward + rotate + forward"""
        self.get_logger().info("\n=== TEST PATTERN ===")
        
        time.sleep(2)  # Wait for other nodes to be ready
        
        # Forward 2m
        self.get_logger().info("1. Moving forward...")
        self.publish_forward(duration=3.0, speed=0.15)
        time.sleep(1)
        
        # Rotate 90 degrees
        self.get_logger().info("2. Rotating...")
        self.publish_rotate(duration=2.0, angular_speed=0.5)
        time.sleep(1)
        
        # Forward again
        self.get_logger().info("3. Moving forward again...")
        self.publish_forward(duration=3.0, speed=0.15)
        time.sleep(1)
        
        # Another rotation
        self.get_logger().info("4. Rotating again...")
        self.publish_rotate(duration=2.0, angular_speed=0.5)
        
        self.get_logger().info("\n=== TEST COMPLETE ===")
        self.get_logger().info("Check /map - it should accumulate scans in L-shape pattern")

def main():
    rclpy.init()
    node = CmdVelPublisher()
    
    # Run test pattern
    node.test_pattern()
    
    # Keep node alive briefly
    time.sleep(2)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
