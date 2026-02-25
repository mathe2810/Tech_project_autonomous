#!/usr/bin/env python3
"""
Test movement node - sends /cmd_vel commands to make the robot move.
"""

import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('test_movement')
    
    # Create publisher for cmd_vel
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    node.get_logger().info("Test movement started. Sending commands to /cmd_vel...")
    
    # Wait a bit for subscribers to connect
    time.sleep(1)
    
    try:
        # Forward movement
        print("\n[1] Moving FORWARD for 3 seconds...")
        twist = Twist()
        twist.linear.x = 0.2  # 20 cm/s forward
        twist.angular.z = 0.0
        start = time.time()
        while time.time() - start < 3:
            cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        # Stop
        print("[2] STOPPING for 1 second...")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        start = time.time()
        while time.time() - start < 1:
            cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        # Rotate
        print("[3] ROTATING for 3 seconds...")
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # 0.5 rad/s rotation
        start = time.time()
        while time.time() - start < 3:
            cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        # Stop
        print("[4] STOPPING...")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(10):
            cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        print("\n✓ Test complete!")
        
    except KeyboardInterrupt:
        print("\nStopping...")
        twist = Twist()
        cmd_vel_pub.publish(twist)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
