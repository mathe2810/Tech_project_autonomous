#!/usr/bin/env python3
"""Find the maximum speeds your motors can handle"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotorLimitTest(Node):
    def __init__(self):
        super().__init__('motor_limit_test')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('🔧 Motor Limit Test - Finding max speeds')
        self.get_logger().info('   Watch your robot and note when motors struggle')
        time.sleep(2)
        
        self.test_linear_speed()
        time.sleep(3)
        self.test_angular_speed()
        
    def test_linear_speed(self):
        """Test linear speeds from 0.05 to 0.30 m/s"""
        self.get_logger().info('\n=== LINEAR SPEED TEST ===')
        speeds = [0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30]
        
        twist = Twist()
        for speed in speeds:
            twist.linear.x = speed
            self.get_logger().info(f'  Testing {speed:.2f} m/s ({speed*100:.0f} cm/s)...')
            
            for _ in range(20):  # 2 seconds
                self.pub_cmd.publish(twist)
                time.sleep(0.1)
            
            # Stop
            twist.linear.x = 0.0
            self.pub_cmd.publish(twist)
            time.sleep(1)
        
        self.get_logger().info('✅ Linear test done!')
        self.get_logger().info('   💡 Use the highest speed where wheels turn smoothly')
    
    def test_angular_speed(self):
        """Test rotation speeds from 0.1 to 0.8 rad/s"""
        self.get_logger().info('\n=== ANGULAR SPEED TEST ===')
        # Convert to degrees for readability
        speeds_rad = [0.1, 0.15, 0.20, 0.26, 0.35, 0.52, 0.70]
        
        twist = Twist()
        for speed in speeds_rad:
            speed_deg = int(speed * 57.3)  # Convert to degrees/s
            twist.angular.z = speed
            self.get_logger().info(f'  Testing {speed:.2f} rad/s (~{speed_deg}°/s)...')
            
            for _ in range(30):  # 3 seconds
                self.pub_cmd.publish(twist)
                time.sleep(0.1)
            
            # Stop
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            time.sleep(1)
        
        self.get_logger().info('✅ Angular test done!')
        self.get_logger().info('   💡 Use the highest speed where robot rotates smoothly')
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('📊 RECOMMENDATIONS:')
        self.get_logger().info('   - If motors struggle above 0.10 m/s linear:')
        self.get_logger().info('     Use max_linear_speed: 0.10 in rf2o_params.yaml')
        self.get_logger().info('   - If motors struggle above 0.26 rad/s (~15°/s):')
        self.get_logger().info('     Use max_angular_speed: 0.8 in rf2o_params.yaml')
        self.get_logger().info('='*50)

def main():
    rclpy.init()
    test = MotorLimitTest()
    time.sleep(2)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
