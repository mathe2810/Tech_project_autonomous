#!/usr/bin/env python3
"""Find optimal angular speed - test multiple speeds and measure accuracy"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class OptimalSpeedFinder(Node):
    def __init__(self):
        super().__init__('optimal_speed_finder')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.measured_rotation = 0.0
        self.last_yaw = None
        
        self.get_logger().info('🔬 Finding Optimal Angular Speed')
        self.get_logger().info('   Testing 1.0, 1.2, 1.5, 1.8, 2.0 rad/s')
        time.sleep(2)
        
        self.results = []
        self.test_all_speeds()
        self.show_results()
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        if self.last_yaw is not None:
            delta = yaw - self.last_yaw
            if delta > math.pi:
                delta -= 2*math.pi
            elif delta < -math.pi:
                delta += 2*math.pi
            self.measured_rotation += abs(delta)
        
        self.last_yaw = yaw
    
    def test_speed(self, angular_speed):
        """Test one angular speed - rotate for 4 seconds"""
        self.measured_rotation = 0.0
        self.last_yaw = None
        
        twist = Twist()
        twist.angular.z = angular_speed
        
        duration = 4.0  # seconds
        expected = angular_speed * duration  # radians
        
        self.get_logger().info(f'  Testing {angular_speed:.1f} rad/s (~{int(angular_speed*57.3)}°/s)...')
        
        start = time.time()
        while time.time() - start < duration:
            self.pub_cmd.publish(twist)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)
        
        # Stop
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)
        time.sleep(2)  # Settle time
        
        accuracy = (self.measured_rotation / expected) * 100 if expected > 0 else 0
        error = abs(accuracy - 100.0)
        
        return {
            'speed': angular_speed,
            'expected_deg': math.degrees(expected),
            'measured_deg': math.degrees(self.measured_rotation),
            'accuracy': accuracy,
            'error': error
        }
    
    def test_all_speeds(self):
        speeds = [1.0, 1.2, 1.5, 1.8, 2.0]
        
        for speed in speeds:
            result = self.test_speed(speed)
            self.results.append(result)
            time.sleep(1)
    
    def show_results(self):
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('📊 OPTIMAL ANGULAR SPEED TEST RESULTS')
        self.get_logger().info('='*70)
        self.get_logger().info(f'{"Speed":>8} | {"Expected":>10} | {"Measured":>10} | {"Accuracy":>9} | {"Error":>7}')
        self.get_logger().info('-'*70)
        
        best_result = None
        min_error = float('inf')
        
        for r in self.results:
            self.get_logger().info(
                f'{r["speed"]:>6.1f} r/s | '
                f'{r["expected_deg"]:>8.1f}° | '
                f'{r["measured_deg"]:>8.1f}° | '
                f'{r["accuracy"]:>7.1f}% | '
                f'{r["error"]:>6.1f}%'
            )
            
            if r['error'] < min_error:
                min_error = r['error']
                best_result = r
        
        self.get_logger().info('='*70)
        self.get_logger().info('')
        self.get_logger().info('🎯 RECOMMENDATION:')
        
        if best_result:
            self.get_logger().info(f'   Best speed: {best_result["speed"]:.1f} rad/s (~{int(best_result["speed"]*57.3)}°/s)')
            self.get_logger().info(f'   Accuracy: {best_result["accuracy"]:.1f}% (error: {best_result["error"]:.1f}%)')
            
            if best_result['error'] < 15:
                self.get_logger().info('   ✅ EXCELLENT accuracy')
            elif best_result['error'] < 30:
                self.get_logger().info('   ✅ GOOD accuracy')
            else:
                self.get_logger().info('   ⚠️  Acceptable but high error')
            
            self.get_logger().info('')
            self.get_logger().info('💡 Update rf2o_params.yaml:')
            self.get_logger().info(f'   max_angular_speed: {min(best_result["speed"]*1.5, 2.5):.1f}')
            self.get_logger().info('')
            self.get_logger().info('💡 Use in teleop/tests:')
            self.get_logger().info(f'   self.angular_speed = {best_result["speed"]:.1f}')
        
        self.get_logger().info('='*70)

def main():
    rclpy.init()
    finder = OptimalSpeedFinder()
    time.sleep(1)
    finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
