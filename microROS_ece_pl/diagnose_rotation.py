#!/usr/bin/env python3
"""Diagnose rotation issue - compare commanded vs measured rotation"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RotationDiagnostic(Node):
    def __init__(self):
        super().__init__('rotation_diagnostic')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.commanded_rotation = 0.0
        self.measured_rotation = 0.0
        self.last_yaw = None
        self.start_time = None
        
        self.get_logger().info('🔬 Rotation Diagnostic - Measuring commanded vs actual')
        time.sleep(2)
        self.run_diagnostic()
    
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
    
    def run_diagnostic(self):
        """Rotate at known speed and compare commanded vs measured"""
        twist = Twist()
        
        angular_speed = 1.0  # rad/s
        duration = 10.0      # seconds
        expected_rotation = angular_speed * duration  # 10 radians = ~572°
        
        self.get_logger().info(f'Commanding {angular_speed} rad/s for {duration}s')
        self.get_logger().info(f'Expected rotation: {math.degrees(expected_rotation):.1f}°')
        
        twist.angular.z = angular_speed
        self.commanded_rotation = 0.0
        self.measured_rotation = 0.0
        
        start = time.time()
        while time.time() - start < duration:
            self.pub_cmd.publish(twist)
            self.commanded_rotation += angular_speed * 0.1
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0)
        
        # Stop
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)
        time.sleep(1)
        
        # Results
        commanded_deg = math.degrees(self.commanded_rotation)
        measured_deg = math.degrees(self.measured_rotation)
        ratio = (measured_deg / commanded_deg) * 100 if commanded_deg > 0 else 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('📊 ROTATION DIAGNOSTIC RESULTS:')
        self.get_logger().info(f'  Commanded: {commanded_deg:.1f}°')
        self.get_logger().info(f'  Measured:  {measured_deg:.1f}°')
        self.get_logger().info(f'  Ratio:     {ratio:.1f}%')
        self.get_logger().info('')
        
        if ratio < 40:
            self.get_logger().info('  ❌ CRITICAL: Odometry measures < 40% of real rotation')
            self.get_logger().info('     → Problem in ESP32 motor odometry publication')
            self.get_logger().info('     → Check wheel diameter, encoder counts, or motor driver')
            self.get_logger().info('     → Expected: /scan_raw should show robot rotating in place')
        elif ratio < 70:
            self.get_logger().info('  ⚠️  HIGH SLIPPAGE: 30-60% rotation loss')
            self.get_logger().info('     → Wheels slipping during rotation')
            self.get_logger().info('     → Try slower rotations or better surface')
            self.get_logger().info('     → SLAM can compensate but quality will suffer')
        elif ratio < 90:
            self.get_logger().info('  ⚠️  MODERATE: 10-30% rotation loss')
            self.get_logger().info('     → Acceptable with SLAM correction')
            self.get_logger().info('     → Keep coarse_search_angle_offset high (±50°)')
        else:
            self.get_logger().info('  ✅ GOOD: < 10% rotation error')
            self.get_logger().info('     → Odometry is accurate')
        
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('💡 Next steps:')
        if ratio < 40:
            self.get_logger().info('   1. Check ESP32 motor controller configuration')
            self.get_logger().info('   2. Verify wheel diameter in firmware')
            self.get_logger().info('   3. Check encoder resolution')
        else:
            self.get_logger().info('   1. Rely on SLAM scan matching (already configured)')
            self.get_logger().info('   2. Avoid fast rotations when possible')
            self.get_logger().info('   3. Test on different surface (less slippage)')

def main():
    rclpy.init()
    diag = RotationDiagnostic()
    time.sleep(1)
    diag.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
