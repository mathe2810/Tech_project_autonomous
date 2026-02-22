#!/usr/bin/env python3
"""Test rotation reactivity - measure how fast SLAM corrects during rapid rotation"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math
import time

class ReactivityTest(Node):
    def __init__(self):
        super().__init__('reactivity_test')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to both /odom (RF2O) and /tf (SLAM corrections)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        
        self.odom_yaw = None
        self.slam_yaw = None
        self.last_odom_time = None
        self.last_slam_time = None
        
        self.odom_updates = []
        self.slam_updates = []
        
        self.get_logger().info('🔬 Reactivity Test - Measuring update rates during rotation')
        time.sleep(2)
        self.run_test()
    
    def odom_callback(self, msg):
        now = time.time()
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        if self.last_odom_time is not None:
            dt = now - self.last_odom_time
            self.odom_updates.append(dt)
        
        self.odom_yaw = yaw
        self.last_odom_time = now
    
    def tf_callback(self, msg):
        """Track when SLAM publishes map->odom corrections"""
        now = time.time()
        
        # Look for map->odom transform
        for transform in msg.transforms:
            if (transform.header.frame_id == 'map' and 
                transform.child_frame_id == 'odom'):
                
                if self.last_slam_time is not None:
                    dt = now - self.last_slam_time
                    self.slam_updates.append(dt)
                
                self.last_slam_time = now
                break
    
    def run_test(self):
        """Rotate rapidly and measure update frequencies"""
        twist = Twist()
        twist.angular.z = 1.5  # Fast rotation
        
        duration = 8.0  # 8 seconds of rotation
        
        self.get_logger().info(f'Rotating at 1.5 rad/s for {duration}s...')
        self.get_logger().info('Measuring RF2O and SLAM update rates...')
        
        # Clear buffers
        self.odom_updates = []
        self.slam_updates = []
        self.last_odom_time = None
        self.last_slam_time = None
        
        start = time.time()
        while time.time() - start < duration:
            self.pub_cmd.publish(twist)
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0)
        
        # Stop
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)
        time.sleep(1)
        
        # Analyze results
        self.show_results()
    
    def show_results(self):
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('📊 REACTIVITY TEST RESULTS')
        self.get_logger().info('='*60)
        
        if self.odom_updates:
            odom_avg = sum(self.odom_updates) / len(self.odom_updates)
            odom_hz = 1.0 / odom_avg if odom_avg > 0 else 0
            odom_max_delay = max(self.odom_updates) * 1000  # ms
            
            self.get_logger().info(f'RF2O (/odom):')
            self.get_logger().info(f'  Average rate: {odom_hz:.1f} Hz')
            self.get_logger().info(f'  Average period: {odom_avg*1000:.1f} ms')
            self.get_logger().info(f'  Max delay: {odom_max_delay:.1f} ms')
            self.get_logger().info(f'  Updates: {len(self.odom_updates)}')
            
            if odom_hz < 35:
                self.get_logger().info(f'  ⚠️  SLOW - Should be ~40 Hz')
            else:
                self.get_logger().info(f'  ✅ GOOD rate')
        else:
            self.get_logger().info('RF2O: ❌ No updates received')
        
        self.get_logger().info('')
        
        if self.slam_updates:
            slam_avg = sum(self.slam_updates) / len(self.slam_updates)
            slam_hz = 1.0 / slam_avg if slam_avg > 0 else 0
            slam_max_delay = max(self.slam_updates) * 1000  # ms
            
            self.get_logger().info(f'SLAM (/tf map->odom):')
            self.get_logger().info(f'  Average rate: {slam_hz:.1f} Hz')
            self.get_logger().info(f'  Average period: {slam_avg*1000:.1f} ms')
            self.get_logger().info(f'  Max delay: {slam_max_delay:.1f} ms')
            self.get_logger().info(f'  Updates: {len(self.slam_updates)}')
            
            if slam_hz < 5:
                self.get_logger().info(f'  ⚠️  SLOW - Corrections lagging behind')
            elif slam_hz < 8:
                self.get_logger().info(f'  ✅ GOOD - Should handle rotations')
            else:
                self.get_logger().info(f'  ✅ EXCELLENT - Very reactive')
        else:
            self.get_logger().info('SLAM: ❌ No map->odom corrections seen')
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('💡 RECOMMENDATIONS:')
        
        if self.odom_updates and odom_hz < 35:
            self.get_logger().info('  - Increase RF2O freq to 40-50 Hz')
        
        if self.slam_updates and slam_hz < 6:
            self.get_logger().info('  - Reduce map_update_interval (0.15s = 6.7 Hz)')
            self.get_logger().info('  - Set minimum_time_interval: 0.0 (instant)')
            self.get_logger().info('  - Lower thresholds (0.05/0.10)')
        
        if self.odom_updates and self.slam_updates:
            latency = (slam_avg - odom_avg) * 1000
            self.get_logger().info(f'  - SLAM lag behind RF2O: {latency:.1f} ms')
            if latency > 150:
                self.get_logger().info('    ⚠️  High latency - reactions too slow!')
        
        self.get_logger().info('='*60)

def main():
    rclpy.init()
    test = ReactivityTest()
    time.sleep(1)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
