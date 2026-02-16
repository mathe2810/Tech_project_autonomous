#!/usr/bin/env python3
"""
SLAM Stabilizer - Monitors SLAM covariance and forces pose reset on drift.
Detects large deviations and triggers calibration routines.

Monitors:
- /slam_toolbox/feedback for pose covariance
- /odom for odometry covariance
- Detects drift when covariances exceed thresholds
- Triggers automatic calibration move

Usage: python3 slam_stabilizer.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty
import math
import time

class SLAMStabilizer(Node):
    def __init__(self):
        super().__init__('slam_stabilizer')
        
        # Thresholds
        self.cov_pos_threshold = 0.5      # Position covariance threshold (m²)
        self.cov_yaw_threshold = 0.3      # Yaw covariance threshold (rad²)
        self.drift_detection_time = 10.0  # seconds to declare drift
        
        # State
        self.last_good_covariance_time = time.time()
        self.in_calibration = False
        self.slam_health = "GOOD"
        
        # Subscriptions
        self.slam_feedback_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_toolbox/pose',
            self.slam_feedback_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Clients for SLAM services (if available)
        self.clear_queue_client = self.create_client(Empty, '/slam_toolbox/clear_queue')
        
        # Timer for monitoring
        self.create_timer(1.0, self.monitor_callback)
        
        self.get_logger().info("SLAM Stabilizer started - monitoring covariance")
    
    def slam_feedback_callback(self, msg: PoseWithCovarianceStamped):
        """Monitor SLAM pose covariance"""
        # Extract covariances
        cov = msg.pose.covariance
        
        # Position covariance (x, y) - indices 0,7 (diagonal of 6x6 matrix)
        cov_x = cov[0]
        cov_y = cov[7]
        
        # Yaw covariance - index 35 (last diagonal element)
        cov_yaw = cov[35]
        
        # Check if covariances are reasonable
        max_pos_cov = max(cov_x, cov_y)
        
        if max_pos_cov < self.cov_pos_threshold and cov_yaw < self.cov_yaw_threshold:
            # Good covariance - update health
            self.last_good_covariance_time = time.time()
            if self.slam_health != "GOOD":
                self.slam_health = "GOOD"
                self.get_logger().info("✓ SLAM stabilized - covariance acceptable")
        else:
            # High covariance - drift detected
            self.slam_health = "DRIFTING"
            self.get_logger().warn(
                f"⚠ HIGH COVARIANCE: pos_x={cov_x:.3f}, pos_y={cov_y:.3f}, yaw={cov_yaw:.3f}"
            )
    
    def monitor_callback(self):
        """Check SLAM health and trigger calibration if needed"""
        time_since_good = time.time() - self.last_good_covariance_time
        
        if time_since_good > self.drift_detection_time and not self.in_calibration:
            self.get_logger().error(
                f"🔴 SLAM DRIFT DETECTED (no good covariance for {time_since_good:.1f}s)"
            )
            self.trigger_calibration()
    
    def trigger_calibration(self):
        """Trigger automatic calibration routine"""
        if self.in_calibration:
            return
        
        self.in_calibration = True
        self.get_logger().info("🔄 Starting automatic calibration...")
        
        try:
            # Stop movement
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.5)
            
            # Perform small oscillation to help SLAM converge
            self.get_logger().info("   - Small forward/backward oscillation...")
            for cycle in range(3):
                # Forward
                msg.linear.x = 0.05  # 5cm/s
                for _ in range(10):
                    self.cmd_vel_pub.publish(msg)
                    time.sleep(0.1)
                
                # Backward
                msg.linear.x = -0.05
                for _ in range(10):
                    self.cmd_vel_pub.publish(msg)
                    time.sleep(0.1)
            
            # Stop
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)
            
            # Clear SLAM queue to force reprocessing
            self.get_logger().info("   - Clearing SLAM queue...")
            if self.clear_queue_client.service_is_ready():
                future = self.clear_queue_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            self.get_logger().info("✓ Calibration complete - resuming normal operation")
            self.last_good_covariance_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Calibration error: {e}")
        finally:
            self.in_calibration = False
    
    def slow_circle_calibration(self):
        """Alternative: Small slow circle for better SLAM convergence"""
        self.get_logger().info("   - Small slow circle (radius 0.5m)...")
        
        radius = 0.5  # 50cm radius
        forward_speed = 0.05  # 5cm/s
        angular_speed = forward_speed / radius  # ~0.1 rad/s
        
        msg = Twist()
        msg.linear.x = forward_speed
        msg.angular.z = angular_speed
        
        # Circle takes ~60 seconds at this speed
        duration = 60  # seconds
        start = time.time()
        
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
