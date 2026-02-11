#!/usr/bin/env python3
"""
Robot Localization EKF - fuses odometry and IMU to produce corrected odometry.
Much simpler than the custom Kalman filter.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class SimpleEKFNode(Node):
    def __init__(self):
        super().__init__('simple_ekf')
        
        # State: [x, y, theta, vx, wz]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.covariance = np.eye(5) * 0.1
        
        # Noise params
        self.process_noise = 0.001
        self.odom_noise = 0.01
        self.imu_noise = 0.005
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_filtered', self.imu_callback, 10
        )
        
        # Publishers
        self.filtered_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timing
        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.publish_callback)  # 50 Hz
        
        self.get_logger().info("Simple EKF started")
    
    def odom_callback(self, msg: Odometry):
        """Process odometry update"""
        # Extract velocity
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        # Update velocity state
        self.state[3] = vx
        self.state[4] = wz
    
    def imu_callback(self, msg: Imu):
        """Process IMU update (mainly for heading)"""
        # Extract yaw rate from gyroscope
        wz = msg.angular_velocity.z
        self.state[4] = 0.8 * self.state[4] + 0.2 * wz  # Low pass filter
    
    def predict(self, dt):
        """Prediction step - integrate velocity"""
        x, y, theta, vx, wz = self.state
        
        # Simple kinematic model
        if abs(wz) > 0.001:
            # Curved motion
            r = vx / wz
            sin_theta = math.sin(theta)
            cos_theta = math.cos(theta)
            sin_new = math.sin(theta + wz * dt)
            cos_new = math.cos(theta + wz * dt)
            
            self.state[0] = x + r * (sin_new - sin_theta)
            self.state[1] = y + r * (-cos_new + cos_theta)
        else:
            # Straight motion
            self.state[0] = x + vx * math.cos(theta) * dt
            self.state[1] = y + vx * math.sin(theta) * dt
        
        self.state[2] = theta + wz * dt
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Update covariance (simplified)
        self.covariance *= (1.0 + self.process_noise * dt)
    
    def publish_callback(self):
        """Publish fused odometry and TF"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        if dt > 0.01:  # Update only every 10ms
            self.predict(min(dt, 0.1))
            
            x, y, theta, vx, wz = self.state
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            odom_msg.pose.pose.position.x = float(x)
            odom_msg.pose.pose.position.y = float(y)
            odom_msg.pose.pose.position.z = 0.0
            
            # Quaternion from yaw
            cy = math.cos(theta * 0.5)
            sy = math.sin(theta * 0.5)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = float(sy)
            odom_msg.pose.pose.orientation.w = float(cy)
            
            odom_msg.twist.twist.linear.x = float(vx)
            odom_msg.twist.twist.angular.z = float(wz)
            
            # Covariance (6x6 for pose: x,y,z,roll,pitch,yaw)
            cov = np.eye(6) * 0.01
            odom_msg.pose.covariance = cov.flatten().tolist()
            
            self.filtered_pub.publish(odom_msg)
            
            # Publish TF
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = float(sy)
            t.transform.rotation.w = float(cy)
            
            self.tf_broadcaster.sendTransform(t)
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main():
    rclpy.init()
    node = SimpleEKFNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
