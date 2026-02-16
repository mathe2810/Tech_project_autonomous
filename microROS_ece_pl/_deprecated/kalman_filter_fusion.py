#!/usr/bin/env python3
"""
Kalman Filter for sensor fusion.
Fuses odometry (fast, drifts) with SLAM corrections (slow, corrects drift).

Inputs:
  - /odom : Odometry from motor_odom_node (prediction)
  - /slam/pose : SLAM estimated pose (measurement/correction)
  - /imu/data_filtered : IMU for additional heading info

Outputs:
  - /odom_filtered : Fused odometry (corrected)
  - /tf : Transform odom -> base_link (corrected)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ExtendedKalmanFilter2D:
    """
    2D Extended Kalman Filter for robot pose estimation.
    State: [x, y, theta]
    """
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        """
        process_noise: How much we trust odometry (lower = more trust)
        measurement_noise: How much we trust SLAM (lower = more trust)
        """
        # State vector: [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        
        # State covariance matrix
        self.P = np.eye(3) * 0.1
        
        # Process noise matrix (odometry uncertainty)
        self.Q = np.eye(3) * process_noise
        
        # Measurement noise matrix (SLAM uncertainty)
        self.R = np.eye(3) * measurement_noise
        
    def predict(self, vx, wz, dt):
        """
        Prediction step using odometry.
        Uses simple kinematic model.
        """
        x, y, theta = self.x
        
        # Jacobian of motion model wrt state
        if abs(wz) > 0.001:
            # Turning motion
            sin_theta = math.sin(theta)
            cos_theta = math.cos(theta)
            sin_new = math.sin(theta + wz * dt)
            cos_new = math.cos(theta + wz * dt)
            
            # Update state
            r = vx / wz
            self.x[0] = x + r * (sin_new - sin_theta)
            self.x[1] = y + r * (-cos_new + cos_theta)
        else:
            # Straight line motion
            self.x[0] = x + vx * math.cos(theta) * dt
            self.x[1] = y + vx * math.sin(theta) * dt
        
        self.x[2] = theta + wz * dt
        self.x[2] = self._normalize_angle_value(self.x[2])
        
        # Compute Jacobian for covariance update
        F = self._jacobian_F(vx, wz, dt)
        
        # Update covariance: P = F*P*F^T + Q
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, z_x, z_y, z_theta):
        """
        Update step using SLAM measurement.
        z: measured state [x, y, theta]
        """
        # Measurement matrix (we measure all states)
        H = np.eye(3)
        
        # Innovation (measurement residual)
        z = np.array([z_x, z_y, z_theta])
        y = z - self.x
        
        # Normalize angle difference
        y[2] = self._normalize_angle_diff(y[2])
        
        # Innovation covariance: S = H*P*H^T + R
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain: K = P*H^T*S^-1
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state: x = x + K*y
        self.x = self.x + K @ y
        self.x[2] = self._normalize_angle_value(self.x[2])
        
        # Update covariance: P = (I - K*H)*P
        self.P = (np.eye(3) - K @ H) @ self.P
    
    def _jacobian_F(self, vx, wz, dt):
        """Jacobian of motion model"""
        F = np.eye(3)
        theta = self.x[2]
        
        if abs(wz) > 0.001:
            r = vx / wz
            dtheta = wz * dt
            sin_theta = math.sin(theta)
            cos_theta = math.cos(theta)
            sin_new = math.sin(theta + dtheta)
            cos_new = math.cos(theta + dtheta)
            
            F[0, 2] = r * (cos_new - cos_theta)
            F[1, 2] = r * (sin_new - sin_theta)
        else:
            F[0, 2] = -vx * math.sin(theta) * dt
            F[1, 2] = vx * math.cos(theta) * dt
        
        return F
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _normalize_angle_value(self, angle):
        """Normalize angle value in place"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def _normalize_angle_diff(diff):
        """Normalize angle difference to [-pi, pi]"""
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
    
    def get_state(self):
        """Get current state estimate"""
        return self.x.copy()
    
    def get_covariance(self):
        """Get current state covariance"""
        return self.P.copy()


class KalmanFilterFusionNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_fusion')
        
        # Parameters
        self.declare_parameter('process_noise', 0.01)
        self.declare_parameter('measurement_noise', 0.15)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('slam_timeout', 2.0)  # seconds
        self.declare_parameter('publish_rate', 50.0)
        
        process_noise = self.get_parameter('process_noise').value
        measurement_noise = self.get_parameter('measurement_noise').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.slam_timeout = self.get_parameter('slam_timeout').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Kalman Filter
        self.kf = ExtendedKalmanFilter2D(
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )
        
        # State
        self.vx = 0.0
        self.wz = 0.0
        self.last_slam_time = None
        self.has_slam = False
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.slam_sub = self.create_subscription(
            PoseStamped, '/slam/pose', self.slam_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_filtered', self.imu_callback, 10
        )
        
        # Publishers
        self.odom_filtered_pub = self.create_publisher(
            Odometry, '/odom_filtered', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / publish_rate, self.update_callback)
        
        # Covariance matrices for output
        self.pose_covariance = [0.0] * 36
        self.twist_covariance = [0.0] * 36
        
        self.get_logger().info(
            f'Kalman Filter Fusion node started\n'
            f'  Process noise: {process_noise}\n'
            f'  Measurement noise: {measurement_noise}\n'
            f'  SLAM timeout: {self.slam_timeout}s'
        )
    
    def odom_callback(self, msg: Odometry):
        """Receive odometry for prediction step"""
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z
    
    def slam_callback(self, msg: PoseStamped):
        """Receive SLAM pose for update step"""
        self.last_slam_time = self.get_clock().now()
        self.has_slam = True
        
        # Extract SLAM measurement
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Convert quaternion to yaw
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2 * math.atan2(qz, qw)
        
        # Kalman filter update
        self.kf.update(x, y, yaw)
        
        self.get_logger().debug(f'SLAM update: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')
    
    def imu_callback(self, msg: Imu):
        """Receive IMU (optional, for diagnostics)"""
        pass
    
    def update_callback(self):
        """Predict and publish filtered odometry"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        if dt <= 0 or dt > 1.0:
            return
        
        # Prediction step (always do this)
        self.kf.predict(self.vx, self.wz, dt)
        
        # Check if SLAM is stale
        if self.has_slam and self.last_slam_time:
            slam_age = (now - self.last_slam_time).nanoseconds / 1e9
            if slam_age > self.slam_timeout:
                self.has_slam = False
                self.get_logger().warn(f'SLAM data stale (age: {slam_age:.1f}s)')
        
        # Publish filtered odometry
        self._publish_odom_filtered(now)
        
        # Publish transform
        self._publish_tf(now)
    
    def _publish_odom_filtered(self, timestamp):
        """Publish fused odometry"""
        state = self.kf.get_state()
        cov = self.kf.get_covariance()
        
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = float(state[0])
        odom.pose.pose.position.y = float(state[1])
        odom.pose.pose.position.z = 0.0
        
        # Orientation from yaw
        yaw = state[2]
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Covariance from Kalman filter
        self._set_pose_covariance(cov, odom)
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.wz
        
        # Twist covariance (must be tuple of 36 floats)
        odom.twist.covariance = tuple(float(v) for v in [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                          0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                                          0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                                          0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                                                          0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.05])
        
        self.odom_filtered_pub.publish(odom)
    
    def _set_pose_covariance(self, kf_cov, odom):
        """Convert KF covariance to ROS format"""
        # Create 36-element tuple of floats (ROS requirement)
        cov = tuple(float(0.0) for _ in range(36))
        cov_list = list(cov)
        # Position (x, y, z)
        cov_list[0] = float(kf_cov[0, 0])  # x
        cov_list[7] = float(kf_cov[1, 1])  # y
        cov_list[14] = float(0.1)  # z (not estimated)
        cov_list[35] = float(kf_cov[2, 2])  # yaw
        odom.pose.covariance = tuple(float(v) for v in cov_list)
    
    def _publish_tf(self, timestamp):
        """Publish corrected transform"""
        state = self.kf.get_state()
        yaw = state[2]
        
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = float(state[0])
        t.transform.translation.y = float(state[1])
        t.transform.translation.z = 0.0
        
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
