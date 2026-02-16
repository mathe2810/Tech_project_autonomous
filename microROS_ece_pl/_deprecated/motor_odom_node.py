#!/usr/bin/env python3
"""
Motor-based Odometry without encoders.
100% LIDAR mode: IMU is NOT used for heading, only motor commands.
SLAM will correct the heading via scan matching.

Input:
  - /cmd_vel : Twist commands (linear.x, angular.z)

Output:
  - /odom : Odometry (PoseWithCovariance + TwistWithCovariance)
  - /tf : Transform odom -> base_link
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class MotorOdomNode(Node):
    def __init__(self):
        super().__init__('motor_odom')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels (meters)
        self.declare_parameter('publish_rate', 50.0)  # 50 Hz
        
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_base = self.get_parameter('wheel_base').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.imu_wz = 0.0  # Angular velocity from IMU
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing
        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / publish_rate, self.update_callback)
        
        # Covariance matrices
        # Position covariance (x, y, z, roll, pitch, yaw)
        self.pose_covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # y
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05  # yaw
        ]
        
        # Twist covariance (linear vel x, y, z, angular vel x, y, z)
        self.twist_covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # vx
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # vy
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # vz
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,  # wx
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,  # wy
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05   # wz
        ]
        
        self.get_logger().info(
            f'Motor Odometry node started\n'
            f'  Frame: {self.odom_frame} -> {self.base_frame}\n'
            f'  Mode: 100% Motor Odometry (NO IMU)\n'
            f'  Wheel base: {self.wheel_base}m'
        )
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands from motor controller"""
        self.vx = msg.linear.x
        self.wz = msg.angular.z
    
    def update_callback(self):
        """Integrate motion and publish odometry"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        if dt <= 0 or dt > 1.0:  # Ignore first call or large jumps
            return
        
        # Integrate pose (simple kinematic model - NO IMU, just motor commands)
        # For differential drive: x, y updates
        if abs(self.wz) > 0.001:  # Turning
            # Curved path using bicycle model
            radius = self.vx / self.wz if abs(self.wz) > 0.001 else float('inf')
            self.x += radius * (math.sin(self.yaw + self.wz * dt) - math.sin(self.yaw))
            self.y += radius * (-math.cos(self.yaw + self.wz * dt) + math.cos(self.yaw))
        else:  # Straight line
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt
        
        # Integrate yaw (motor commands ONLY - SLAM will correct)
        self.yaw += self.wz * dt
        self.yaw = self._normalize_angle(self.yaw)
        
        # Publish odometry
        self._publish_odom(now)
        
        # Publish transform
        self._publish_tf(now)
    def _publish_odom(self, timestamp):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Pose covariance
        odom.pose.covariance = self.pose_covariance
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.wz  # Motor command only
        
        # Twist covariance
        odom.twist.covariance = self.twist_covariance
        
        self.odom_pub.publish(odom)
    
    def _publish_tf(self, timestamp):
        """Publish TF transform odom -> base_link"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Rotation (quaternion)
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MotorOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
