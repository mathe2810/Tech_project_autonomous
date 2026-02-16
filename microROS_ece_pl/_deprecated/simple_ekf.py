#!/usr/bin/env python3
"""
Simple Odometry Relay - NO FUSION
Just re-publishes /odom as /odom_filtered for compatibility.
100% LIDAR mode: All heading correction comes from SLAM scan matching.
IMU is NOT used.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class SimpleOdomRelayNode(Node):
    def __init__(self):
        super().__init__('simple_ekf')
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Publishers
        self.filtered_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.create_timer(0.05, self.publish_callback)  # 20 Hz
        
        self.get_logger().info("Odometry Relay (NO IMU fusion) started - 100% LIDAR mode")
    
    def odom_callback(self, msg: Odometry):
        """Just relay the odometry directly"""
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
        # Extract velocity
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z
    
    def publish_callback(self):
        """Publish the odometry message"""
        now = self.get_clock().now()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (from yaw)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        
        # Velocity
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        
        # Covariance (HIGH = don't trust me, let SLAM correct)
        # 6x6 matrix: [x, y, z, roll, pitch, yaw]
        msg.pose.covariance = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # x (HIGH - no wheel encoders)
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,      # y (HIGH)
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,      # z (low - just fixed at 0)
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,     # roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,     # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 10.0      # yaw (VERY HIGH - motor odometry drifts on rotation)
        ]
        
        # Twist covariance
        msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        self.filtered_pub.publish(msg)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
