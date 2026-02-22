#!/usr/bin/env python3
"""
Motor Odometry Node - Simple version (no lifecycle)
Publishes motor-based odometry with high covariance
Let SLAM Toolbox do the correction
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math

class MotorOdomNode(Node):
    def __init__(self):
        super().__init__('motor_odom')
        
        # Parameters
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        # Get parameters
        self.robot_frame = self.get_parameter('robot_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers (publish as /odom_motor - will be fused)
        self.odom_pub = self.create_publisher(Odometry, '/odom_motor', qos)
        
        # Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_vel = Twist()
        
        # Timer
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(f'Motor Odometry Node started at {self.update_rate}Hz')

    def cmd_vel_callback(self, msg):
        """Track cmd_vel for odometry"""
        self.last_vel = msg

    def timer_callback(self):
        """Publish odometry at regular intervals"""
        # Simple kinematic model
        dt = 1.0 / self.update_rate
        
        if self.last_vel.linear.x != 0 or self.last_vel.angular.z != 0:
            # Update pose
            self.x += self.last_vel.linear.x * math.cos(self.theta) * dt
            self.y += self.last_vel.linear.x * math.sin(self.theta) * dt
            self.theta += self.last_vel.angular.z * dt
        
        # Publish odometry
        now = self.get_clock().now()
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.robot_frame
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from theta
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = self.last_vel.linear.x
        odom.twist.twist.angular.z = self.last_vel.angular.z
        
        # HIGH COVARIANCE - let SLAM correct us
        odom.pose.covariance[0] = 1.0   # x
        odom.pose.covariance[7] = 1.0   # y
        odom.pose.covariance[35] = 10.0 # yaw (HIGH - SLAM will correct)
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.robot_frame
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = MotorOdomNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
