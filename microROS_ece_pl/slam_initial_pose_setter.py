#!/usr/bin/env python3
"""
Sets SLAM's initial pose from ground truth odometry on startup.
This gives SLAM a good starting point, then scan_matching refines from there.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class SLAMInitialPoseSetter(Node):
    def __init__(self):
        super().__init__('slam_initial_pose_setter')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to ground truth odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_ground_truth',
            self.odom_callback,
            qos
        )
        
        # Publish initial pose to SLAM
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10  # Latched topic, higher QoS for reliability
        )
        
        self.first_msg_received = False
        self.get_logger().info('✅ SLAM Initial Pose Setter started, waiting for first ground truth...')
    
    def odom_callback(self, msg):
        """On first ground truth message, set SLAM's initial pose."""
        if self.first_msg_received:
            return  # Only set once
        
        self.first_msg_received = True
        
        # Create initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        
        # Use ground truth as initial estimate
        initial_pose.pose.pose.position.x = msg.pose.pose.position.x
        initial_pose.pose.pose.position.y = msg.pose.pose.position.y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = msg.pose.pose.orientation
        
        # Set covariance: high uncertainty initially (SLAM will refine with scan_matching)
        # Diagonal: [x, y, z, roll, pitch, yaw]
        initial_pose.pose.covariance[0] = 0.25    # x variance
        initial_pose.pose.covariance[7] = 0.25    # y variance
        initial_pose.pose.covariance[14] = 0.25   # z variance
        initial_pose.pose.covariance[21] = math.pi * math.pi  # roll variance
        initial_pose.pose.covariance[28] = math.pi * math.pi  # pitch variance
        initial_pose.pose.covariance[35] = 0.10   # yaw variance (tighter for direction)
        
        self.initial_pose_pub.publish(initial_pose)
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.get_logger().info(
            f'🎯 SLAM initial pose set: x={x:.3f}m y={y:.3f}m '
            f'q=[{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]'
        )

def main():
    rclpy.init()
    node = SLAMInitialPoseSetter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
