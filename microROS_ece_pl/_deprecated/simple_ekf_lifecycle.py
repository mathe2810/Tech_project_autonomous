#!/usr/bin/env python3
"""
Odometry Relay Lifecycle Node
Simply relays /odom as /odom_filtered for compatibility.
100% LIDAR mode - no IMU fusion.

Lifecycle aware - conforms to Nav2 lifecycle architecture.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class OdomRelayLifecycleNode(LifecycleNode):
    """Odometry relay without fusion - Lifecycle Node"""
    
    def __init__(self):
        super().__init__('simple_ekf')
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        
        # Publishers and subscribers
        self.odom_sub = None
        self.filtered_pub = None
        self.tf_broadcaster = None
        
        self.get_logger().info("Odometry Relay Node created (UNCONFIGURED)")
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure - create publishers and subscribers"""
        self.get_logger().info("Configuring Odometry Relay Node...")
        
        try:
            # Create subscriber
            self.odom_sub = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10
            )
            
            # Create publisher
            self.filtered_pub = self.create_publisher(
                Odometry, '/odom_filtered', 10
            )
            
            # TF broadcaster
            self.tf_broadcaster = TransformBroadcaster(self)
            
            # Timer for publishing
            self.publish_timer = self.create_timer(
                0.05,  # 20 Hz
                self.publish_callback
            )
            
            self.get_logger().info("✓ Odometry Relay configured (NO IMU fusion)")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate - start publishing"""
        self.get_logger().info("Activating Odometry Relay Node...")
        try:
            self.get_logger().info("✓ Odometry Relay active")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate - stop publishing"""
        self.get_logger().info("Deactivating Odometry Relay Node...")
        try:
            self.get_logger().info("✓ Odometry Relay deactivated")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Deactivation error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup - destroy resources"""
        self.get_logger().info("Cleaning up Odometry Relay Node...")
        try:
            if self.odom_sub is not None:
                self.destroy_subscription(self.odom_sub)
                self.odom_sub = None
            
            if self.filtered_pub is not None:
                self.destroy_publisher(self.filtered_pub)
                self.filtered_pub = None
            
            if self.publish_timer is not None:
                self.destroy_timer(self.publish_timer)
                self.publish_timer = None
            
            self.get_logger().info("✓ Odometry Relay cleanup complete")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown"""
        self.get_logger().info("Shutting down Odometry Relay Node...")
        return TransitionCallbackReturn.SUCCESS
    
    def odom_callback(self, msg: Odometry):
        """Receive odometry"""
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
        """Publish relayed odometry"""
        if self.get_current_state().id != 3:  # Not ACTIVE
            return
        
        now = self.get_clock().now()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        
        # Velocity
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        
        # HIGH COVARIANCE
        msg.pose.covariance = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 10.0
        ]
        
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
    node = OdomRelayLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
