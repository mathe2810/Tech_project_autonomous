#!/usr/bin/env python3
"""
Motor Odometry Node - Lifecycle aware
100% LIDAR mode: IMU NOT used for heading, only motor commands.
SLAM corrects heading via scan matching.

Based on Nav2 Lifecycle architecture:
- UNCONFIGURED (initial state)
- INACTIVE (configured, ready)
- ACTIVE (publishing)
"""

import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import time

class MotorOdomLifecycleNode(LifecycleNode):
    """Motor odometry without encoders - Lifecycle Node"""
    
    def __init__(self):
        super().__init__('motor_odom')
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_update_time = None
        
        # Publishers and subscribers (created on_configure)
        self.odom_pub = None
        self.cmd_vel_sub = None
        self.tf_broadcaster = None
        
        self.get_logger().info("Motor Odometry Node created (UNCONFIGURED)")
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node - load parameters, create publishers"""
        self.get_logger().info("Configuring Motor Odometry Node...")
        
        try:
            # Declare parameters
            self.declare_parameter('odom_frame', 'odom')
            self.declare_parameter('base_frame', 'base_link')
            self.declare_parameter('wheel_base', 0.2)
            self.declare_parameter('publish_rate', 50.0)
            
            # Get parameters
            self.odom_frame = self.get_parameter('odom_frame').value
            self.base_frame = self.get_parameter('base_frame').value
            self.wheel_base = self.get_parameter('wheel_base').value
            self.publish_rate = self.get_parameter('publish_rate').value
            
            # Create publishers
            self.odom_pub = self.create_publisher(
                Odometry, '/odom', 10
            )
            
            # Create subscribers
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10
            )
            
            # TF broadcaster
            self.tf_broadcaster = TransformBroadcaster(self)
            
            # Timer
            self.update_timer = self.create_timer(
                1.0 / self.publish_rate,
                self.update_callback
            )
            
            self.last_update_time = time.time()
            
            self.get_logger().info(
                f"✓ Motor Odometry configured\n"
                f"  Frame: {self.odom_frame} -> {self.base_frame}\n"
                f"  Wheel base: {self.wheel_base}m\n"
                f"  Mode: 100% Motor (NO IMU)"
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node - start publishing"""
        self.get_logger().info("Activating Motor Odometry Node...")
        try:
            self.get_logger().info("✓ Motor Odometry active - publishing /odom and /tf")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node - stop publishing"""
        self.get_logger().info("Deactivating Motor Odometry Node...")
        try:
            self.get_logger().info("✓ Motor Odometry deactivated")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Deactivation error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup the node - destroy resources"""
        self.get_logger().info("Cleaning up Motor Odometry Node...")
        try:
            if self.odom_pub is not None:
                self.destroy_publisher(self.odom_pub)
                self.odom_pub = None
            
            if self.cmd_vel_sub is not None:
                self.destroy_subscription(self.cmd_vel_sub)
                self.cmd_vel_sub = None
            
            if self.update_timer is not None:
                self.destroy_timer(self.update_timer)
                self.update_timer = None
            
            self.get_logger().info("✓ Motor Odometry cleanup complete")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node"""
        self.get_logger().info("Shutting down Motor Odometry Node...")
        return TransitionCallbackReturn.SUCCESS
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands"""
        self.vx = msg.linear.x
        self.wz = msg.angular.z
    
    def update_callback(self):
        """Integrate motion and publish odometry"""
        if self.get_current_state().id != 3:  # Not ACTIVE
            return
        
        now = time.time()
        if self.last_update_time is None:
            self.last_update_time = now
            return
        
        dt = now - self.last_update_time
        self.last_update_time = now
        
        if dt <= 0:
            return
        
        # Simple kinematic integration
        if abs(self.wz) > 0.001:
            # Curved motion
            r = self.vx / self.wz
            sin_theta = math.sin(self.yaw)
            cos_theta = math.cos(self.yaw)
            sin_new = math.sin(self.yaw + self.wz * dt)
            cos_new = math.cos(self.yaw + self.wz * dt)
            
            self.x += r * (sin_new - sin_theta)
            self.y += r * (-cos_new + cos_theta)
        else:
            # Straight motion
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt
        
        self.yaw += self.wz * dt
        self.yaw = self._normalize_angle(self.yaw)
        
        # Publish odometry
        self._publish_odometry()
        
        # Publish TF
        self._publish_tf()
    
    def _publish_odometry(self):
        """Publish /odom message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        
        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Orientation from yaw
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        
        # Velocity
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        
        # HIGH COVARIANCE - signal SLAM to correct
        msg.pose.covariance = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # x (HIGH)
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,      # y (HIGH)
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,      # z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,     # roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,     # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 10.0      # yaw (VERY HIGH)
        ]
        
        msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        self.odom_pub.publish(msg)
    
    def _publish_tf(self):
        """Publish odom -> base_link transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
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
    node = MotorOdomLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
