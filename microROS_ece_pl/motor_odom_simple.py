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
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('linear_bias', 0.0)
        self.declare_parameter('angular_bias', 0.0)
        self.declare_parameter('linear_deadband', 0.02)
        self.declare_parameter('angular_deadband', 0.05)
        self.declare_parameter('velocity_time_constant', 0.12)
        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('max_dt', 0.10)

        self.declare_parameter('base_cov_pose_x', 1.0)
        self.declare_parameter('base_cov_pose_y', 1.0)
        self.declare_parameter('base_cov_pose_yaw', 10.0)
        self.declare_parameter('base_cov_twist_x', 1.0)
        self.declare_parameter('base_cov_twist_yaw', 4.0)
        self.declare_parameter('cov_gain_v', 2.0)
        self.declare_parameter('cov_gain_w', 1.5)
        self.declare_parameter('cov_gain_accel', 0.4)
        self.declare_parameter('cov_multiplier_min', 1.0)
        self.declare_parameter('cov_multiplier_max', 20.0)
        
        # Get parameters
        self.robot_frame = self.get_parameter('robot_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.update_rate = self.get_parameter('update_rate').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.linear_bias = self.get_parameter('linear_bias').value
        self.angular_bias = self.get_parameter('angular_bias').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.angular_deadband = self.get_parameter('angular_deadband').value
        self.velocity_time_constant = self.get_parameter('velocity_time_constant').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_dt = self.get_parameter('max_dt').value

        self.base_cov_pose_x = self.get_parameter('base_cov_pose_x').value
        self.base_cov_pose_y = self.get_parameter('base_cov_pose_y').value
        self.base_cov_pose_yaw = self.get_parameter('base_cov_pose_yaw').value
        self.base_cov_twist_x = self.get_parameter('base_cov_twist_x').value
        self.base_cov_twist_yaw = self.get_parameter('base_cov_twist_yaw').value
        self.cov_gain_v = self.get_parameter('cov_gain_v').value
        self.cov_gain_w = self.get_parameter('cov_gain_w').value
        self.cov_gain_accel = self.get_parameter('cov_gain_accel').value
        self.cov_multiplier_min = self.get_parameter('cov_multiplier_min').value
        self.cov_multiplier_max = self.get_parameter('cov_multiplier_max').value
        
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
        self.v_est = 0.0
        self.w_est = 0.0
        self.prev_v_est = 0.0
        self.prev_w_est = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        
        # Timer
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(
            f'Motor Odometry Node started at {self.update_rate}Hz '
            f'(lin_scale={self.linear_scale:.3f}, ang_scale={self.angular_scale:.3f}, tau={self.velocity_time_constant:.3f}s)'
        )

    @staticmethod
    def _sgn(value):
        return 1.0 if value >= 0.0 else -1.0

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _map_command(self, cmd_value, scale, bias, deadband):
        magnitude = abs(cmd_value)
        if magnitude < deadband:
            return 0.0
        return self._sgn(cmd_value) * (scale * magnitude + bias)

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def cmd_vel_callback(self, msg):
        """Track cmd_vel for odometry"""
        self.last_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        """Publish odometry at regular intervals"""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return
        dt = min(dt, self.max_dt)

        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        cmd_linear = self.last_vel.linear.x if cmd_age <= self.cmd_timeout else 0.0
        cmd_angular = self.last_vel.angular.z if cmd_age <= self.cmd_timeout else 0.0

        v_target = self._map_command(
            cmd_linear,
            self.linear_scale,
            self.linear_bias,
            self.linear_deadband,
        )
        w_target = self._map_command(
            cmd_angular,
            self.angular_scale,
            self.angular_bias,
            self.angular_deadband,
        )

        if self.velocity_time_constant <= 0.0:
            alpha = 1.0
        else:
            alpha = dt / (self.velocity_time_constant + dt)

        self.prev_v_est = self.v_est
        self.prev_w_est = self.w_est
        self.v_est += alpha * (v_target - self.v_est)
        self.w_est += alpha * (w_target - self.w_est)

        theta_mid = self.theta + 0.5 * self.w_est * dt
        self.x += self.v_est * math.cos(theta_mid) * dt
        self.y += self.v_est * math.sin(theta_mid) * dt
        self.theta = self._wrap_angle(self.theta + self.w_est * dt)

        accel_v = abs(self.v_est - self.prev_v_est) / dt
        accel_w = abs(self.w_est - self.prev_w_est) / dt
        cov_multiplier = 1.0 + (
            self.cov_gain_v * abs(self.v_est)
            + self.cov_gain_w * abs(self.w_est)
            + self.cov_gain_accel * (accel_v + accel_w)
        )
        cov_multiplier = self._clamp(
            cov_multiplier,
            self.cov_multiplier_min,
            self.cov_multiplier_max,
        )
        
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
        
        odom.twist.twist.linear.x = self.v_est
        odom.twist.twist.angular.z = self.w_est

        odom.pose.covariance[0] = self.base_cov_pose_x * cov_multiplier
        odom.pose.covariance[7] = self.base_cov_pose_y * cov_multiplier
        odom.pose.covariance[35] = self.base_cov_pose_yaw * cov_multiplier
        odom.twist.covariance[0] = self.base_cov_twist_x * cov_multiplier
        odom.twist.covariance[35] = self.base_cov_twist_yaw * cov_multiplier
        
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
