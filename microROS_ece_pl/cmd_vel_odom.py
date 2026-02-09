#!/usr/bin/env python3
"""
Simple odometry integrator from /cmd_vel for Nav2.
Publishes /odom and TF odom -> base_link.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class CmdVelOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def cmd_vel_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        # Quaternion from yaw
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
