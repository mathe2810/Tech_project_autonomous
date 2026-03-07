#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException


class SlamOdomFromTF(Node):
    def __init__(self):
        super().__init__('slam_odom_from_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, '/odom_slam', 10)
        self.last_x = None
        self.last_y = None
        self.last_t = None
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('✅ Publishing SLAM odom on /odom_slam from TF map->base_link')

    def tick(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return

        now = self.get_clock().now()
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q

        if self.last_x is not None and self.last_t is not None:
            dt = (now - self.last_t).nanoseconds / 1e9
            if dt > 1e-4:
                msg.twist.twist.linear.x = (x - self.last_x) / dt
                msg.twist.twist.linear.y = (y - self.last_y) / dt
                msg.twist.twist.angular.z = (yaw - self.last_yaw) / dt

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.last_t = now

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SlamOdomFromTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
