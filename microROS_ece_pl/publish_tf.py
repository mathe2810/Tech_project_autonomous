#!/usr/bin/env python3
"""
Publish static TF transforms between robot frames.
Critical for SLAM: laser_link -> base_link, imu_link -> base_link
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transforms
        self.publish_transforms()
        self.get_logger().info("Static TF transforms published")
    
    def publish_transforms(self):
        """Publish all static TF transforms"""
        
        # laser_link -> base_link (LIDAR on top of robot, centered)
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser_link'
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.05  # 5cm above base
        t_laser.transform.rotation.x = 0.0
        t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = 0.0
        t_laser.transform.rotation.w = 1.0
        
        # imu_link -> base_link (IMU centered on robot)
        t_imu = TransformStamped()
        t_imu.header.stamp = self.get_clock().now().to_msg()
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu_link'
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.0
        t_imu.transform.rotation.x = 0.0
        t_imu.transform.rotation.y = 0.0
        t_imu.transform.rotation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        
        # Publish both transforms
        self.tf_static_broadcaster.sendTransform([t_laser, t_imu])


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
