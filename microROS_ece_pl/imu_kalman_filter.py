#!/usr/bin/env python3
"""
Simple Exponential Moving Average (EMA) filter for IMU data smoothing
Subscribes to raw /imu/data and publishes filtered /imu/data_filtered
EMA is much faster with lower latency than Kalman filtering
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class EMAFilter1D:
    """Simple Exponential Moving Average Filter (low latency)"""
    def __init__(self, alpha=0.3):
        """
        alpha: smoothing factor (0 to 1)
        - Higher alpha = more responsive, less smoothing
        - Lower alpha = more smoothing, more lag
        Default 0.3 = good balance
        """
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False
        
    def update(self, measurement):
        """Update with new measurement, return filtered value"""
        if not self.initialized:
            self.value = measurement
            self.initialized = True
            return measurement
        
        # EMA = alpha * measurement + (1 - alpha) * previous_value
        self.value = self.alpha * measurement + (1.0 - self.alpha) * self.value
        return self.value


class IMUFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter')
        
        # Declare parameters
        self.declare_parameter('accel_alpha', 0.35)  # Higher = more responsive
        self.declare_parameter('gyro_alpha', 0.4)    # Gyro can be more responsive
        
        accel_alpha = self.get_parameter('accel_alpha').value
        gyro_alpha = self.get_parameter('gyro_alpha').value
        
        # Create EMA filters for each axis
        self.filter_accel_x = EMAFilter1D(alpha=accel_alpha)
        self.filter_accel_y = EMAFilter1D(alpha=accel_alpha)
        self.filter_accel_z = EMAFilter1D(alpha=accel_alpha)
        self.filter_gyro_x = EMAFilter1D(alpha=gyro_alpha)
        self.filter_gyro_y = EMAFilter1D(alpha=gyro_alpha)
        self.filter_gyro_z = EMAFilter1D(alpha=gyro_alpha)
        
        # Subscriber and publisher
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_filtered',
            10
        )
        
        self.get_logger().info(f'IMU EMA Filter node started (accel_alpha={accel_alpha}, gyro_alpha={gyro_alpha})')
        
    def imu_callback(self, msg: Imu):
        """Filter incoming IMU data"""
        # Create filtered message
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        # Filter accelerations
        filtered_msg.linear_acceleration.x = self.filter_accel_x.update(msg.linear_acceleration.x)
        filtered_msg.linear_acceleration.y = self.filter_accel_y.update(msg.linear_acceleration.y)
        filtered_msg.linear_acceleration.z = self.filter_accel_z.update(msg.linear_acceleration.z)
        
        # Filter angular velocities
        filtered_msg.angular_velocity.x = self.filter_gyro_x.update(msg.angular_velocity.x)
        filtered_msg.angular_velocity.y = self.filter_gyro_y.update(msg.angular_velocity.y)
        filtered_msg.angular_velocity.z = self.filter_gyro_z.update(msg.angular_velocity.z)
        
        # Copy covariance matrices
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance
        
        # Publish filtered data
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

