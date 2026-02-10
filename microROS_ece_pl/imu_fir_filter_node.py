#!/usr/bin/env python3
"""
FIR Filter Node for IMU data
Removes low-frequency vibrations from the robot using a high-pass FIR filter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.signal import butter, lfilter
from collections import deque

class IMUFIRFilterNode(Node):
    def __init__(self):
        super().__init__('imu_fir_filter_node')
        
        # Parameters
        self.declare_parameter('cutoff_freq', 2.5)  # Hz (high-pass cutoff)
        self.declare_parameter('filter_order', 3)   # IIR filter order (lower for stability)
        self.declare_parameter('sampling_rate', 100.0)  # Hz (ESP32 IMU rate)
        
        cutoff_freq = self.get_parameter('cutoff_freq').value
        filter_order = self.get_parameter('filter_order').value
        sampling_rate = self.get_parameter('sampling_rate').value
        
        # Design high-pass IIR filter (simpler, no padding issues)
        nyquist = sampling_rate / 2
        normalized_cutoff = cutoff_freq / nyquist
        
        # Ensure normalized cutoff is in valid range
        if normalized_cutoff >= 1.0:
            normalized_cutoff = 0.99
            self.get_logger().warn(f"Cutoff frequency too high, clamped to {normalized_cutoff * nyquist:.2f} Hz")
        
        # Design Butterworth high-pass IIR filter
        self.b, self.a = butter(filter_order, normalized_cutoff, btype='high', analog=False)
        
        self.get_logger().info(f"IIR Filter initialized:")
        self.get_logger().info(f"  Cutoff frequency: {cutoff_freq} Hz")
        self.get_logger().info(f"  Filter order: {filter_order}")
        self.get_logger().info(f"  Sampling rate: {sampling_rate} Hz")
        
        # State variables for IIR filtering (instead of buffering)
        self.ax_state = deque([0.0] * filter_order, maxlen=filter_order)
        self.ay_state = deque([0.0] * filter_order, maxlen=filter_order)
        self.az_state = deque([0.0] * filter_order, maxlen=filter_order)
        self.gx_state = deque([0.0] * filter_order, maxlen=filter_order)
        self.gy_state = deque([0.0] * filter_order, maxlen=filter_order)
        self.gz_state = deque([0.0] * filter_order, maxlen=filter_order)
        
        # Publisher and Subscriber
        self.publisher_ = self.create_publisher(Imu, '/imu/filtered', 10)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Correct topic from ESP32
            self.imu_callback,
            10
        )
        
        self.get_logger().info("IMU IIR Filter Node started")
        self.get_logger().info("Subscribing to: /imu/data")
        self.get_logger().info("Publishing to: /imu/filtered")
    
    def apply_iir_filter(self, value, state_buffer):
        """Apply single-sample IIR filter"""
        # Simple first-order high-pass: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
        # For higher orders, we use scipy's lfilter on a small window
        filtered = lfilter(self.b, self.a, [value])
        return float(filtered[0])
    
    def imu_callback(self, msg):
        """Process incoming IMU message and apply filter"""
        # Extract data
        raw_ax = msg.linear_acceleration.x
        raw_ay = msg.linear_acceleration.y
        raw_az = msg.linear_acceleration.z
        raw_gx = msg.angular_velocity.x
        raw_gy = msg.angular_velocity.y
        raw_gz = msg.angular_velocity.z
        
        # Apply filter to each axis (simple one-sample IIR)
        filt_ax = self.apply_iir_filter(raw_ax, self.ax_state)
        filt_ay = self.apply_iir_filter(raw_ay, self.ay_state)
        filt_az = self.apply_iir_filter(raw_az, self.az_state)
        filt_gx = self.apply_iir_filter(raw_gx, self.gx_state)
        filt_gy = self.apply_iir_filter(raw_gy, self.gy_state)
        filt_gz = self.apply_iir_filter(raw_gz, self.gz_state)
        
        # Create filtered message
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        # Set filtered accelerometer
        filtered_msg.linear_acceleration.x = filt_ax
        filtered_msg.linear_acceleration.y = filt_ay
        filtered_msg.linear_acceleration.z = filt_az
        
        # Set filtered gyroscope
        filtered_msg.angular_velocity.x = filt_gx
        filtered_msg.angular_velocity.y = filt_gy
        filtered_msg.angular_velocity.z = filt_gz
        
        # Copy covariance from original message
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance
        
        # Publish filtered data
        self.publisher_.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUFIRFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
