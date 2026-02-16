#!/usr/bin/env python3
"""
Advanced IMU Filter with FIR filtering.
Replaces the simple EMA filter with a proper FIR filter for better frequency response.

Combines:
- FIR low-pass filter (cuts high-frequency noise)
- Optional notch filter (removes specific frequency parasites)
- Adaptive filtering based on signal energy

Usage:
  python3 imu_fir_filter.py --cutoff 20 --order 21
  
Parameters:
  --cutoff: Low-pass cutoff frequency (Hz)
  --order: FIR filter order (higher = sharper but more latency)
  --sample-rate: IMU sample rate (Hz)
"""

import argparse
import math
import numpy as np
from scipy import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class FIRFilter1D:
    """FIR (Finite Impulse Response) Filter"""
    def __init__(self, coefficients):
        """
        coefficients: FIR filter coefficients (from scipy.signal.firwin, etc.)
        """
        self.coeffs = np.array(coefficients, dtype=np.float64)
        self.order = len(coefficients)
        self.buffer = np.zeros(self.order)
        self.index = 0
    
    def update(self, sample):
        """Apply FIR filter to sample"""
        self.buffer[self.index] = sample
        self.index = (self.index + 1) % self.order
        
        # Circular convolution
        output = np.dot(self.coeffs, np.roll(self.buffer, self.order - self.index))
        return output


class NotchFilter1D:
    """Simple IIR Notch Filter to remove specific frequency"""
    def __init__(self, freq, sample_rate, Q=30):
        """
        freq: Frequency to remove (Hz)
        sample_rate: Sampling frequency (Hz)
        Q: Quality factor (higher = sharper notch)
        """
        self.w0 = 2 * math.pi * freq / sample_rate
        self.alpha = math.sin(self.w0) / (2 * Q)
        
        # IIR coefficients
        self.b0 = 1
        self.b1 = -2 * math.cos(self.w0)
        self.b2 = 1
        
        self.a0 = 1 + self.alpha
        self.a1 = -2 * math.cos(self.w0)
        self.a2 = 1 - self.alpha
        
        # Normalize
        self.b0 /= self.a0
        self.b1 /= self.a0
        self.b2 /= self.a0
        self.a1 /= self.a0
        self.a2 /= self.a0
        
        # History
        self.x1 = self.x2 = 0
        self.y1 = self.y2 = 0
    
    def update(self, x):
        """Apply notch filter"""
        y = self.b0*x + self.b1*self.x1 + self.b2*self.x2 - self.a1*self.y1 - self.a2*self.y2
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return y


class IMUFIRFilterNode(Node):
    def __init__(self, cutoff_freq=20, fir_order=21, sample_rate=100, 
                 notch_freqs=None):
        super().__init__('imu_fir_filter')
        
        self.declare_parameter('cutoff_freq', float(cutoff_freq))
        self.declare_parameter('fir_order', fir_order)
        self.declare_parameter('sample_rate', float(sample_rate))
        self.declare_parameter('notch_freqs', notch_freqs or [])
        
        cutoff_freq = self.get_parameter('cutoff_freq').value
        fir_order = self.get_parameter('fir_order').value
        sample_rate = self.get_parameter('sample_rate').value
        notch_freqs = self.get_parameter('notch_freqs').value
        
        self.sample_rate = sample_rate
        
        # Design FIR filter (Hamming window)
        # Cutoff is normalized to Nyquist frequency
        nyquist = sample_rate / 2
        normalized_cutoff = cutoff_freq / nyquist
        
        # Ensure valid range
        if normalized_cutoff >= 1.0:
            self.get_logger().warn(f'Cutoff {cutoff_freq}Hz > Nyquist {nyquist}Hz, clamping')
            normalized_cutoff = 0.99
        
        fir_coeffs = signal.firwin(fir_order, normalized_cutoff, window='hamming')
        
        # Create filters for each axis
        self.accel_filters = [FIRFilter1D(fir_coeffs) for _ in range(3)]
        self.gyro_filters = [FIRFilter1D(fir_coeffs) for _ in range(3)]
        
        # Optional notch filters (for known parasitic frequencies)
        self.accel_notches = [[] for _ in range(3)]
        self.gyro_notches = [[] for _ in range(3)]
        
        if notch_freqs:
            for freq in notch_freqs:
                if 0 < freq < nyquist:
                    for i in range(3):
                        self.accel_notches[i].append(NotchFilter1D(freq, sample_rate))
                        self.gyro_notches[i].append(NotchFilter1D(freq, sample_rate))
                    self.get_logger().info(f'Added notch filter at {freq}Hz')
        
        # Subscriptions
        self.sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        
        # Publisher
        self.pub = self.create_publisher(
            Imu, '/imu/data_filtered', 10
        )
        
        self.get_logger().info(
            f'IMU FIR Filter started\n'
            f'  Cutoff: {cutoff_freq}Hz\n'
            f'  FIR Order: {fir_order}\n'
            f'  Sample Rate: {sample_rate}Hz\n'
            f'  Nyquist: {nyquist}Hz\n'
            f'  Notch frequencies: {notch_freqs}'
        )
    
    def imu_callback(self, msg: Imu):
        """Filter IMU data"""
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        # Filter acceleration
        accel_raw = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        accel_filtered = []
        for i, val in enumerate(accel_raw):
            # Apply FIR filter
            val = self.accel_filters[i].update(val)
            # Apply notch filters if any
            for notch in self.accel_notches[i]:
                val = notch.update(val)
            accel_filtered.append(val)
        
        filtered_msg.linear_acceleration.x = accel_filtered[0]
        filtered_msg.linear_acceleration.y = accel_filtered[1]
        filtered_msg.linear_acceleration.z = accel_filtered[2]
        
        # Filter angular velocity
        gyro_raw = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        gyro_filtered = []
        for i, val in enumerate(gyro_raw):
            # Apply FIR filter
            val = self.gyro_filters[i].update(val)
            # Apply notch filters if any
            for notch in self.gyro_notches[i]:
                val = notch.update(val)
            gyro_filtered.append(val)
        
        filtered_msg.angular_velocity.x = gyro_filtered[0]
        filtered_msg.angular_velocity.y = gyro_filtered[1]
        filtered_msg.angular_velocity.z = gyro_filtered[2]
        
        # Copy covariance matrices
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance
        
        # Publish
        self.pub.publish(filtered_msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='IMU FIR Filter Node')
    parser.add_argument('--cutoff', type=float, default=20, 
                        help='Low-pass cutoff frequency (Hz)')
    parser.add_argument('--order', type=int, default=21,
                        help='FIR filter order (must be odd)')
    parser.add_argument('--sample-rate', type=float, default=100,
                        help='IMU sample rate (Hz)')
    parser.add_argument('--notch', type=float, nargs='*', default=[],
                        help='Notch filter frequencies (Hz)')
    
    parsed_args = parser.parse_args()
    
    # Ensure FIR order is odd
    if parsed_args.order % 2 == 0:
        parsed_args.order += 1
    
    rclpy.init(args=None)
    try:
        node = IMUFIRFilterNode(
            cutoff_freq=parsed_args.cutoff,
            fir_order=parsed_args.order,
            sample_rate=parsed_args.sample_rate,
            notch_freqs=parsed_args.notch if parsed_args.notch else None
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
