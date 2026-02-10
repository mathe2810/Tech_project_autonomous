#!/usr/bin/env python3
"""
IMU Data Capture and FFT Analysis
Records raw IMU data and performs FFT to identify noise frequencies.

MPU6050 specs:
- Internal sampling: ~1kHz
- I2C configurable rate (DIV_CLOCK in register 0x19)
- Typical usage: 50Hz, 100Hz, or 200Hz

Usage:
  python3 imu_fft_analysis.py --duration 30 --sample-rate 100
  
This creates:
  - imu_raw_data.csv (raw readings)
  - accel_fft.png (FFT plot for accelerometer)
  - gyro_fft.png (FFT plot for gyroscope)
"""

import argparse
import csv
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUCaptureNode(Node):
    def __init__(self, duration=30, sample_rate=100, output_file='imu_raw_data.csv'):
        super().__init__('imu_fft_capture')
        
        self.duration = duration
        self.sample_rate = sample_rate
        self.max_samples = duration * sample_rate
        self.output_file = output_file
        
        # Buffers for each axis
        self.accel_x = deque(maxlen=self.max_samples)
        self.accel_y = deque(maxlen=self.max_samples)
        self.accel_z = deque(maxlen=self.max_samples)
        self.gyro_x = deque(maxlen=self.max_samples)
        self.gyro_y = deque(maxlen=self.max_samples)
        self.gyro_z = deque(maxlen=self.max_samples)
        self.timestamps = deque(maxlen=self.max_samples)
        
        # Subscription
        self.sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.start_time = self.get_clock().now()
        self.count = 0
        
        self.get_logger().info(
            f'IMU FFT Capture started\n'
            f'  Duration: {duration}s\n'
            f'  Target sample rate: {sample_rate}Hz\n'
            f'  Expected samples: {self.max_samples}\n'
            f'  Output: {output_file}'
        )
    
    def imu_callback(self, msg: Imu):
        """Capture IMU data"""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # Stop after duration
        if elapsed > self.duration:
            self.get_logger().info(f'Capture complete: {self.count} samples')
            self.save_data()
            raise KeyboardInterrupt
        
        # Store data
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)
        self.timestamps.append(elapsed)
        
        self.count += 1
        
        if self.count % 50 == 0:
            self.get_logger().info(
                f'Captured {self.count}/{self.max_samples} samples '
                f'({100*self.count/self.max_samples:.1f}%)'
            )
    
    def save_data(self):
        """Save captured data to CSV"""
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time', 
                'accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z'
            ])
            for i in range(len(self.timestamps)):
                writer.writerow([
                    self.timestamps[i],
                    self.accel_x[i], self.accel_y[i], self.accel_z[i],
                    self.gyro_x[i], self.gyro_y[i], self.gyro_z[i]
                ])
        
        self.get_logger().info(f'Data saved to {self.output_file}')


def perform_fft_analysis(csv_file, sample_rate):
    """Analyze FFT of captured data"""
    print(f'\n=== FFT Analysis ({csv_file}) ===\n')
    
    # Load data
    data = {}
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for key in ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
            data[key] = []
        
        for row in reader:
            for key in data:
                data[key].append(float(row[key]))
    
    # Convert to numpy arrays
    for key in data:
        data[key] = np.array(data[key])
    
    # Remove DC offset (mean)
    for key in data:
        data[key] = data[key] - np.mean(data[key])
    
    # Number of samples
    N = len(data['accel_x'])
    print(f'Samples: {N}')
    print(f'Duration: {N/sample_rate:.1f}s')
    print(f'Sample rate: {sample_rate}Hz')
    print(f'Nyquist frequency: {sample_rate/2}Hz\n')
    
    # Frequency axis
    freqs = np.fft.fftfreq(N, 1/sample_rate)[:N//2]
    
    # Create FFT plots
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'IMU FFT Analysis (SR={sample_rate}Hz, Duration={N/sample_rate:.1f}s)', 
                 fontsize=14, fontweight='bold')
    
    # Accelerometer FFT
    for idx, axis in enumerate(['x', 'y', 'z']):
        key = f'accel_{axis}'
        fft_vals = np.fft.fft(data[key])[:N//2]
        magnitude = np.abs(fft_vals) * 2/N
        
        ax = axes[idx, 0]
        ax.semilogy(freqs, magnitude, linewidth=0.8)
        ax.set_ylabel(f'Magnitude (m/s²)')
        ax.set_title(f'Accelerometer {axis.upper()}')
        ax.grid(True, alpha=0.3)
        ax.set_xlim([0, sample_rate/2])
        
        # Find peaks
        threshold = np.max(magnitude) * 0.1
        peaks = []
        for i in range(1, len(magnitude)-1):
            if magnitude[i] > threshold and magnitude[i] > magnitude[i-1] and magnitude[i] > magnitude[i+1]:
                peaks.append((freqs[i], magnitude[i]))
        
        if peaks:
            peaks.sort(key=lambda x: x[1], reverse=True)
            print(f'Accel {axis}: Top peaks:')
            for freq, mag in peaks[:3]:
                ax.plot(freq, mag, 'ro', markersize=6)
                print(f'  {freq:.1f}Hz: {mag:.3f}')
    
    # Gyroscope FFT
    for idx, axis in enumerate(['x', 'y', 'z']):
        key = f'gyro_{axis}'
        fft_vals = np.fft.fft(data[key])[:N//2]
        magnitude = np.abs(fft_vals) * 2/N
        
        ax = axes[idx, 1]
        ax.semilogy(freqs, magnitude, linewidth=0.8, color='orange')
        ax.set_ylabel(f'Magnitude (rad/s)')
        ax.set_title(f'Gyroscope {axis.upper()}')
        ax.grid(True, alpha=0.3)
        ax.set_xlim([0, sample_rate/2])
        
        # Find peaks
        threshold = np.max(magnitude) * 0.1
        peaks = []
        for i in range(1, len(magnitude)-1):
            if magnitude[i] > threshold and magnitude[i] > magnitude[i-1] and magnitude[i] > magnitude[i+1]:
                peaks.append((freqs[i], magnitude[i]))
        
        if peaks:
            peaks.sort(key=lambda x: x[1], reverse=True)
            print(f'Gyro {axis}: Top peaks:')
            for freq, mag in peaks[:3]:
                ax.plot(freq, mag, 'ro', markersize=6)
                print(f'  {freq:.1f}Hz: {mag:.3f}')
    
    axes[-1, 0].set_xlabel('Frequency (Hz)')
    axes[-1, 1].set_xlabel('Frequency (Hz)')
    
    plt.tight_layout()
    plt.savefig('imu_fft_analysis.png', dpi=150, bbox_inches='tight')
    print(f'\nPlot saved to imu_fft_analysis.png')
    plt.show()


def main(args=None):
    parser = argparse.ArgumentParser(description='Capture and analyze IMU data with FFT')
    parser.add_argument('--duration', type=int, default=30, help='Capture duration in seconds')
    parser.add_argument('--sample-rate', type=int, default=100, help='Expected IMU sample rate (Hz)')
    parser.add_argument('--output', type=str, default='imu_raw_data.csv', help='Output CSV file')
    parser.add_argument('--analyze-only', action='store_true', help='Only analyze existing CSV')
    
    args = parser.parse_args()
    
    if args.analyze_only:
        # Just analyze existing CSV
        perform_fft_analysis(args.output, args.sample_rate)
    else:
        # Capture data
        rclpy.init(args=None)
        try:
            node = IMUCaptureNode(
                duration=args.duration,
                sample_rate=args.sample_rate,
                output_file=args.output
            )
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()
        
        # Analyze captured data
        perform_fft_analysis(args.output, args.sample_rate)


if __name__ == '__main__':
    main()
