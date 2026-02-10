#!/usr/bin/env python3
"""
Test node to compare raw vs filtered IMU data
Shows side-by-side comparison and saves data to CSV for analysis
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime

class IMUTestNode(Node):
    def __init__(self):
        super().__init__('imu_test_node')
        
        # Create output file for data logging
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = f"/tmp/imu_comparison_{timestamp}.csv"
        
        # Open CSV file for writing
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Time (s)', 
            'Raw_ax', 'Raw_ay', 'Raw_az',
            'Filtered_ax', 'Filtered_ay', 'Filtered_az',
            'Diff_ax', 'Diff_ay', 'Diff_az'
        ])
        
        self.raw_msg = None
        self.filtered_msg = None
        self.sample_count = 0
        self.max_samples = 1000  # Log 1000 samples for analysis
        
        # Subscribers
        self.sub_raw = self.create_subscription(Imu, '/imu/data', self.raw_callback, 10)
        self.sub_filtered = self.create_subscription(Imu, '/imu/filtered', self.filtered_callback, 10)
        
        self.get_logger().info(f"Test node started - logging to {self.output_file}")
        self.get_logger().info("Waiting for raw and filtered IMU data...")
    
    def raw_callback(self, msg):
        self.raw_msg = msg
        self.process_data()
    
    def filtered_callback(self, msg):
        self.filtered_msg = msg
        self.process_data()
    
    def process_data(self):
        """Compare raw and filtered data when both are available"""
        if self.raw_msg is None or self.filtered_msg is None:
            return
        
        if self.sample_count >= self.max_samples:
            self.get_logger().info(f"Reached {self.max_samples} samples. Test complete!")
            self.csv_file.close()
            return
        
        # Extract data
        raw_ax = self.raw_msg.linear_acceleration.x
        raw_ay = self.raw_msg.linear_acceleration.y
        raw_az = self.raw_msg.linear_acceleration.z
        
        filt_ax = self.filtered_msg.linear_acceleration.x
        filt_ay = self.filtered_msg.linear_acceleration.y
        filt_az = self.filtered_msg.linear_acceleration.z
        
        # Calculate differences
        diff_ax = raw_ax - filt_ax
        diff_ay = raw_ay - filt_ay
        diff_az = raw_az - filt_az
        
        # Log to console every 50 samples
        if self.sample_count % 50 == 0:
            self.get_logger().info(
                f"Sample {self.sample_count}:\n"
                f"  Raw:      ax={raw_ax:7.3f}  ay={raw_ay:7.3f}  az={raw_az:7.3f}\n"
                f"  Filtered: ax={filt_ax:7.3f}  ay={filt_ay:7.3f}  az={filt_az:7.3f}\n"
                f"  Diff:     ax={diff_ax:7.3f}  ay={diff_ay:7.3f}  az={diff_az:7.3f}"
            )
        
        # Write to CSV
        self.csv_writer.writerow([
            self.sample_count / 100.0,  # Assuming 100 Hz sampling
            f"{raw_ax:.6f}", f"{raw_ay:.6f}", f"{raw_az:.6f}",
            f"{filt_ax:.6f}", f"{filt_ay:.6f}", f"{filt_az:.6f}",
            f"{diff_ax:.6f}", f"{diff_ay:.6f}", f"{diff_az:.6f}"
        ])
        self.csv_file.flush()
        
        self.sample_count += 1
        
        # Reset messages
        self.raw_msg = None
        self.filtered_msg = None


def main(args=None):
    rclpy.init(args=args)
    node = IMUTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
        node.csv_file.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
