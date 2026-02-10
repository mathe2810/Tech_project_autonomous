#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class ScanAccumulator(Node):
    def __init__(self):
        super().__init__('scan_accumulator')
        
        # Parameters
        self.accumulate_frames = 60  # Accumulate 60 frames to get 720 points (more density)
        self.pts_per_frame = 12
        
        # Buffers
        self.frame_buffer = []
        self.accumulated_scan = None
        
        # Subscriber & Publisher
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_accumulated',
            10)
        
        self.frame_count = 0
        self.get_logger().info('ScanAccumulator started')
        
    def scan_callback(self, msg):
        """Accumulate frames until we have 30, then publish 360-point scan"""
        
        self.frame_count += 1
        
        # Count non-infinity points in this frame
        valid_points = sum(1 for r in msg.ranges if not math.isinf(r))
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Frame {self.frame_count}: {valid_points} points received')
        
        # Initialize accumulated_scan on first frame
        if self.accumulated_scan is None:
            self.accumulated_scan = LaserScan()
            self.accumulated_scan.angle_min = 0.0
            self.accumulated_scan.angle_max = 2.0 * math.pi
            self.accumulated_scan.angle_increment = 2.0 * math.pi / 360.0
            self.accumulated_scan.time_increment = 0.0
            self.accumulated_scan.scan_time = 0.033 * self.accumulate_frames
            self.accumulated_scan.range_min = 0.15
            self.accumulated_scan.range_max = 12.0
            self.accumulated_scan.ranges = [float('inf')] * 360
            self.accumulated_scan.intensities = [0.0] * 360
        
        # Add this frame's points to accumulator (they're already distributed across 360)
        for i, r in enumerate(msg.ranges):
            if not math.isinf(r) and r > 0:
                # Overlay non-infinity points
                self.accumulated_scan.ranges[i] = r
                if i < len(msg.intensities):
                    self.accumulated_scan.intensities[i] = msg.intensities[i]
        
        # Store frame for counter
        self.frame_buffer.append(msg)
        
        # When we have 30 frames, publish and reset
        if len(self.frame_buffer) >= self.accumulate_frames:
            # Update timestamp
            self.accumulated_scan.header.stamp = self.get_clock().now().to_msg()
            self.accumulated_scan.header.frame_id = 'laser'
            
            # Count final points
            final_points = sum(1 for r in self.accumulated_scan.ranges if not math.isinf(r))
            self.get_logger().info(f'Publishing accumulated scan: {final_points} points total (frames 1-{len(self.frame_buffer)})')
            
            # Publish
            self.publisher.publish(self.accumulated_scan)
            
            # Reset for next cycle
            self.frame_buffer = []
            self.accumulated_scan = None

def main(args=None):
    rclpy.init(args=args)
    node = ScanAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
