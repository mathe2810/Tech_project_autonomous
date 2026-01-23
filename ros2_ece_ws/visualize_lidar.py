#!/usr/bin/env python3
"""
Visualize LIDAR data in real-time using matplotlib
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import threading

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        self.get_logger().info("Starting LIDAR Visualizer...")
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.current_ranges = None
        self.current_angles = None
        self.scan_count = 0
        
        # Setup matplotlib
        self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8, 8))
        self.scatter = None
        
        self.get_logger().info("Waiting for LIDAR data...")
        
        # Start animation
        ani = FuncAnimation(self.fig, self.update_plot, interval=200, blit=False)
        plt.show()
    
    def scan_callback(self, msg):
        self.scan_count += 1
        if self.scan_count % 5 == 0:
            self.get_logger().info(f"Received scan #{self.scan_count} with {len(msg.ranges)} points")
        
        self.current_ranges = np.array(msg.ranges)
        
        # Compute angles
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.current_angles = angles
    
    def update_plot(self, frame):
        if self.current_ranges is None:
            self.ax.set_title('Waiting for LIDAR data...')
            return
        
        self.ax.clear()
        
        # Filter out zero/invalid ranges
        valid = self.current_ranges > 0.06
        angles = self.current_angles[valid]
        ranges = self.current_ranges[valid]
        
        if len(ranges) == 0:
            self.ax.set_title('No valid points')
            return
        
        # Plot - color by distance
        colors = ranges
        self.ax.scatter(angles, ranges, c=colors, cmap='viridis', s=20, alpha=0.6)
        self.ax.set_ylim(0, 12)
        self.ax.set_title(f'LIDAR Scan #{self.scan_count} - {len(ranges)} valid points\nMin: {ranges.min():.2f}m, Max: {ranges.max():.2f}m')
        self.ax.grid(True)

def main(args=None):
    rclpy.init(args=args)
    visualizer = LidarVisualizer()
    
    # Run ROS2 in background thread
    def ros_spin():
        rclpy.spin(visualizer)
    
    spinner = threading.Thread(target=ros_spin, daemon=True)
    spinner.start()
    
    # Keep matplotlib running in main thread
    plt.show()
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
