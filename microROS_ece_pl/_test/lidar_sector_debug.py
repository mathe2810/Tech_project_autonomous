#!/usr/bin/env python3
"""
LiDAR sector debug node: Affiche les valeurs front, gauche, droite à partir de /scan_raw.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSectorDebug(Node):
    def __init__(self):
        super().__init__('lidar_sector_debug')
        self.lidar_offset_deg = -89.1  # Calibré selon test_lidar_front_alignment
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.get_logger().info('LidarSectorDebug ready.')

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(ranges)

        def valid_mean(start_deg, end_deg):
            start_rad = np.deg2rad(start_deg)
            end_rad = np.deg2rad(end_deg)
            i_start = int((start_rad - angle_min) / angle_inc)
            i_end = int((end_rad - angle_min) / angle_inc)
            i_start = np.clip(i_start, 0, n-1)
            i_end = np.clip(i_end, 0, n-1)
            vals = ranges[i_start:i_end+1]
            vals = vals[np.isfinite(vals)]
            return np.mean(vals) if len(vals) > 0 else np.nan

        # Secteur gauche (0°)
        left_start = -10
        left_end = +10
        d_left = valid_mean(left_start, left_end)

        # Secteur frontal (90°)
        front_start = 80
        front_end = 100
        d_front = valid_mean(front_start, front_end)

        # Secteur droite (180°)
        right_start = 170
        right_end = 190
        d_right = valid_mean(right_start, right_end)

        self.get_logger().info(f"d_front={d_front:.2f} | d_left={d_left:.2f} | d_right={d_right:.2f}")


def main():
    rclpy.init()
    node = LidarSectorDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
