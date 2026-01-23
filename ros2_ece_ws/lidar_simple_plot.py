#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess
import sys

class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)
        self.count = 0
    
    def callback(self, msg):
        self.count += 1
        ranges = [r for r in msg.ranges if 0.06 < r < 12.0]
        print(f"\n📊 SCAN #{self.count}")
        print(f"   Valid points: {len(ranges)}")
        if ranges:
            print(f"   Distance range: {min(ranges):.2f}m - {max(ranges):.2f}m")
            print(f"   Avg distance: {sum(ranges)/len(ranges):.2f}m")
        
        # Plot en ASCII
        if ranges:
            max_dist = max(ranges)
            print(f"\n   Distance histogram (0-{max_dist:.1f}m):")
            bins = 10
            for i in range(bins):
                bin_min = i * max_dist / bins
                bin_max = (i+1) * max_dist / bins
                count = sum(1 for r in ranges if bin_min <= r < bin_max)
                bar = "█" * (count // 2)
                print(f"   {bin_min:5.1f}m: {bar}")

def main():
    rclpy.init()
    node = LidarLogger()
    print("🎯 LIDAR Data Logger - Affichage des données en temps réel\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nArrêt...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
