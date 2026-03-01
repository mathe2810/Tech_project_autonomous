#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanRestamperSimu(Node):
    def __init__(self):
        super().__init__('scan_restamper_simu')
        # Utilise BEST_EFFORT pour meilleure compatibilité
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, qos)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.scan_count = 0
        self.get_logger().info('✅ Restamper démarré - Inpute: /scan_raw -> Output: /scan')

    def callback(self, msg):
        # Mets à jour le timestamp avec l'horloge ROS courante
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        self.pub.publish(msg)
        self.scan_count += 1
        if self.scan_count % 30 == 0:
            self.get_logger().info(f'✅ {self.scan_count} scans traités, {len(msg.ranges)} rayons par scan')

def main():
    try:
        rclpy.init()
    except:
        pass
    
    node = ScanRestamperSimu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()