#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_sub')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.count = 0
        self.get_logger().info('Waiting for /scan...')

    def callback(self, msg):
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f'✅ Received {self.count} messages from /scan: {len(msg.ranges)} rays, angle_max={msg.angle_max:.4f}')

def main():
    rclpy.init()
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
