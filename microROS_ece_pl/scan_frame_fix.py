#!/usr/bin/env python3
"""
Republish /scan with a fixed frame_id for Nav2/SLAM Toolbox.
Input: /scan_raw -> Output: /scan
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFix(Node):
    def __init__(self):
        super().__init__('scan_frame_fix')
        self.declare_parameter('input', '/scan')
        self.declare_parameter('output', '/scan_fixed')
        self.declare_parameter('frame_id', 'laser')

        self.input_topic = self.get_parameter('input').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(f'Subscribing to {self.input_topic}, publishing {self.output_topic}')
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

    def cb(self, msg: LaserScan):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFix()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
