#!/usr/bin/env python3
"""Debug Ground Truth odometry messages"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import math

class DebugGTListener(Node):
    def __init__(self):
        super().__init__('debug_gt_listener')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sub = self.create_subscription(Odometry, '/odom_ground_truth', self.callback, qos)
        self.count = 0
        self.get_logger().info('🔍 Listening to /odom_ground_truth...')
        
    def callback(self, msg):
        self.count += 1
        if self.count % 50 == 0:  # Print every 50 messages
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # Convert quaternion to yaw
            yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
            yaw_deg = math.degrees(yaw)
            
            self.get_logger().info(
                f"Message #{self.count}: x={x:8.3f}m  y={y:8.3f}m  θ={yaw_deg:7.1f}°  "
                f"q=[{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]"
            )

def main():
    rclpy.init()
    node = DebugGTListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
