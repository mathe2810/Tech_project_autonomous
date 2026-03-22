#!/usr/bin/env python3
"""
Script pour arrêter le robot après la sortie d'un node.
Publie STOP sur /cmd_vel (Twist).
"""
import rclpy
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = rclpy.create_node('stop_cmd_vel')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    stop = Twist()
    stop.linear.x = 0.0
    stop.angular.z = 0.0
    for _ in range(10):
        pub.publish(stop)
        print('⊙ STOP')
        import time
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
