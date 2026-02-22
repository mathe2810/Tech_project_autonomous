#!/usr/bin/env python3
"""
Keyboard teleop - control robot with WASD keys.
"""

import rclpy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Get a single key press"""
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        return ch
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

def main():
    rclpy.init()
    node = rclpy.create_node('teleop')
    
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    print("""
╔════════════════════════════════════════╗
║  ROBOT TELEOP - Keyboard Control       ║
╠════════════════════════════════════════╣
║  W / ↑  : Forward                      ║
║  S / ↓  : Backward                     ║
║  A / ←  : Rotate Left                  ║
║  D / →  : Rotate Right                 ║
║  SPACE  : Stop                         ║
║  Q      : Quit                         ║
╚════════════════════════════════════════╝
    """)
    
    twist = Twist()
    
    try:
        while True:
            key = get_key()
            
            if key.upper() == 'W':
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                print("→ FORWARD")
            
            elif key.upper() == 'S':
                twist.linear.x = -0.5
                twist.angular.z = 0.0
                print("← BACKWARD")
            
            elif key.upper() == 'A':
                twist.linear.x = 0.0
                twist.angular.z = 1.3
                print("↻ ROTATE LEFT")
            
            elif key.upper() == 'D':
                twist.linear.x = 0.0
                twist.angular.z = -1.3
                print("↺ ROTATE RIGHT")
            
            elif key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("⊙ STOP")
            
            elif key.upper() == 'Q':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)
                print("Quitting...")
                break
            
            else:
                continue
            
            cmd_vel_pub.publish(twist)
    
    except KeyboardInterrupt:
        print("\nStopping...")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
