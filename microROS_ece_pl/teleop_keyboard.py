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
║  E      : Forward (min speed)          ║
║  X      : Backward (min speed)         ║
║  J      : Rotate Left (min speed)      ║
║  L      : Rotate Right (min speed)     ║
║  F      : Forward (lidar_gap_follower) ║
║  C      : Rotate (lidar_gap_follower)  ║
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

            # Extra test keys for minimum values
            elif key.upper() == 'E':
                twist.linear.x = 0.15  # Minimum forward speed
                twist.angular.z = 0.0
                print("→ FORWARD (min speed)")

            elif key.upper() == 'X':
                twist.linear.x = -0.15  # Minimum backward speed
                twist.angular.z = 0.0
                print("← BACKWARD (min speed)")

            elif key.upper() == 'J':
                twist.linear.x = 0.0
                twist.angular.z = 1.1  # Minimum left rotation (matches lidar_gap_follower near obstacle)
                print("↻ ROTATE LEFT (min speed)")

            elif key.upper() == 'L':
                twist.linear.x = 0.0
                twist.angular.z = -1.1  # Minimum right rotation
                print("↺ ROTATE RIGHT (min speed)")

            # Keys for lidar_gap_follower typical values
            elif key.upper() == 'F':
                twist.linear.x = 0.15  # Typical max_linear_mps from lidar_gap_follower
                twist.angular.z = 0.0
                print("→ FORWARD (lidar_gap_follower)")

            elif key.upper() == 'C':
                twist.linear.x = 0.15  # Near obstacle min_linear_mps
                twist.angular.z = 1.3  # Near obstacle max_angular_rps
                print("↻ ROTATE (lidar_gap_follower)")

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
