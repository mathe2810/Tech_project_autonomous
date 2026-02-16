#!/usr/bin/env python3
"""
Simple manual test: tu contrôles le robot avec:
W = forward
A = rotate left  
D = rotate right
S = backward
SPACE = stop
Q = quit

Observe dans RViz:
- La carte (costmap) devrait rester FIXE
- Le robot devrait bouger en avant/arrière/rotation
- SI la carte TOURNE avec le robot → YAW DIVERGENCE (mapping broken)
- SI la carte reste immobile → MAPPING WORKING!
"""

import rclpy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        return ch
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

def main():
    rclpy.init()
    node = rclpy.create_node('manual_test')
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    print("""
╔════════════════════════════════════════╗
║  100% LIDAR MODE - MANUAL TEST         ║
╚════════════════════════════════════════╝

Controls:
  W = Forward (slow)
  A = Rotate Left (VERY SLOW for SLAM)
  D = Rotate Right (VERY SLOW for SLAM)  
  S = Backward (slow)
  SPACE = Stop
  Q = Quit

Test Sequence:
1. Go forward 2m
2. Rotate 180° SLOWLY (watch map!)
3. Go backward 2m
4. Check: robot back at start? Map still aligned with room?

KEY OBSERVATION:
- If map ROTATES → SLAM can't track yaw (BROKEN)
- If map STAYS PUT → SLAM yaw tracking working (GOOD!)

Watch RViz carefully during rotation!
    """)
    
    twist = Twist()
    
    while True:
        key = get_key().lower()
        
        if key == 'w':
            twist.linear.x = 0.50  # FULL SPEED: test if it's a power issue
            twist.angular.z = 0.0
            print("→ Forward 0.5 m/s")
        elif key == 's':
            twist.linear.x = -0.50  # FULL SPEED
            twist.angular.z = 0.0
            print("← Backward 0.5 m/s")
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = 0.50  # FULL SPEED ROTATION: test if rotation needs more power
            print("↻ Rotate LEFT 0.50 rad/s (28°/s)")
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -0.50  # FULL SPEED ROTATION
            print("↺ Rotate RIGHT 0.50 rad/s (-28°/s)")
        elif key == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print("⏹ STOP")
        elif key == 'q':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            print("✓ Quitting")
            break
        else:
            continue
        
        cmd_vel_pub.publish(twist)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
