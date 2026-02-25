#!/usr/bin/env python3
"""Teleop keyboard adapted for weak motors (min angular = 1.0 rad/s)"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class RobotTeleopWeak(Node):
    def __init__(self):
        super().__init__('robot_teleop_weak')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Speeds adapted to motor limits (tested: 1.5 rad/s works best)
        self.linear_speed = 0.15    # m/s - 15 cm/s
        self.angular_speed = 1.5    # rad/s - OPTIMAL for these motors
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Robot Teleop - Weak Motors Mode')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  i/k : Forward/Backward (0.15 m/s)')
        self.get_logger().info('  j/l : Rotate Left/Right (1.0 rad/s = 57°/s)')
        self.get_logger().info('  u/o : Forward + Rotate')
        self.get_logger().info('  m/. : Backward + Rotate')
        self.get_logger().info('  SPACE : Stop')
        self.get_logger().info('  +/- : Adjust speeds')
        self.get_logger().info('  q : Quit')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Current: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
        self.get_logger().info('⚡ Note: 1.5 rad/s is optimal for these motors (tested)')
        self.get_logger().info('=' * 60)
        
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.publisher.publish(twist)
    
    def update_speeds(self, linear, angular):
        self.current_linear = linear * self.linear_speed
        self.current_angular = angular * self.angular_speed
        
        if angular != 0:
            status = "Rotating" if linear == 0 else "Arc"
            self.get_logger().info(f'{status}: linear={self.current_linear:.2f} m/s, angular={self.current_angular:.2f} rad/s')
        elif linear != 0:
            self.get_logger().info(f'Straight: {self.current_linear:.2f} m/s')
        else:
            self.get_logger().info('STOP')
    
    def increase_speeds(self):
        self.linear_speed = min(0.30, self.linear_speed + 0.02)
        self.angular_speed = min(1.8, self.angular_speed + 0.1)
        self.get_logger().info(f'Speed UP: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
    
    def decrease_speeds(self):
        self.linear_speed = max(0.05, self.linear_speed - 0.02)
        self.angular_speed = max(1.0, self.angular_speed - 0.1)  # Never below 1.0!
        self.get_logger().info(f'Speed DOWN: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
        if self.angular_speed <= 1.0:
            self.get_logger().info('⚠️  Angular at minimum (1.0 rad/s) - not optimal, 1.5 preferred!')

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    teleop = RobotTeleopWeak()
    
    key_bindings = {
        'i': (1, 0),    # forward
        'k': (-1, 0),   # backward
        'j': (0, 1),    # rotate left
        'l': (0, -1),   # rotate right
        'u': (1, 1),    # forward + left
        'o': (1, -1),   # forward + right
        'm': (-1, 1),   # backward + left
        '.': (-1, -1),  # backward + right
        ' ': (0, 0),    # stop
    }
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key == 'q':
                break
            elif key == '+' or key == '=':
                teleop.increase_speeds()
            elif key == '-' or key == '_':
                teleop.decrease_speeds()
            elif key in key_bindings:
                linear, angular = key_bindings[key]
                teleop.update_speeds(linear, angular)
            
            rclpy.spin_once(teleop, timeout_sec=0)
    
    except Exception as e:
        print(f'Error: {e}')
    
    finally:
        # Stop robot
        teleop.update_speeds(0, 0)
        time.sleep(0.2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import time
    main()
