#!/usr/bin/env python3
"""
SLAM Calibration Routine - Performs controlled movements to help SLAM converge.
Small, slow, predictable movements that give SLAM good data without jerky motions.

Routines:
  - micro_oscillation: ±3cm forward/backward (very gentle)
  - slow_spiral: Slow expanding spiral
  - scan_calibration: Static position scans (just rotate in place)
  - convergence_dance: Figure-8 at very low speed

Usage:
  python3 calibration_routine.py --mode micro_oscillation
  python3 calibration_routine.py --mode scan_calibration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import argparse

class CalibrationRoutine(Node):
    def __init__(self):
        super().__init__('calibration_routine')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Calibration Routine ready")
    
    def publish_movement(self, vx: float, wz: float, duration: float, step: float = 0.05):
        """Publish velocity command for specified duration"""
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(step)
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.1)
    
    def micro_oscillation(self, cycles: int = 5):
        """
        Very small forward/backward oscillation (±3cm).
        Helps SLAM refine without accumulating error.
        Duration: ~2 minutes per 5 cycles
        """
        self.get_logger().info(f"Starting MICRO OSCILLATION ({cycles} cycles)...")
        
        for cycle in range(cycles):
            self.get_logger().info(f"  Cycle {cycle+1}/{cycles}")
            
            # Forward 3cm/s for 5 seconds = 15cm
            self.publish_movement(vx=0.03, wz=0.0, duration=5.0)
            time.sleep(0.5)
            
            # Backward 3cm/s for 5 seconds = 15cm (back to start)
            self.publish_movement(vx=-0.03, wz=0.0, duration=5.0)
            time.sleep(0.5)
        
        self.get_logger().info("✓ Micro oscillation complete")
    
    def slow_spiral(self, duration: float = 60.0, cycles: int = 1):
        """
        Slow expanding spiral - covers area while building dense scan map.
        Good for initial mapping and convergence.
        """
        self.get_logger().info(f"Starting SLOW SPIRAL ({duration:.0f}s)...")
        
        for cycle in range(cycles):
            start = time.time()
            radius = 0.2  # Start at 20cm
            
            while time.time() - start < duration:
                elapsed = time.time() - start
                # Slowly expand radius
                current_radius = radius + (elapsed / duration) * 0.3  # max 50cm
                
                # Move in circle
                forward_speed = 0.04  # 4cm/s
                angular_speed = forward_speed / max(current_radius, 0.1)
                
                self.publish_movement(
                    vx=forward_speed,
                    wz=angular_speed,
                    duration=0.5,
                    step=0.05
                )
        
        self.get_logger().info("✓ Slow spiral complete")
    
    def scan_calibration(self, duration: float = 30.0, samples: int = 6):
        """
        Stay in place and slowly rotate to collect full scans.
        Pure rotation - SLAM can focus on heading alignment.
        """
        self.get_logger().info(f"Starting SCAN CALIBRATION ({duration:.0f}s, {samples} rotations)...")
        
        # Slow rotation: 360deg / samples rotations, spread over duration
        time_per_rotation = duration / samples
        angular_speed = (2.0 * math.pi) / time_per_rotation  # rad/s
        
        for sample in range(samples):
            self.get_logger().info(f"  Rotation {sample+1}/{samples}")
            self.publish_movement(vx=0.0, wz=angular_speed, duration=time_per_rotation)
            time.sleep(0.3)
        
        self.get_logger().info("✓ Scan calibration complete")
    
    def figure_eight(self, duration: float = 90.0):
        """
        Figure-8 pattern at very low speed.
        Combines forward/backward with rotations - good for final refinement.
        """
        self.get_logger().info(f"Starting FIGURE-8 ({duration:.0f}s)...")
        
        start = time.time()
        while time.time() - start < duration:
            elapsed = time.time() - start
            t = elapsed / 30.0  # 30 second period for one figure-8
            
            # Figure-8 parameterization
            x_progress = math.sin(2 * math.pi * t) * 0.5  # Oscillate in x
            theta = 2 * math.pi * t
            
            # Combine forward and rotation
            forward_speed = 0.03 * (1 + 0.5 * math.cos(2 * math.pi * t))
            angular_speed = 0.3 * math.sin(2 * math.pi * t)
            
            self.publish_movement(
                vx=forward_speed,
                wz=angular_speed,
                duration=0.5,
                step=0.05
            )
        
        self.get_logger().info("✓ Figure-8 complete")
    
    def convergence_dance(self):
        """
        Sequence designed to help SLAM converge quickly.
        Combines all calibration elements in optimal order.
        """
        self.get_logger().info("🎯 Starting CONVERGENCE DANCE (full sequence)...")
        
        # Phase 1: Micro oscillation (build dense local map)
        self.get_logger().info("\n[Phase 1/3] Building local density...")
        self.micro_oscillation(cycles=3)
        time.sleep(1.0)
        
        # Phase 2: Scan calibration (align heading)
        self.get_logger().info("\n[Phase 2/3] Aligning heading...")
        self.scan_calibration(duration=45.0, samples=4)
        time.sleep(1.0)
        
        # Phase 3: Figure-8 (final refinement)
        self.get_logger().info("\n[Phase 3/3] Final refinement...")
        self.figure_eight(duration=60.0)
        
        self.get_logger().info("\n✓ CONVERGENCE DANCE COMPLETE - SLAM should be stable")

def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line
    parser = argparse.ArgumentParser(description='SLAM Calibration Routine')
    parser.add_argument('--mode', 
                       choices=['micro_oscillation', 'slow_spiral', 'scan_calibration', 
                               'figure_eight', 'convergence_dance'],
                       default='convergence_dance',
                       help='Calibration mode to run')
    parser.add_argument('--duration', type=float, default=60.0, help='Duration in seconds')
    parser.add_argument('--cycles', type=int, default=3, help='Number of cycles')
    
    args_parsed = parser.parse_args()
    
    # Create node and run
    node = CalibrationRoutine()
    
    try:
        if args_parsed.mode == 'micro_oscillation':
            node.micro_oscillation(cycles=args_parsed.cycles)
        elif args_parsed.mode == 'slow_spiral':
            node.slow_spiral(duration=args_parsed.duration, cycles=args_parsed.cycles)
        elif args_parsed.mode == 'scan_calibration':
            node.scan_calibration(duration=args_parsed.duration)
        elif args_parsed.mode == 'figure_eight':
            node.figure_eight(duration=args_parsed.duration)
        elif args_parsed.mode == 'convergence_dance':
            node.convergence_dance()
    
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted")
    finally:
        # Ensure robot stops
        msg = Twist()
        node.cmd_vel_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
