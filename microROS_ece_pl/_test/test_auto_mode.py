#!/usr/bin/env python3
"""Test autonomous mode by publishing cmd_vel commands"""
import sys
import time
sys.path.insert(0, '/home/matheo/microros_ece_ws/src/Tech_project_autonomous/microROS_ece_pl/microROS_ece_pl')

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('test_auto')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

# Send autonomous enable signal
msg = Twist()
msg.linear.x = -999.0  # Magic signal to switch modes
pub.publish(msg)
print("✅ Sent AUTO mode toggle signal")

# Let it run for 30 seconds
start = time.time()
while time.time() - start < 30:
    time.sleep(0.1)
    if (time.time() - start) % 5 < 0.1:
        print(f"⏱️  Running... {int(time.time() - start)}s")

rclpy.shutdown()
print("✅ Test complete")
