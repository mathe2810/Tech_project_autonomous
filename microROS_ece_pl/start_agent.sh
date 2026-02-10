#!/bin/bash
# Launch ONLY the Micro-ROS Agent
# Run this ONCE before start_stack.sh
# The agent maintains the ESP32 connection - do NOT kill it!

set -e

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "=== Starting Micro-ROS Agent (ESP32 Connection) ==="
echo "This maintains the ESP32 connection. Do NOT restart this unless needed!"
echo "To restart ROS2 nodes only, use: ./start_stack.sh"
echo ""

# Kill only if already running
pkill -f "micro_ros_agent" || true
sleep 1

echo "Starting agent on UDP port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Agent runs forever
