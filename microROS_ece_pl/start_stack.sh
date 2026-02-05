#!/bin/bash
# Startup script - Lance agent + RViz + IMU Kalman Filter

set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "=== Starting ROS2 Micro-ROS Stack ==="

# Kill any previous instances
pkill -f "micro_ros_agent" || true
pkill -f "rviz2" || true
pkill -f "imu_kalman_filter" || true

sleep 1

# Get the package share directory
RVIZ_CONFIG=$(ros2 pkg prefix cpp_pubsub)/share/cpp_pubsub/launch/lidar_buffer_imu.rviz

# Start processes in background
echo "[1/3] Starting Micro-ROS Agent on port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!

sleep 2

echo "[2/3] Starting RViz2..."
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

sleep 2

echo "[3/3] Starting IMU Kalman Filter..."
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 imu_kalman_filter.py &
FILTER_PID=$!

echo ""
echo "=== All processes started ==="
echo "Agent PID: $AGENT_PID"
echo "RViz PID: $RVIZ_PID"
echo "Filter PID: $FILTER_PID"
echo ""
echo "To stop: pkill -P $$ (or Ctrl+C)"
echo ""

# Wait for all processes
wait
