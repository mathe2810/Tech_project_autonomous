#!/bin/bash
# Startup script - Lance agent + RViz + IMU Filter + SLAM

set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "=== Starting ROS2 Micro-ROS SLAM Stack ==="

# Kill any previous instances
pkill -f "micro_ros_agent" || true
pkill -f "rviz2" || true
pkill -f "imu_kalman_filter" || true
pkill -f "simple_slam" || true

sleep 1

# Get the package share directory
RVIZ_CONFIG=$(ros2 pkg prefix cpp_pubsub)/share/cpp_pubsub/launch/lidar_buffer_imu.rviz

# Start processes in background
echo "[1/4] Starting Micro-ROS Agent on port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!

sleep 2

echo "[2/4] Starting RViz2..."
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

sleep 2

echo "[3/4] Starting IMU EMA Filter..."
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 imu_kalman_filter.py &
FILTER_PID=$!

sleep 1

echo "[4/4] Starting Simple SLAM Node..."
python3 simple_slam.py &
SLAM_PID=$!

echo ""
echo "=== All processes started ==="
echo "Agent PID: $AGENT_PID"
echo "RViz PID: $RVIZ_PID"
echo "IMU Filter PID: $FILTER_PID"
echo "SLAM PID: $SLAM_PID"
echo ""
echo "Topics available:"
echo "  /scan (raw LIDAR)"
echo "  /imu/data (raw IMU)"
echo "  /imu/data_filtered (filtered IMU)"
echo "  /map (occupancy grid from SLAM)"
echo "  /slam/pose (estimated pose)"
echo ""
echo "To stop: pkill -P $$ (or Ctrl+C)"
echo ""

# Wait for all processes
wait
