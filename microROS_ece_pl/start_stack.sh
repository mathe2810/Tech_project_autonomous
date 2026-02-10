#!/bin/bash
# Startup script - Démarre le stack de navigation complet

set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "=== Starting ROS2 Navigation Stack (Localisation + SLAM + Nav2) ==="

# Kill any previous instances
pkill -f "micro_ros_agent" || true
pkill -f "rviz2" || true
pkill -f "motor_odom" || true
pkill -f "imu_kalman_filter" || true
pkill -f "simple_slam" || true
pkill -f "kalman_filter_fusion" || true

sleep 1

# Start processes in background
echo "[1/6] Starting Micro-ROS Agent on port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!

sleep 2

echo "[2/6] Starting RViz2..."
rviz2 &
RVIZ_PID=$!

sleep 2

echo "[3/6] Starting IMU FIR Filter (Cutoff 20Hz, removes LIDAR vibrations)..."
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
# LIDAR @ 10Hz generates harmonics at 20, 30, 40, 50Hz (from FFT analysis)
# FIR cutoff at 20Hz removes all LIDAR vibrations while preserving robot dynamics
# Latency: ~100ms (acceptable for navigation)
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100 &
FILTER_PID=$!

sleep 1

echo "[4/6] Starting Motor Odometry Node..."
python3 motor_odom_node.py &
MOTOR_ODOM_PID=$!

sleep 1

echo "[5/6] Starting Simple SLAM Node..."
python3 simple_slam.py &
SLAM_PID=$!

sleep 1

echo "[6/6] Starting Kalman Filter Fusion..."
python3 kalman_filter_fusion.py &
FUSION_PID=$!

echo ""
echo "=== All processes started ==="
echo "Agent PID: $AGENT_PID"
echo "RViz PID: $RVIZ_PID"
echo "IMU Filter PID: $FILTER_PID"
echo "Motor Odom PID: $MOTOR_ODOM_PID"
echo "SLAM PID: $SLAM_PID"
echo "Fusion PID: $FUSION_PID"
echo ""
echo "Topics:"
echo "  /odom (raw motor odometry)"
echo "  /odom_filtered (fused odometry, recommended)"
echo "  /imu/data_filtered (filtered IMU)"
echo "  /map (occupancy grid)"
echo "  /tf (transforms)"
echo ""
echo "To stop: pkill -P $$ (or Ctrl+C)"
echo ""

# Wait for all processes
wait
