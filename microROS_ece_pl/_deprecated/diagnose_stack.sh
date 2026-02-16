#!/bin/bash
# Diagnostic: Launch SLAM + Motor Odom and see what happens

set -e
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

echo "=== Diagnostic: Check what's publishing ==="

# Terminal 1: Motor Odom
echo "[1] Starting Motor Odometry in background..."
python3 motor_odom_node.py &
MOTOR_PID=$!
sleep 2

# Terminal 2: Scan Restamper
echo "[2] Starting Scan Restamper in background..."
python3 scan_restamper.py &
SCAN_PID=$!
sleep 2

# Terminal 3: SLAM in background
echo "[3] Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml \
  autostart:=true use_lifecycle_manager:=false &
SLAM_PID=$!
sleep 5

# Check topics
echo ""
echo "=== CHECKING PUBLISHED TOPICS ==="
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

ros2 topic list 2>/dev/null | grep -E "scan|odom|map" || echo "No topics found"

echo ""
echo "=== CHECKING MAP ==="
ros2 topic info /map 2>/dev/null || echo "No /map topic"

echo ""
echo "=== All processes running ==="
ps aux | grep -E "motor_odom|slam|scan_restamper" | grep -v grep

echo ""
echo "Press Ctrl+C to kill all"
wait
