#!/bin/bash
# 100% LIDAR MODE - NO IMU, NO COMPLEX FUSION
# Direct: LIDAR → SLAM Toolbox
# Motor Odom only for reference (SLAM ignores it)

set -e

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "=== 100% LIDAR MODE - Starting Stack ==="
echo "NO IMU, NO EKF - Pure SLAM Toolbox from LIDAR"

# Kill previous
pkill -f "rviz2" || true
pkill -f "motor_odom" || true
pkill -f "slam_toolbox" || true
pkill -f "scan_restamper" || true
pkill -f "publish_tf" || true
pkill -f "imu_fir_filter" || true
pkill -f "simple_ekf" || true

sleep 1

# Check agent
if ! pgrep -f "micro_ros_agent" > /dev/null; then
  echo "[1/5] Starting Micro-ROS Agent..."
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
  sleep 2
else
  echo "[1/5] Agent already running ✓"
fi

echo "[2/5] Starting RViz2..."
rviz2 &
sleep 2

echo "[3/5] Starting Motor Odometry (reference only, SLAM ignores it)..."
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 motor_odom_node.py &
sleep 1

echo "[4/5] Starting Scan Restamper (fix LIDAR timestamps)..."
python3 scan_restamper.py &
sleep 1

echo "[5/5] Starting SLAM Toolbox (100% LIDAR mode)..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml \
  autostart:=true use_lifecycle_manager:=false &

sleep 3
echo ""
echo "✓ Stack started - LIDAR only mode"
echo "  No IMU, no EKF, pure SLAM matching"
echo "  Use teleop_keyboard.py to control"
