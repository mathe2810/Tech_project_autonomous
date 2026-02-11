#!/bin/bash
# Startup script - ROS2 Stack with SLAM Toolbox + Nav2
# NOTE: Does NOT restart micro_ros_agent (ESP32 connection stays alive)
#       Uses official ROS2 SLAM Toolbox instead of custom SLAM

set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "=== Starting ROS2 Stack (SLAM Toolbox + Nav2) ==="
echo "NOTE: Micro-ROS Agent NOT restarted (ESP32 connection preserved)"

# Kill only ROS2 nodes (NOT the agent!)
pkill -f "rviz2" || true
pkill -f "motor_odom" || true
pkill -f "imu_fir_filter" || true
pkill -f "slam_toolbox" || true
pkill -f "slam_async_launch" || true
pkill -f "kalman_filter_fusion" || true

sleep 1

# Check if agent is running
if ! pgrep -f "micro_ros_agent" > /dev/null; then
  echo "[1/7] Starting Micro-ROS Agent on port 8888..."
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
  AGENT_PID=$!
  sleep 2
else
  echo "[1/7] Micro-ROS Agent already running ✓"
fi

echo "[2/7] Starting RViz2..."
rviz2 &
RVIZ_PID=$!

sleep 2

echo "[3/7] Starting IMU FIR Filter (Cutoff 20Hz, removes LIDAR vibrations)..."
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100 &
IMU_FILTER_PID=$!

sleep 1

echo "[4/7] Starting Motor Odometry Node..."
python3 motor_odom_node.py &
MOTOR_ODOM_PID=$!

sleep 1

echo "[4.2/7] Starting Scan Restamper (fix timestamp mismatch)..."
python3 scan_restamper.py &
SCAN_RESTAMPER_PID=$!

sleep 1

echo "[4.5/7] Publishing static TF: laser_link -> base_link..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
TF_STATIC_PID=$!

sleep 1

echo "[5/7] Starting SLAM Toolbox (Official ROS2 library)..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml autostart:=true use_lifecycle_manager:=false &
SLAM_PID=$!

echo "[6/7] Starting Simple EKF (fuses odom + IMU)..."
python3 simple_ekf.py &
EKF_PID=$!

echo ""
echo "=== All processes started ==="
echo "Agent PID: $AGENT_PID"
echo "RViz PID: $RVIZ_PID"
echo "IMU Filter PID: $FILTER_PID"
echo "Motor Odom PID: $MOTOR_ODOM_PID"
echo "SLAM Toolbox PID: $SLAM_PID"
echo "Fusion PID: $FUSION_PID"
echo ""
echo "Topics:"
echo "  /scan (LIDAR scans)"
echo "  /odom (motor odometry)"
echo "  /odom_filtered (fused odometry)"
echo "  /imu/data_filtered (filtered IMU)"
echo "  /map (SLAM Toolbox occupancy grid - STATIC!)"
echo "  /slam_toolbox/pose (SLAM estimated pose)"
echo "  /tf (coordinate frames)"
echo ""
echo "To stop: Ctrl+C or pkill -P $$"
echo ""

# Signal handler for graceful shutdown
cleanup() {
  echo ""
  echo "=== Shutting down stack ==="
  kill $FUSION_PID 2>/dev/null || true
  sleep 0.5
  kill $SLAM_PID 2>/dev/null || true
  sleep 0.5
  kill $MOTOR_ODOM_PID 2>/dev/null || true
  sleep 0.5
  kill $SCAN_RESTAMPER_PID 2>/dev/null || true
  sleep 0.5
  kill $TF_STATIC_PID 2>/dev/null || true
  sleep 0.5
  kill $IMU_FILTER_PID 2>/dev/null || true
  sleep 0.5
  kill $EKF_PID 2>/dev/null || true
  sleep 0.5
  kill $RVIZ_PID 2>/dev/null || true
  
  sleep 1
  pkill -9 -P $$ 2>/dev/null || true
  
  echo "=== Stack stopped ==="
  exit 0
}

trap cleanup SIGINT SIGTERM
wait
