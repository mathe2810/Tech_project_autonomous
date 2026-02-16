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
pkill -f "slam_toolbox" || true
pkill -f "slam_async_launch" || true

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

echo "[3/6] Starting Motor Odometry Node..."
python3 motor_odom_node.py &
MOTOR_ODOM_PID=$!

sleep 1

echo "[4/6] Starting Scan Restamper (fix timestamp mismatch)..."
python3 scan_restamper.py &
SCAN_RESTAMPER_PID=$!

sleep 1

echo "[5/6] Publishing static TF: laser_link -> base_link..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
TF_STATIC_PID=$!

sleep 1

echo "[6/6] Starting SLAM Toolbox (Official ROS2 library, LIDAR-only mode)..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml autostart:=true use_lifecycle_manager:=false &
SLAM_PID=$!

echo "[6.5/6] Starting Odometry Relay (NO IMU fusion - 100% LIDAR mode)..."
python3 simple_ekf.py &
EKF_PID=$!

echo "[6.7/6] Starting SLAM Stabilizer (detects drift, auto-calibrates)..."
python3 slam_stabilizer.py &
STABILIZER_PID=$!

echo ""
echo "=== All processes started ==="
echo "Agent PID: $AGENT_PID"
echo "RViz PID: $RVIZ_PID"
echo "Motor Odom PID: $MOTOR_ODOM_PID"
echo "Scan Restamper PID: $SCAN_RESTAMPER_PID"
echo "SLAM Toolbox PID: $SLAM_PID"
echo "Odom Relay PID: $EKF_PID"
echo "SLAM Stabilizer PID: $STABILIZER_PID"
echo ""
echo "SYSTEM MODE: 100% LIDAR + Motor Odometry + AUTO-STABILIZATION"
echo "Topics:"
echo "  /scan (LIDAR scans @ 10Hz)"
echo "  /odom (motor odometry - HIGH covariance)"
echo "  /odom_filtered (relay of motor odom, for compatibility)"
echo "  /map (SLAM Toolbox occupancy grid - STATIC!)"
echo "  /slam_toolbox/pose (SLAM estimated pose)"
echo "  /tf (coordinate frames: map -> odom -> base_link)"
echo ""
echo "CALIBRATION ROUTINES (run in separate terminal):"
echo "  python3 calibration_routine.py --mode convergence_dance    (full sequence)"
echo "  python3 calibration_routine.py --mode scan_calibration     (in-place rotation)"
echo "  python3 calibration_routine.py --mode micro_oscillation    (gentle movement)"
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
  kill $STABILIZER_PID 2>/dev/null || true
  sleep 0.5
  kill $RVIZ_PID 2>/dev/null || true
  
  sleep 1
  pkill -9 -P $$ 2>/dev/null || true
  
  echo "=== Stack stopped ==="
  exit 0
}

trap cleanup SIGINT SIGTERM
wait
