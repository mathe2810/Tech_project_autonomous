#!/bin/bash
# Simplified launch without lifecycle manager
# Direct node launching in correct order

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo ""
echo "🚀 Starting SLAM Stack (Simple Mode)"
echo ""

# Kill any existing processes
pkill -9 -f "ros2 run" 2>/dev/null || true
pkill -9 python3 2>/dev/null || true
sleep 1

echo "  1️⃣  Starting Static TF (base_link -> laser_link)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
TF_PID=$!
sleep 1

echo "  2️⃣  Starting Motor Odometry..."
python3 /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/motor_odom_lifecycle.py &
MOTOR_PID=$!
sleep 2

echo "  3️⃣  Starting Scan Restamper..."
python3 /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/scan_restamper_lifecycle.py &
SCAN_PID=$!
sleep 2

echo "  4️⃣  Starting Odometry Relay..."
python3 /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/simple_ekf_lifecycle.py &
RELAY_PID=$!
sleep 2

echo "  5️⃣  Starting SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p slam_params_file:=/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/config/slam_toolbox_params.yaml \
  &
SLAM_PID=$!
sleep 3

echo ""
echo "✅ Stack Ready ==="
echo ""
echo "Running PIDs:"
echo "  TF: $TF_PID"
echo "  Motor Odom: $MOTOR_PID"
echo "  Scan Restamper: $SCAN_PID"
echo "  Odom Relay: $RELAY_PID"
echo "  SLAM Toolbox: $SLAM_PID"
echo ""
echo "Topics available:"
ros2 topic list | grep -E "(scan|odom|map|tf)" || echo "  (waiting for nodes...)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Cleanup on exit
trap "
  echo ''
  echo '⏹️  Shutting down...'
  kill $TF_PID 2>/dev/null || true
  kill $MOTOR_PID 2>/dev/null || true
  kill $SCAN_PID 2>/dev/null || true
  kill $RELAY_PID 2>/dev/null || true
  kill $SLAM_PID 2>/dev/null || true
  sleep 1
  echo '✓ All processes stopped'
" SIGINT SIGTERM

# Keep running
wait
