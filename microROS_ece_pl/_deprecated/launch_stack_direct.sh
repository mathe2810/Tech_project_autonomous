#!/bin/bash
# Direct launch without package discovery
# Uses absolute paths and direct node launching

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo ""
echo "  • Starting Lifecycle Manager..."

# Lifecycle manager
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args \
  -p node_names:="['motor_odom','scan_restamper','simple_ekf','slam_toolbox']" \
  -p autostart:=true \
  -p node_active_states:="['active']" \
  &
LCM_PID=$!
sleep 2

echo "  • Starting Motor Odometry..."
python3 /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/motor_odom_lifecycle.py &
MOTOR_PID=$!
sleep 1

echo "  • Starting Scan Restamper..."
python3 /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/scan_restamper_lifecycle.py &
SCAN_PID=$!
sleep 1

echo "  • Starting Odometry Relay..."
python3 /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/simple_ekf_lifecycle.py &
RELAY_PID=$!
sleep 1

echo "  • Static TF (base_link -> laser_link)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
TF_PID=$!
sleep 1

echo "  • Starting SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p slam_params_file:=/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/config/slam_toolbox_params.yaml \
  &
SLAM_PID=$!
sleep 2

echo ""
echo "=== Stack Ready ==="
echo "PIDs:"
echo "  Lifecycle Manager: $LCM_PID"
echo "  Motor Odom: $MOTOR_PID"
echo "  Scan Restamper: $SCAN_PID"
echo "  Odom Relay: $RELAY_PID"
echo "  SLAM Toolbox: $SLAM_PID"
echo ""
echo "Lifecycle transitions happening automatically..."
echo "Press Ctrl+C to stop"
echo ""

# Cleanup on exit
trap "
  echo ''
  echo 'Shutting down...'
  kill $LCM_PID 2>/dev/null || true
  kill $MOTOR_PID 2>/dev/null || true
  kill $SCAN_PID 2>/dev/null || true
  kill $RELAY_PID 2>/dev/null || true
  kill $TF_PID 2>/dev/null || true
  kill $SLAM_PID 2>/dev/null || true
  wait
  echo 'Stack stopped'
" SIGINT SIGTERM

# Wait for all processes
wait
