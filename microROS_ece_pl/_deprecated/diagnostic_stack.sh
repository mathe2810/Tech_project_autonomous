#!/bin/bash
# Diagnostic: Test each component one by one

set -e
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

echo "=== Diagnostic Stack Test ==="
echo ""

# Kill previous
pkill -9 python3 || true
pkill -9 ros2 || true
sleep 2

# TEST 1: SLAM alone
echo "[TEST 1] Starting SLAM ONLY..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml \
  autostart:=true use_lifecycle_manager:=false &
SLAM_PID=$!
sleep 5

echo "Checking /map..."
if ros2 topic list | grep -q "^/map$"; then
  echo "✓ /map exists with SLAM alone"
else
  echo "✗ /map MISSING with SLAM alone - PROBLEM!"
  kill $SLAM_PID
  exit 1
fi

echo ""
echo "[TEST 2] Adding Motor Odometry..."
python3 motor_odom_node.py &
MOTOR_PID=$!
sleep 3

echo "Checking /map still exists..."
if ros2 topic list | grep -q "^/map$"; then
  echo "✓ /map still exists after adding motor_odom"
else
  echo "✗ /map DISAPPEARED after motor_odom - PROBLEM IN motor_odom_node.py!"
  kill $SLAM_PID $MOTOR_PID
  exit 1
fi

echo ""
echo "[TEST 3] Adding Scan Restamper..."
python3 scan_restamper.py &
SCAN_PID=$!
sleep 3

echo "Checking /map still exists..."
if ros2 topic list | grep -q "^/map$"; then
  echo "✓ /map still exists after adding scan_restamper"
else
  echo "✗ /map DISAPPEARED after scan_restamper - PROBLEM IN scan_restamper.py!"
  kill $SLAM_PID $MOTOR_PID $SCAN_PID
  exit 1
fi

echo ""
echo "✓ ALL TESTS PASSED - Stack components are OK"
echo ""
echo "Processes running:"
ps aux | grep -E "motor_odom|slam|scan_restamper" | grep -v grep
