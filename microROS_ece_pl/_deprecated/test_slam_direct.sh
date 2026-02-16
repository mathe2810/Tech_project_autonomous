#!/bin/bash
# Test direct: launch SLAM binary directly with parameters

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

echo "Killing old processes..."
pkill -9 python3 || true
pkill -9 ros2 || true
pkill -9 slam || true
sleep 2

echo "Starting motor_odom..."
python3 motor_odom_node.py > /tmp/motor.log 2>&1 &
MOTOR_PID=$!
sleep 2

echo "Starting scan_restamper..."
python3 scan_restamper.py > /tmp/scan.log 2>&1 &
SCAN_PID=$!
sleep 2

echo "Starting SLAM directly..."
/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node \
  --ros-args \
  -p use_sim_time:=false \
  -p scan_topic:=/scan \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p publish_occupancy_grid:=true \
  --params-file $(pwd)/config/slam_toolbox_params.yaml \
  > /tmp/slam_direct.log 2>&1 &
SLAM_PID=$!

echo "Waiting for /map..."
sleep 5

echo ""
echo "Checking topics..."
ros2 topic list | grep -E "map|scan|odom"

echo ""
echo "Checking /map frequency..."
timeout 3 ros2 topic hz /map || echo "No /map updates"

echo ""
echo "Checking /map data..."
ros2 topic echo /map --once 2>&1 | head -10 || echo "No /map data"
