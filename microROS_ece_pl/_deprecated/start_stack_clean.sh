#!/bin/bash
# 100% LIDAR MODE - Launch in separate processes
# Each process is independent and won't kill others

set -e

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

echo "=== 100% LIDAR MODE - Starting processes independently ==="

# Kill previous
pkill -9 -f "motor_odom_node\|slam_toolbox\|scan_restamper" 2>/dev/null || true
sleep 2

# Check agent
if ! pgrep -f "micro_ros_agent" > /dev/null; then
  echo "[1] Starting Micro-ROS Agent..."
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 2>&1 | tee /tmp/agent.log &
  sleep 2
else
  echo "[1] Agent already running"
fi

# Launch motor_odom in separate shell
echo "[2] Starting Motor Odometry..."
bash -c "
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 motor_odom_node.py 2>&1 | tee /tmp/motor_odom.log
" &
sleep 1

# Launch scan_restamper in separate shell
echo "[3] Starting Scan Restamper..."
bash -c "
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 scan_restamper.py 2>&1 | tee /tmp/scan_restamper.log
" &
sleep 1

# Launch SLAM in separate shell
echo "[4] Starting SLAM Toolbox..."
bash -c "
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml \
  autostart:=true use_lifecycle_manager:=false 2>&1 | tee /tmp/slam.log
" &
sleep 3

echo ""
echo "✓ All processes launched independently"
echo ""
echo "Logs available:"
echo "  /tmp/agent.log       - Micro-ROS Agent"
echo "  /tmp/motor_odom.log  - Motor Odometry"
echo "  /tmp/scan_restamper.log - Scan Restamper"
echo "  /tmp/slam.log        - SLAM Toolbox"
echo ""
echo "Monitor them with: tail -f /tmp/motor_odom.log"
