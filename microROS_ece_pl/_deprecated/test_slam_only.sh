#!/bin/bash
# Ultra simple test: Just SLAM, verify /map is published

set -e
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

echo "=== STEP 1: Launch SLAM ONLY ==="
echo "Launching SLAM Toolbox..."

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml \
  autostart:=true use_lifecycle_manager:=false
