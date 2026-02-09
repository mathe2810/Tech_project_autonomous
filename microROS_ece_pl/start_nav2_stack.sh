#!/bin/bash
# Start Micro-ROS + SLAM Toolbox + Nav2

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

ROOT=/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

echo "Cleaning up old processes..."
pkill -f "ros2 run micro_ros_agent" || true
pkill -f "slam_toolbox" || true
pkill -f "nav2_bringup" || true
pkill -f "cmd_vel_odom.py" || true
pkill -f "scan_frame_fix.py" || true
pkill -f "static_transform_publisher" || true
pkill -f "rviz2" || true
pkill -f "frontier_explorer.py" || true

sleep 1

echo "[1/8] Micro-ROS Agent"
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!
echo "Agent PID: $AGENT_PID"

sleep 2

echo "[2/8] Static TF base_link -> laser"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser &
TF_PID=$!
echo "TF PID: $TF_PID"

sleep 1

echo "[3/8] Scan frame fixer (/scan -> /scan_fixed)"
nohup python3 $ROOT/scan_frame_fix.py --ros-args -p input:=/scan -p output:=/scan_fixed -p frame_id:=laser > /tmp/scan_frame_fix.log 2>&1 &
SCANFIX_PID=$!
echo "ScanFix PID: $SCANFIX_PID"

sleep 2

echo "[4/8] Odom from /cmd_vel"
nohup python3 $ROOT/cmd_vel_odom.py > /tmp/cmd_vel_odom.log 2>&1 &
ODOM_PID=$!
echo "Odom PID: $ODOM_PID"

sleep 1

echo "[5/8] SLAM Toolbox"
ros2 launch slam_toolbox online_async_launch.py params_file:=$ROOT/config/slam_toolbox_params.yaml &
SLAM_PID=$!
echo "SLAM PID: $SLAM_PID"

sleep 2

echo "[6/8] Nav2"
ros2 launch nav2_bringup navigation_launch.py params_file:=$ROOT/config/nav2_params_simple.yaml use_sim_time:=false autostart:=true &
NAV2_PID=$!
echo "Nav2 PID: $NAV2_PID"

sleep 2

echo "[7/8] RViz2"
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz &
RVIZ_PID=$!
echo "RViz PID: $RVIZ_PID"

sleep 1

echo "[8/8] Frontier Explorer (Autonomous Exploration)"
nohup python3 $ROOT/frontier_explorer.py > /tmp/frontier_explorer.log 2>&1 &
FRONTIER_PID=$!
echo "Frontier PID: $FRONTIER_PID"

echo ""
echo "=== Started Nav2 Stack with Autonomous Exploration ==="
echo "Agent PID: $AGENT_PID"
echo "TF PID: $TF_PID"
echo "ScanFix PID: $SCANFIX_PID"
echo "Odom PID: $ODOM_PID"
echo "SLAM PID: $SLAM_PID"
echo "Nav2 PID: $NAV2_PID"
echo "RViz PID: $RVIZ_PID"
echo "Frontier Explorer PID: $FRONTIER_PID"
echo ""
echo "🚀 Robot is exploring autonomously - watch the map in RViz!"
echo ""
echo "Press Ctrl+C to stop all processes"

# Wait for Ctrl+C
wait
