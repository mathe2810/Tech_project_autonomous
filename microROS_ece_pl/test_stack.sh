#!/bin/bash
# Minimal test - just launch components without waiting

cd /home/matheo/microros_ece_ws/src/Tech_project_autonomous/microROS_ece_pl/microROS_ece_pl

# Clean up any existing processes first
echo "Cleaning up old processes..."
pkill -9 -f "simu_lidar" 2>/dev/null || true
pkill -9 -f "simu_bridge" 2>/dev/null || true
pkill -9 -f "rf2o" 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "scan_restamper" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
sleep 2

echo "Starting components..."

python3 simu_lidar.py &
echo "1. Simulator: $!"

sleep 1

python3 simu_bridge.py &
echo "2. Bridge: $!"

sleep 1

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
echo "3. TF2: $!"

sleep 1

python3 scan_restamper_simu.py &
echo "4. Restamper: $!"

sleep 1

ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args --params-file config/rf2o_params.yaml &
echo "5. RF2O: $!"

sleep 2

ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file config/slam_toolbox_closed_loop.yaml &
echo "6. SLAM: $!"

echo ""
echo "All components started!"
echo ""
echo "Press Ctrl+C to exit"
wait
