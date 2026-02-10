#!/bin/bash
# Launch IMU filter testing

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Starting IMU FIR Filter Test ==="
echo ""
echo "1. Starting IMU Filter Node..."
ros2 run micro_ros_setup imu_fir_filter_node --ros-args -p cutoff_freq:=1.0 -p filter_order:=4 &
FILTER_PID=$!

sleep 2

echo "2. Starting Test Comparison Node..."
ros2 run micro_ros_setup imu_test_node &
TEST_PID=$!

sleep 2

echo ""
echo "=== Monitoring IMU data ==="
echo "Raw:      /imu"
echo "Filtered: /imu/filtered"
echo ""
echo "Press Ctrl+C to stop..."
echo ""

# Keep the processes running
wait
