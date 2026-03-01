#!/bin/bash

# --- Nettoyage radical ---
echo "Cleaning up..."
pkill -9 rf2o 2>/dev/null || true
pkill -9 slam_toolbox 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
pkill -9 scan_restamper 2>/dev/null || true
pkill -9 simu_lidar 2>/dev/null || true
pkill -9 simu_bridge 2>/dev/null || true
sleep 2

# --- Lancement ---

echo "Launching SLAM with LIDAR simulation..."
python3 simu_lidar.py &
SIMU_LIDAR_PID=$!
sleep 1

python3 simu_bridge.py &
SIMU_BRIDGE_PID=$!
sleep 1

echo "1. TFs - Static transform base_link -> laser_link"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
TF_PID=$!
sleep 0.5

echo "2. Restamper - Retimestamp /scan_raw -> /scan"
python3 scan_restamper_simu.py &
RESTAMPER_PID=$!
sleep 1

echo "3. RF2O - Odometry from scan"
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file config/rf2o_params.yaml &
RF2O_PID=$!

echo ""
echo "========================================="
echo "STACK RUNNING - PIDs:"
echo "  Simulator:    $SIMU_LIDAR_PID"
echo "  Bridge:       $SIMU_BRIDGE_PID"
echo "  TF Publisher: $TF_PID"
echo "  Restamper:    $RESTAMPER_PID"
echo "  RF2O:         $RF2O_PID"
echo "========================================="
echo ""
echo "Press Ctrl+C to stop"

# Attendre les processus
wait