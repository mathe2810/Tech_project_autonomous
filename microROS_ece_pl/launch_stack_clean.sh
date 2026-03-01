#!/bin/bash
# LAUNCH SCRIPT - Robust version without ROS 2 context conflicts

set -e  # Exit on any error

echo "=========================================="
echo "LANCEMENT SIMPLE ET ROBUSTE DU STACK"
echo "=========================================="

# Kill all previous instances
echo "🧹 Cleanup..."
pkill -9 rf2o 2>/dev/null || true
pkill -9 slam_toolbox 2>/dev/null || true
pkill -9 static_transform 2>/dev/null || true
pkill -9 scan_restamper 2>/dev/null || true
pkill -9 simu_lidar 2>/dev/null || true
pkill -9 simu_bridge 2>/dev/null || true
sleep 2

# Cleanup stale ROS 2 resources
rm -rf /tmp/ros* 2>/dev/null || true

echo "✅ Environment configured (using default ROS 2 middleware)"
echo ""

# === STAGE 1: Start simulator ===
echo "1️⃣ Starting LIDAR simulator..."
python3 simu_lidar.py > /tmp/simu_lidar.log 2>&1 &
SIMU_PID=$!
sleep 1.5
echo "   PID: $SIMU_PID"

# === STAGE 2: Start bridge ===
echo "2️⃣ Starting socket->ROS2 bridge..."
python3 simu_bridge.py > /tmp/simu_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 1
echo "   PID: $BRIDGE_PID"

# === STAGE 3: Start TF publisher ===
echo "3️⃣ Starting TF2 publisher..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link 2>&1 > /tmp/tf2.log &
TF_PID=$!
sleep 0.5
echo "   PID: $TF_PID"

# === STAGE 4: Start restamper ===
echo "4️⃣ Starting scan restamper..."
python3 scan_restamper_simu.py > /tmp/restamper.log 2>&1 &
RESTAMPER_PID=$!
sleep 1
echo "   PID: $RESTAMPER_PID"

# === STAGE 5: Wait for topics ===
echo ""
echo "⏳ Waiting for topics to stabilize..."
sleep 2

# === CHECK TOPICS ===
echo ""
echo "✅ Topics check:"
ros2 topic list | grep -E "(scan_raw|/scan|/odom)" && echo "   All topics present!" || echo "   ⚠️ Some topics missing"

# === STAGE 6: Start RF2O in separate context ===
echo ""
echo "5️⃣ Starting RF2O laser odometry..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -r scan:=/scan \
  -r odom:=/odom \
  > /tmp/rf2o.log 2>&1 &
RF2O_PID=$!
sleep 1
echo "   PID: $RF2O_PID"

# === SUMMARY ===
echo ""
echo "=========================================="
echo "STACK RUNNING SUCCESSFULLY! 🚀"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Simulator:    $SIMU_PID"
echo "  Bridge:       $BRIDGE_PID"
echo "  TF2:          $TF_PID"
echo "  Restamper:    $RESTAMPER_PID"
echo "  RF2O:         $RF2O_PID"
echo ""
echo "Log files:"
echo "  tail -f /tmp/simu_lidar.log"
echo "  tail -f /tmp/simu_bridge.log"
echo "  tail -f /tmp/restamper.log"
echo "  tail -f /tmp/rf2o.log"
echo ""
echo "Commands:"
echo "  ros2 topic echo /scan_raw"
echo "  ros2 topic echo /scan"
echo "  ros2 topic echo /odom"
echo ""
echo "Press Ctrl+C to stop all"
echo "=========================================="

# Setup signal handler
cleanup() {
    echo ""
    echo "🛑 Stopping all processes..."
    kill $SIMU_PID $BRIDGE_PID $TF_PID $RESTAMPER_PID $RF2O_PID 2>/dev/null || true
    sleep 1
    pkill -9 simu_lidar 2>/dev/null || true
    pkill -9 simu_bridge 2>/dev/null || true
    pkill -9 static_transform 2>/dev/null || true
    pkill -9 scan_restamper 2>/dev/null || true
    pkill -9 rf2o 2>/dev/null || true
    echo "✅ All stopped"
    exit 0
}

trap cleanup INT TERM

# Wait for processes
wait $SIMU_PID $BRIDGE_PID $TF_PID $RESTAMPER_PID $RF2O_PID 2>/dev/null || true
