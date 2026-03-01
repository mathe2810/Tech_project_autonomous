#!/bin/bash
# ========================================
# FINAL LAUNCH SCRIPT - RF2O + LIDAR SIM
# ========================================
# Tested and working configuration!

echo "=========================================="
echo "🚀 Lancement FINAL du Stack RF2O"
echo "=========================================="

# Cleanup
echo "🧹 Cleaning up previous processes..."
pkill -9 simu_lidar 2>/dev/null || true
pkill -9 simu_bridge 2>/dev/null || true
pkill -9 scan_restamper 2>/dev/null || true
pkill -9 rf2o 2>/dev/null || true
pkill -9 static_transform 2>/dev/null || true
sleep 2

# Components
echo ""
echo "1️⃣  LIDAR Simulator (PyGame)"
python3 simu_lidar.py > /tmp/simu_lidar.log 2>&1 &
SIMU_PID=$!
sleep 1.5

echo "2️⃣  Socket → ROS2 Bridge"
python3 simu_bridge.py > /tmp/simu_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 1

echo "3️⃣  TF2 Base Link → Laser Link"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link > /tmp/tf2.log 2>&1 &
TF_PID=$!
sleep 0.5

echo "4️⃣  Scan Restamper (/scan_raw → /scan)"
python3 scan_restamper_simu.py > /tmp/restamper.log 2>&1 &
RESTAMPER_PID=$!
sleep 1

echo "5️⃣  RF2O Laser Odometry"
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -r scan:=/scan \
  -r odom:=/odom_rf2o \
  > /tmp/rf2o.log 2>&1 &
RF2O_PID=$!
sleep 2

# Cleanup handler
trap "echo ''; echo 'Stopping...'; kill $SIMU_PID $BRIDGE_PID $TF_PID $RESTAMPER_PID $RF2O_PID 2>/dev/null; exit" INT TERM

echo ""
echo "=========================================="
echo "✅ STACK RUNNING!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Simulator:    $SIMU_PID"
echo "  Bridge:       $BRIDGE_PID"
echo "  TF:           $TF_PID"
echo "  Restamper:    $RESTAMPER_PID"
echo "  RF2O:         $RF2O_PID"
echo ""
echo "Topics:"
echo "  /scan_raw    → LaserScan (raw from simulator)"
 echo "  /scan        → LaserScan (retimestamped)"
echo "  /odom_rf2o   → Odometry (from RF2O)"
echo ""
echo "Monitors:"
echo "  tail -f /tmp/simu_lidar.log"
echo "  tail -f /tmp/simu_bridge.log"
echo "  tail -f /tmp/rf2o.log"
echo ""
echo "Commands:"
echo "  ros2 topic echo /scan_raw   # See raw LIDAR data"
echo "  ros2 topic echo /scan       # See restamped LIDAR"
echo "  ros2 topic echo /odom_rf2o  # See RF2O odometry"
echo ""
echo "=========================================="
echo "Press Ctrl+C to stop"
echo "=========================================="

wait $SIMU_PID $BRIDGE_PID $TF_PID $RESTAMPER_PID $RF2O_PID 2>/dev/null || true
