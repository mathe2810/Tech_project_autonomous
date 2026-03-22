#!/bin/bash
# ========================================
# RF2O + LIDAR SIMULATOR LAUNCH
# With FastRTPS SHM disabled (UDP only)
# ========================================

# Configuration pour désactiver SHM
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastrtps.xml
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=========================================="
echo "🚀 Lancement du Stack RF2O + LIDAR"
echo "=========================================="
echo "FastRTPS config: UDP only (SHM disabled)"
echo ""

# Cleanup
echo "🧹 Cleanup..."
pkill -9 -f "simu_lidar.py" 2>/dev/null || true
pkill -9 -f "simu_bridge.py" 2>/dev/null || true
pkill -9 -f "scan_restamper" 2>/dev/null || true
pkill -9 -f "rf2o_laser_odometry" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
rm -rf /tmp/ros* /tmp/cyclone* /dev/shm/sem* /dev/shm/rtps* 2>/dev/null
sleep 2

echo "✅ Ready to start"
echo ""

# 1. LIDAR Simulator
echo "1️⃣  LIDAR Simulator"
python3 simu_lidar.py > /tmp/simu_lidar.log 2>&1 &
SIMU_PID=$!
sleep 2

# 2. Bridge
echo "2️⃣  Socket → ROS2 Bridge"
python3 simu_bridge.py > /tmp/simu_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 2

# 3. TF
echo "3️⃣  TF2 Broadcaster"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link > /tmp/tf2.log 2>&1 &
TF_PID=$!
sleep 1

# 4. Restamper
echo "4️⃣  Scan Restamper"
python3 scan_restamper_simu.py > /tmp/restamper.log 2>&1 &
RESTAMPER_PID=$!
sleep 2

# WAIT FOR SCANS TO START FLOWING
echo "⏳ Waiting for scan data to flow..."
for i in {1..10}; do
  if timeout 1 ros2 topic echo /scan > /dev/null 2>&1; then
    echo "✅ Scans detected!"
    break
  fi
  echo "  Attempt $i..."
  sleep 1
done

# 5. RF2O (APRÈS que les scans circulent)
echo "5️⃣  RF2O Odometry"
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -r scan:=/scan \
  -r odom:=/odom_rf2o \
  > /tmp/rf2o.log 2>&1 &
RF2O_PID=$!
sleep 1

echo ""
echo "=========================================="
echo "✅ STACK RUNNING"
echo "=========================================="
echo ""
echo "Topics:"
echo "  /scan_raw    - Raw LIDAR"
echo "  /scan        - Restamped LIDAR"
echo "  /odom_rf2o   - RF2O Odometry"
echo ""
echo "Logs:"
echo "  tail -f /tmp/simu_lidar.log"
echo "  tail -f /tmp/rf2o.log"
echo ""
echo "Test:"
echo "  ros2 topic list"
echo "  ros2 topic echo /odom_rf2o"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Trap for cleanup
trap "echo ''; echo 'Stopping...'; kill $SIMU_PID $BRIDGE_PID $TF_PID $RESTAMPER_PID $RF2O_PID 2>/dev/null || true; exit" INT TERM

# Wait
wait $SIMU_PID $BRIDGE_PID $TF_PID $RESTAMPER_PID $RF2O_PID 2>/dev/null || true
