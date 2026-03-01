#!/bin/bash
# Stack rapide: RF2O + SLAM optimisé pour corridors

echo "🧹 Cleaning up..."
pkill -9 simu_lidar 2>/dev/null || true
pkill -9 simu_bridge 2>/dev/null || true
pkill -9 rf2o_laser_odometry_node 2>/dev/null || true
pkill -9 async_slam_toolbox_node 2>/dev/null || true
pkill -9 scan_restamper 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
sleep 2

echo ""
echo "==========================================="
echo "🚀 RF2O + SLAM RAPIDE (Corridors)"
echo "==========================================="
echo ""

# 1. Simulator
echo "1️⃣  Simulator..."
python3 simu_lidar.py &
SIMU_PID=$!
sleep 1

# 2. Bridge
echo "2️⃣  Bridge..."
python3 simu_bridge.py &
BRIDGE_PID=$!
sleep 1

# 3. TF2
echo "3️⃣  TF2..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
TF_PID=$!
sleep 0.5

# 4. Restamper
echo "4️⃣  Restamper..."
python3 scan_restamper_simu.py &
RESTAMPER_PID=$!
sleep 1

# 5. RF2O
echo "5️⃣  RF2O..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file config/rf2o_params.yaml &
RF2O_PID=$!
sleep 2

# 6. SLAM Toolbox (config rapide!)
echo "6️⃣  SLAM Toolbox (fast corridor mode)..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_corridor_fast.yaml &
SLAM_PID=$!
sleep 1

echo ""
echo "==========================================="
echo "✅ STACK RUNNING! 🏃"
echo "==========================================="
echo ""
echo "PIDs:"
echo "  Simulator:   $SIMU_PID"
echo "  Bridge:      $BRIDGE_PID"
echo "  TF2:         $TF_PID"
echo "  Restamper:   $RESTAMPER_PID"
echo "  RF2O:        $RF2O_PID"
echo "  SLAM:        $SLAM_PID"
echo ""
echo "Topics:"
echo "  /scan               (LaserScan)"
echo "  /odom_rf2o          (RF2O odometry)"
echo "  /odom               (SLAM corrected odometry)"
echo "  /map                (Occupancy grid)"
echo "  /odom_ground_truth  (Ground truth)"
echo ""
echo "Compare: python3 compare_rf2o_vs_slam.py"
echo "Control: python3 teleop_keyboard.py"
echo "Stop:    pkill -9 -f 'simu_|rf2o|slam'"
echo ""

wait
