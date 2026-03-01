#!/bin/bash
# Complete stack: Simulator -> RF2O + SLAM with Closed-Loop
# Based on proven start_slam_lidar_simu.sh pattern

echo "🧹 Cleaning up old processes..."
pkill -9 -f "simu_lidar" 2>/dev/null || true
pkill -9 -f "simu_bridge" 2>/dev/null || true
pkill -9 -f "rf2o" 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "scan_restamper" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
sleep 2

echo ""
echo "==========================================="
echo "🚀 LAUNCHING RF2O + SLAM STACK"
echo "==========================================="
echo ""

# 1. Simulator
echo "1️⃣  Starting LIDAR simulator..."
python3 simu_lidar.py &
SIMU_PID=$!
sleep 1

# 2. Bridge (publishes /scan_raw + /odom_ground_truth)
echo "2️⃣  Starting socket->ROS2 bridge..."
python3 simu_bridge.py &
BRIDGE_PID=$!
sleep 1

# 3. TF2 Publisher
echo "3️⃣  Starting TF2 publisher (base_link ↔ laser_link)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
TF_PID=$!
sleep 0.5

# 4. Scan Restamper
echo "4️⃣  Starting scan restamper (/scan_raw → /scan)..."
python3 scan_restamper_simu.py &
RESTAMPER_PID=$!
sleep 1

# 5. RF2O Odometry
echo "5️⃣  Starting RF2O laser odometry..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file config/rf2o_params.yaml &
RF2O_PID=$!
sleep 2

# 6. SLAM Toolbox (with closed-loop correction)
echo "6️⃣  Starting SLAM Toolbox with closed-loop..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_toolbox_closed_loop.yaml &
SLAM_PID=$!
sleep 1

echo ""
echo "==========================================="
echo "✅ STACK RUNNING WITH CLOSED-LOOP SLAM! 🎉"
echo "==========================================="
echo ""
echo "Process IDs:"
echo "  Simulator:     $SIMU_PID"
echo "  Bridge:        $BRIDGE_PID"
echo "  TF2:           $TF_PID"
echo "  Restamper:     $RESTAMPER_PID"  
echo "  RF2O:          $RF2O_PID"
echo "  SLAM Toolbox:  $SLAM_PID"
echo ""
echo "Topics available:"
echo "  /scan              (LaserScan from simulator)"
echo "  /odom_ground_truth (Ground truth from simulator)"
echo "  /odom_rf2o         (Raw odometry from RF2O)"
echo "  /odom              (Corrected odometry from SLAM)"
echo "  /map               (Occupancy grid map from SLAM)"
echo "  /pose              (Robot pose from SLAM)"
echo ""
echo "To visualize:"
echo "  ros2 run rviz2 rviz2 -c rviz_simu_nav2.rviz"
echo ""
echo "To compare odometries in another terminal:"
echo "  python3 compare_rf2o_vs_slam.py"
echo ""
echo "To stop: Press Ctrl+C or run pkill -9 -f 'simu_|rf2o|slam_'"
echo ""

# Wait for all processes
wait
