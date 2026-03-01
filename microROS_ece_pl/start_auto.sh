#!/bin/bash
# Test du robot autonome avec SLAM

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
echo "🤖 ROBOT AUTONOME + SLAM"
echo "==========================================="
echo ""
echo "Appuyez sur 'A' dans la fenêtre pygame"
echo "pour activer le mode AUTONOME!"
echo ""

echo "1️⃣  Simulator (avec mode autonome)..."
python3 simu_lidar_auto.py &
sleep 1

echo "2️⃣  Bridge..."
python3 simu_bridge.py &
sleep 1

echo "3️⃣  TF2..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
sleep 0.5

echo "4️⃣  Restamper..."
python3 scan_restamper_simu.py &
sleep 1

echo "5️⃣  RF2O..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file config/rf2o_params.yaml &
sleep 2

echo "6️⃣  SLAM with Loop Closure..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_corridor_loop.yaml &
sleep 1

echo ""
echo "==========================================="
echo "✅ READY!"
echo "==========================================="
echo ""
echo "🎮 CONTRÔLES:"
echo "   A          = Toggle mode AUTONOME"
echo "   Flèches    = Contrôle manuel"
echo ""
echo "📊 Observer: python3 compare_rf2o_vs_slam.py"
echo "🛑 Stop:     pkill -9 -f 'simu_|rf2o|slam'"
echo ""

wait
