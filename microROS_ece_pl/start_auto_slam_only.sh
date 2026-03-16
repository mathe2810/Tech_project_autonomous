#!/bin/bash
# Test robot autonome en mode SLAM-only (sans RF2O)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

WORKSPACE_SETUP=""
SEARCH_DIR="$SCRIPT_DIR"
while [ "$SEARCH_DIR" != "/" ]; do
  CANDIDATE_SETUP="$SEARCH_DIR/install/setup.bash"
  if [ -f "$CANDIDATE_SETUP" ]; then
    WORKSPACE_SETUP="$CANDIDATE_SETUP"
    break
  fi
  SEARCH_DIR="$(dirname "$SEARCH_DIR")"
done

if [ -n "$WORKSPACE_SETUP" ]; then
  source "$WORKSPACE_SETUP"
fi

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "❌ Package slam_toolbox introuvable dans l'environnement courant"
  echo "Vérifie le source de ROS2 et de ton workspace"
  exit 1
fi

echo "🧹 Cleaning up..."
pkill -9 simu_lidar 2>/dev/null || true
pkill -9 simu_bridge 2>/dev/null || true
pkill -9 rf2o_laser_odometry_node 2>/dev/null || true
pkill -9 async_slam_toolbox_node 2>/dev/null || true
pkill -9 scan_restamper 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
pkill -9 slam_map_saver 2>/dev/null || true
pkill -9 slam_odom_from_tf 2>/dev/null || true
pkill -9 slam_initial_pose_setter 2>/dev/null || true
pkill -9 odom_to_tf 2>/dev/null || true
sleep 2

echo ""
echo "==========================================="
echo "🤖 ROBOT AUTONOME + SLAM (SANS RF2O)"
echo "==========================================="
echo ""
echo "Appuyez sur 'A' dans la fenêtre pygame"
echo "pour activer le mode AUTONOME!"
echo ""

echo "1️⃣  Simulator (avec mode autonome)..."
python3 simu_lidar_auto.py &
sleep 1

echo "2️⃣  Bridge (scan + odom ground truth)..."
python3 simu_bridge.py &
sleep 1

echo "3️⃣  TF2..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
sleep 0.5

echo "4️⃣  Odom -> TF (bootstrap TF chain)..."
python3 odom_to_tf.py &
sleep 1

echo "5️⃣  SLAM Initial Pose Setter (from ground truth)..."
python3 slam_initial_pose_setter.py &
sleep 1

echo "6️⃣  SLAM Toolbox (odom = /odom_ground_truth)..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_corridor_slam_only.yaml &
sleep 2

echo "7️⃣  Map Saver (PNG debug)..."
python3 slam_map_saver.py &
sleep 1

echo "8️⃣  SLAM Odom Publisher (/odom_slam)..."
python3 slam_odom_from_tf.py &
sleep 1

echo ""
echo "==========================================="
echo "✅ READY (SLAM-only)"
echo "==========================================="
echo ""
echo "🎮 CONTRÔLES:"
echo "   A          = Toggle mode AUTONOME"
echo "   Flèches    = Contrôle manuel"
echo ""
echo "📊 Observer: python3 compare_rf2o_vs_slam.py"
echo "🛑 Stop:     pkill -9 -f 'simu_|rf2o|slam|restamper|transform'"
echo ""

wait
