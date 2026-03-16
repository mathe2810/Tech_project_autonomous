#!/bin/bash
# Robot autonome + SLAM + retour SIMPLE (A* + PID), sans Nav2

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "❌ Package slam_toolbox introuvable"
  exit 1
fi

echo "🧹 Cleaning up..."
pkill -9 -f "simu_lidar_auto.py|simu_bridge.py|odom_to_tf.py|slam_.*\.py|async_slam_toolbox_node" 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
sleep 2

echo ""
echo "==========================================="
echo "🤖 ROBOT AUTONOME + SLAM + RETOUR SIMPLE"
echo "==========================================="
echo ""

echo "1️⃣  Simulator (autonome + mode simple intégré)..."
python3 simu_lidar_auto.py &
sleep 1

echo "2️⃣  Bridge (/scan + /odom_ground_truth)..."
python3 simu_bridge.py &
sleep 1

echo "3️⃣  TF statique base_link -> laser_link..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser_link &
sleep 0.5

echo "4️⃣  TF dynamique odom -> base_link..."
python3 odom_to_tf.py &
sleep 1

echo "5️⃣  SLAM initial pose..."
python3 slam_initial_pose_setter.py &
sleep 1

echo "6️⃣  SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_corridor_slam_only.yaml &
sleep 2

echo "7️⃣  Map saver + odom_slam debug..."
python3 slam_map_saver.py &
python3 slam_odom_from_tf.py &
sleep 1

echo ""
echo "==========================================="
echo "✅ READY (SLAM + SIMPLE RETURN)"
echo "==========================================="
echo ""
echo "ℹ️ Fonctionnement:"
echo "   - Mapping autonome"
echo "   - Freeze SLAM"
echo "   - Retour A* + PID (sans Nav2)"
echo ""
echo "📊 Monitoring:"
echo "   ros2 topic hz /scan"
echo "   ros2 topic echo /odom_ground_truth --once"
echo ""
echo "🛑 Stop:"
echo "   pkill -9 -f 'simu_|slam_|odom_to_tf|static_transform_publisher'"
echo ""

wait
