#!/bin/bash
# Robot autonome + SLAM + Nav2 officiel

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

if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
  echo "❌ Package nav2_bringup introuvable"
  echo "Installe Nav2: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
  exit 1
fi

echo "🧹 Cleaning up..."
pkill -9 -f "simu_lidar_auto.py|simu_bridge.py|odom_to_tf.py|slam_.*\.py|async_slam_toolbox_node" 2>/dev/null || true
pkill -9 -f "controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager_navigation" 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
sleep 2

echo ""
echo "==========================================="
echo "🤖 ROBOT AUTONOME + SLAM + NAV2"
echo "==========================================="
echo ""

echo "1️⃣  Simulator (autonome)..."
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
sleep 3

echo "7️⃣  Nav2 bringup (navigation_launch.py)..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=False \
  params_file:=$SCRIPT_DIR/config/nav2_params.yaml &
sleep 3

if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
  echo "✅ Nav2 action server prêt: /navigate_to_pose"
else
  echo "⚠️ Nav2 pas prêt: /navigate_to_pose absent (voir logs planner_server)"
fi

echo "8️⃣  Map saver + odom_slam debug..."
python3 slam_map_saver.py &
python3 slam_odom_from_tf.py &
sleep 1

echo ""
echo "==========================================="
echo "✅ READY (SLAM + NAV2)"
echo "==========================================="
echo ""
echo "🎯 TEST NAV2 (dans RViz):"
echo "   1) ros2 run rviz2 rviz2 -d rviz_simu_nav2.rviz"
echo "   2) Outil '2D Goal Pose' pour envoyer un objectif"
echo "   3) Goal auto début de piste: python3 nav2_goal_start.py"
echo ""
echo "📊 Monitoring:"
echo "   ros2 topic echo /plan --once"
echo "   ros2 topic hz /cmd_vel"
echo ""
echo "🛑 Stop:"
echo "   pkill -9 -f 'simu_|slam_|nav2|planner_server|controller_server|bt_navigator'"
echo ""

wait
