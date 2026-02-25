#!/bin/bash
# Minimal lidar-only SLAM baseline (no RF2O)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ALREADY_CLEANED=0
cleanup() {
  if [ "$ALREADY_CLEANED" -eq 1 ]; then
    return
  fi
  ALREADY_CLEANED=1
  echo ""
  echo "Stopping lidar-only stack..."
  kill "$TF_LASER_PID" "$TF_ODOM_PID" "$RESTAMPER_PID" "$SLAM_PID" "$RVIZ_PID" 2>/dev/null || true
  wait "$TF_LASER_PID" "$TF_ODOM_PID" "$RESTAMPER_PID" "$SLAM_PID" "$RVIZ_PID" 2>/dev/null || true
}

trap cleanup INT TERM EXIT

echo "=============================================="
echo "SLAM Toolbox - Lidar only (no RF2O)"
echo "=============================================="

pkill -f "slam_toolbox|scan_restamper|rviz2|static_transform_publisher" 2>/dev/null || true
sleep 1

echo "[1/6] Starting static TF base_link -> laser_link"
ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0.0 --z 0.15 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser_link &
TF_LASER_PID=$!
sleep 1

echo "[2/6] Starting static TF odom -> base_link (lidar-only baseline)"
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_link &
TF_ODOM_PID=$!
sleep 1

echo "[3/6] Starting scan restamper (/scan_raw -> /scan)"
python3 scan_restamper_simple.py &
RESTAMPER_PID=$!
sleep 2

echo "[4/6] Waiting for /scan topic"
python3 wait_for_topic.py /scan 15 || {
  echo "❌ /scan not available. Stopping stack."
  kill "$TF_LASER_PID" "$TF_ODOM_PID" "$RESTAMPER_PID" 2>/dev/null || true
  exit 1
}

echo "[5/6] Starting SLAM Toolbox (async)"
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_toolbox_lidar_only.yaml &
SLAM_PID=$!
sleep 3

echo "[6/6] Starting RViz"
rviz2 -d rviz_config.rviz &
RVIZ_PID=$!

echo ""
echo "=============================================="
echo "Running lidar-only baseline"
echo "=============================================="
echo "Topics to check:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /map"
echo "  ros2 topic hz /tf"
echo "  ros2 topic echo /tf --once"
echo ""
echo "Move robot slowly with:"
echo "  python3 test_cmd_vel.py"
echo ""
echo "Stop all with:"
echo "  pkill -f 'slam_toolbox|scan_restamper|rviz2|static_transform_publisher'"
echo "=============================================="

wait
