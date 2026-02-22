#!/bin/bash
# SLAM Toolbox + RF2O (LIDAR odometry)
# Strategy: Use RF2O as single odometry source for robust pure-rotation handling

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================================"
echo "SLAM Toolbox + RF2O LIDAR Odometry"
echo "================================================"

# Kill any existing processes
pkill -f "slam_toolbox|rf2o|scan_rest|teleop|rviz2" 2>/dev/null || true
sleep 2

# 1. Static Transform (base_link -> laser_link)
echo "[1/5] Starting Static Transform Publisher..."
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.15 0 0 0 base_link laser_link &
TF_PID=$!
sleep 1

# 2. Scan Restamper (synchronize LIDAR timestamps)
echo "[2/5] Starting Scan Restamper..."
python3 scan_restamper_simple.py &
RESTAMPER_PID=$!
sleep 2

# 3. RF2O Laser Odometry (LIDAR-based odometry)
echo "[3/5] Starting RF2O Laser Odometry..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file config/rf2o_params.yaml \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p freq:=20.0 \
  -p publish_tf:=true &
RF2O_PID=$!
sleep 3

# 4. SLAM Toolbox
echo "[4/5] Starting SLAM Toolbox (async mode)..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_toolbox_rf2o.yaml &
SLAM_PID=$!
sleep 3

# 5. RViz
echo "[5/5] Starting RViz2 with project config..."
rviz2 -d rviz_config.rviz &
RVIZ_PID=$!
sleep 2

echo ""
echo "================================================"
echo "Stack running!"
echo "================================================"
echo "Topics:"
echo "  /scan          (LIDAR scans)"
echo "  /odom          (RF2O odometry - used by SLAM)"
echo "  /map           (Generated map)"
echo ""
echo "Test with:"
echo "  ros2 topic hz /scan /odom /map"
echo "  RViz started automatically with rviz_config.rviz"
echo ""
echo "Send movement commands:"
echo "  python3 test_cmd_vel.py"
echo ""
echo "Quit: pkill -f 'slam_toolbox|rf2o|scan_rest|rviz2'"
echo "================================================"

# Keep script alive
wait
