#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
else
  echo "❌ Missing workspace setup: $WORKSPACE_ROOT/install/setup.bash"
  exit 1
fi

ALREADY_CLEANED=0
TF_LASER_PID=""
RESTAMPER_PID=""
RF2O_PID=""
SLAM_PID=""

cleanup() {
  if [ "$ALREADY_CLEANED" -eq 1 ]; then
    return
  fi
  ALREADY_CLEANED=1
  local pids=("$TF_LASER_PID" "$RESTAMPER_PID" "$RF2O_PID" "$SLAM_PID")
  local has_running=0

  for pid in "${pids[@]}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      has_running=1
      break
    fi
  done

  if [ "$has_running" -eq 1 ]; then
    echo ""
    echo "Stopping SLAM stack (no RViz)..."
  fi

  for pid in "${pids[@]}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  for pid in "${pids[@]}"; do
    if [ -n "$pid" ]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}

trap cleanup INT TERM EXIT

#!/bin/bash
set -e

# ... (garder le début habituel : sourcing, paths, cleanup) ...

echo "=============================================="
echo "SLAM OPTIMIZED (No RViz - CPU Efficient)"
echo "=============================================="

pkill -f "slam_toolbox|rf2o|scan_restamper|static_transform_publisher" 2>/dev/null || true
sleep 1

echo "[1/5] Static TF base_link -> laser_link"
# Version simplifiée compatible Humble
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.15 0 0 0 base_link laser_link &
TF_LASER_PID=$!
sleep 1

echo "[2/5] Scan restamper"
python3 scan_restamper_simple.py &
RESTAMPER_PID=$!
sleep 2

echo "[3/5] Waiting for /scan"
# On s'assure que le laser tourne
python3 wait_for_topic.py /scan 15 || { echo "❌ /scan timeout"; exit 1; }

echo "[4/5] RF2O (Synchronized with LiDAR)"
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file config/rf2o_params.yaml \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p freq:=15.0 \
  -p publish_tf:=true &
RF2O_PID=$!
sleep 2

echo "[5/5] SLAM Toolbox (Async Mode)"
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_toolbox_rf2o.yaml \
  -p use_sim_time:=false &
SLAM_PID=$!

wait
