#!/bin/bash
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
else
  echo "❌ Missing workspace setup: install/setup.bash not found in parent directories of $SCRIPT_DIR"
  exit 1
fi

ALREADY_CLEANED=0
TF_LASER_PID=""
RESTAMPER_PID=""
MOTOR_ODOM_PID=""
PYGAME_VIZ_PID=""
WALL_CENTER_PID=""
SLAM_PID=""

cleanup() {
  if [ "$ALREADY_CLEANED" -eq 1 ]; then
    return
  fi
  ALREADY_CLEANED=1

  local pids=("$TF_LASER_PID" "$RESTAMPER_PID" "$MOTOR_ODOM_PID" "$PYGAME_VIZ_PID" "$WALL_CENTER_PID" "$SLAM_PID")
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

echo "=============================================="
echo "ROBOT MAPPING AUTO (Corridor mode + Motor Odom)"
echo "=============================================="

pkill -f "slam_toolbox|scan_restamper|static_transform_publisher|motor_odom_simple|wall_centering_node|odom_pygame_visualizer|rf2o" 2>/dev/null || true
sleep 1

echo "[1/7] Static TF base_link -> laser_link"
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.15 0 0 0 base_link laser_link &
TF_LASER_PID=$!
sleep 1

echo "[2/7] Scan restamper (/scan_raw -> /scan, yaw offset)"
python3 scan_restamper_simple.py --ros-args -p scan_yaw_offset:=-1.57079632679 &
RESTAMPER_PID=$!
sleep 1

echo "[3/7] Waiting for /scan_raw"
python3 wait_for_topic.py /scan_raw 15 || { echo "❌ /scan_raw timeout"; exit 1; }

echo "[4/7] Waiting for /scan"
python3 wait_for_topic.py /scan 15 || { echo "❌ /scan timeout"; exit 1; }

echo "[5/7] Motor odometry (primary /odom for corridor)"
python3 motor_odom_simple.py --ros-args -p initial_yaw:=0.0 -p linear_scale:=-1.0 -p angular_scale:=-1.0 &
MOTOR_ODOM_PID=$!
sleep 1

#echo "[6/7] Wall centering autonomous drive"
#python3 wall_centering_node.py &
#WALL_CENTER_PID=$!
sleep 1

echo "[6/7] Python visualizer (pygame: /scan + /odom)"
python3 odom_pygame_visualizer.py --ros-args -p odom_topic:=/odom_motor -p heading_offset:=0.0 -p scan_yaw_offset:=0.0 &
PYGAME_VIZ_PID=$!
sleep 1

echo "[7/7] SLAM Toolbox mapping (odom prioritized, scan-matching light)"
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_toolbox_motor_corridor.yaml \
  -p use_sim_time:=false \
  -p scan_topic:=/scan \
  -p odom_topic:=/odom_motor \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map &
SLAM_PID=$!

echo ""
echo "✅ Stack started: corridor mapping + pygame visualizer"
echo "🖥️  Visualizer: pygame window (scan + odom path)"
echo "🛑 Stop with Ctrl+C"

wait
