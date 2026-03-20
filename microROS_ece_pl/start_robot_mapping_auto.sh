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
SLAM_PID=""

cleanup() {
    if [ "$ALREADY_CLEANED" -eq 1 ]; then return; fi
    ALREADY_CLEANED=1
    local pids=("$TF_LASER_PID" "$RESTAMPER_PID" "$MOTOR_ODOM_PID" "$SLAM_PID")
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
echo "  ROBOT MAPPING - Pure Scan Matching"
echo "=============================================="

pkill -f "slam_toolbox|scan_restamper|static_transform_publisher|motor_odom_simple" 2>/dev/null || true
sleep 1

# [1] TF statique base_link -> laser_link
# Ajuste x/y/z/roll/pitch/YAW selon la position physique réelle du lidar sur le robot
echo "[1/6] Static TF: base_link -> laser_link"
ros2 run tf2_ros static_transform_publisher \
    0.10 0.0 0.15 \
    0 0 0 \
    base_link laser_link &
TF_LASER_PID=$!
sleep 1

# [2] Scan restamper
echo "[2/6] Scan restamper (/scan_raw -> /scan)"
python3 scan_restamper_simple.py --ros-args &
RESTAMPER_PID=$!
sleep 1

# [3] Attente topics
echo "[3/6] Waiting for /scan_raw..."
python3 wait_for_topic.py /scan_raw 15 || { echo "❌ /scan_raw timeout"; exit 1; }
echo "      Waiting for /scan..."
python3 wait_for_topic.py /scan 15    || { echo "❌ /scan timeout"; exit 1; }

# [4] Odom init-only : publie TF odom->base_link identité + /odom avec covariance énorme
#     SLAM Toolbox recevra un prior "je ne sais rien" et fera 100% scan matching
echo "[4/6] Odom init (static identity, covariance=1e6)"
python3 motor_odom_simple.py --ros-args &
MOTOR_ODOM_PID=$!
sleep 1

# [5] SLAM Toolbox
echo "[5/6] SLAM Toolbox (pure scan matching)"
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    --params-file config/slam_toolbox_minimal.yaml \
    -p use_sim_time:=false &
SLAM_PID=$!

sleep 4

echo "[6/6] Lancement de wall_centering_node.py"
python3 wall_centering_node.py --ros-args &


echo ""
echo "✅ Stack started"
echo "   TF chain: map -> odom -> base_link -> laser_link"
echo "   Odom = identité fixe, SLAM = scan matching pur"
echo "🛑 Stop: Ctrl+C"
wait