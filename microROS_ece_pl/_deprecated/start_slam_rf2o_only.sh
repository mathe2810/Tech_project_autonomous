#!/bin/bash
# Simplified SLAM Stack - RF2O Only
# Direct LIDAR odometry + SLAM Toolbox
# No motor odometry (simpler, cleaner, faster)

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

STACK_DIR="/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl"
CONFIG_DIR="$STACK_DIR/config"

# Define cleanup FIRST
cleanup() {
    echo ""
    echo "⏹️  Shutting down..."
    pkill -f "scan_restamper_simple" 2>/dev/null || true
    pkill -f "async_slam_toolbox_node" 2>/dev/null || true
    pkill -f "rf2o_laser_odometry" 2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
    sleep 1
    echo "✓ All nodes stopped"
    exit 0
}

# Set trap FIRST
trap cleanup SIGINT SIGTERM EXIT

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     SLAM Mapping Stack - RF2O Laser Odometry Only          ║"
echo "║        (Motor Odometry Removed - Simpler)                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Cleanup old processes
echo "🧹 Cleaning up old processes..."
pkill -f "scan_restamper_simple" 2>/dev/null || true
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
pkill -f "rf2o_laser_odometry" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
sleep 1

# Step 1: Static TF
echo ""
echo "1️⃣  Publishing static transform (base_link → laser_link)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
TF_PID=$!
sleep 8  # LONGER wait - TF buffer needs time to initialize

# Step 2: Scan Restamper (timestamp sync)
echo "2️⃣  Starting Scan Restamper (LIDAR timestamp sync)..."
python3 "$STACK_DIR/scan_restamper_simple.py" &
SCAN_PID=$!

# Wait for /scan topic to be available
echo "   ⏳ Waiting for /scan topic..."
python3 "$STACK_DIR/wait_for_topic.py" "/scan" 15 || {
    echo "   ❌ /scan topic not available, exiting"
    exit 1
}
sleep 1

# Step 3: RF2O Laser Odometry (publishes directly to /odom)
echo "3️⃣  Starting RF2O Laser Odometry (LIDAR-based, 50Hz)..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  --params-file "$CONFIG_DIR/rf2o_params.yaml" \
  -r /odom_rf2o:=/odom \
  &
RF2O_PID=$!

# Wait for /odom_rf2o topic (RF2O's actual output - not /odom!)
echo "   ⏳ Waiting for /odom topic..."
python3 "$STACK_DIR/wait_for_topic.py" "/odom" 30 || {
    echo "   ⚠️  /odom topic detection timeout, but RF2O is processing scans (continuing anyway)"
}
sleep 2

# Step 4: SLAM Toolbox
echo "4️⃣  Starting SLAM Toolbox (Async mode)..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p slam_params_file:="$CONFIG_DIR/slam_toolbox_rf2o.yaml" \
  &
SLAM_PID=$!
sleep 2

echo ""
echo "✅ STACK STARTED - RF2O Laser Odometry (Simplified)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Running Node PIDs:"
echo "  • Static TF: $TF_PID"
echo "  • Scan Restamper: $SCAN_PID"
echo "  • RF2O Laser Odom: $RF2O_PID (publishes to /odom)"
echo "  • SLAM Toolbox: $SLAM_PID (uses /odom)"
echo ""
echo "Topics:"
ros2 topic list 2>/dev/null | grep -E "(scan|odom|map)" || echo "  (waiting for topics...)"
echo ""
echo "📍 Monitor:"
echo "  ros2 topic echo /odom          # RF2O laser odometry"
echo "  ros2 topic echo /scan          # LIDAR scan"
echo "  ros2 topic echo /map           # Generated map"
echo ""
echo "Architecture:"
echo "  LIDAR (/scan_raw)"
echo "    ↓"
echo "  Scan Restamper (/scan)"
echo "    ↓"
echo "  RF2O Laser Odometry (/odom) ← Direct LIDAR-based odometry"
echo "    ↓"
echo "  SLAM Toolbox (/map) ← Long-term correction"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Wait for all background processes
wait
    kill $SCAN_PID 2>/dev/null || true
    kill $RF2O_PID 2>/dev/null || true
    kill $SLAM_PID 2>/dev/null || true
    sleep 1
    echo "✓ All nodes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for all
wait
