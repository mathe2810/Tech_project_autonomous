#!/bin/bash
# Simple SLAM Stack Launcher
# No lifecycle, no package discovery issues
# Direct node launching with proper setup

set -e

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

STACK_DIR="/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl"
CONFIG_DIR="$STACK_DIR/config"

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║          SLAM Mapping Stack - Simple Mode                  ║"
echo "║  LIDAR + Motor Odometry + SLAM Toolbox                     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Cleanup old processes
echo "🧹 Cleaning up old processes..."
pkill -f "motor_odom_simple" 2>/dev/null || true
pkill -f "scan_restamper_simple" 2>/dev/null || true
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
sleep 1

# Step 1: Static TF
echo ""
echo "1️⃣  Publishing static transform (base_link → laser_link)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
TF_PID=$!
sleep 1

# Step 2: Motor Odometry (publishes to /odom_motor)
echo "2️⃣  Starting Motor Odometry Node..."
python3 "$STACK_DIR/motor_odom_simple.py" &
MOTOR_PID=$!
sleep 1

# Step 3: Scan Restamper
echo "3️⃣  Starting Scan Restamper (LIDAR timestamp sync)..."
python3 "$STACK_DIR/scan_restamper_simple.py" &
SCAN_PID=$!
sleep 1

# Step 4: RF2O Laser Odometry (publishes to /odom_rf2o)
echo "4️⃣  Starting RF2O Laser Odometry..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom_rf2o \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p freq:=50.0 \
  &
RF2O_PID=$!
sleep 1

# Step 5: Odometry Fusion (merges rf2o + motor, publishes /odom)
echo "5️⃣  Starting Odometry Fusion (RF2O + Motor)..."
python3 "$STACK_DIR/odom_fusion.py" &
FUSION_PID=$!
sleep 1

# Step 6: SLAM Toolbox
echo "6️⃣  Starting SLAM Toolbox (Async mode)..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p slam_params_file:="$CONFIG_DIR/slam_simple.yaml" \
  &
SLAM_PID=$!
sleep 2

echo ""
echo "✅ STACK STARTED - RF2O + Motor Odometry Fusion"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Running Node PIDs:"
echo "  • Static TF: $TF_PID"
echo "  • Motor Odometry: $MOTOR_PID (publishes to /odom_motor)"
echo "  • Scan Restamper: $SCAN_PID"
echo "  • RF2O Laser Odom: $RF2O_PID (publishes to /odom_rf2o)"
echo "  • Odometry Fusion: $FUSION_PID (merges → /odom)"
echo "  • SLAM Toolbox: $SLAM_PID (uses /odom)"
echo ""
echo "Topics:"
ros2 topic list 2>/dev/null | grep -E "(scan|odom|map)" || echo "  (waiting for topics...)"
echo ""
echo "📍 Check status:"
echo "  ros2 topic echo /odom        # FUSED odometry (RF2O 80% + Motor 20%)"
echo "  ros2 topic echo /odom_rf2o   # RF2O LIDAR-based odometry"
echo "  ros2 topic echo /odom_motor  # Motor dead-reckoning"
echo "  ros2 topic echo /scan        # LIDAR scan"
echo "  ros2 topic echo /map         # Generated map"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Cleanup handler
cleanup() {
    echo ""
    echo "⏹️  Shutting down..."
    kill $TF_PID 2>/dev/null || true
    kill $MOTOR_PID 2>/dev/null || true
    kill $SCAN_PID 2>/dev/null || true
    kill $RF2O_PID 2>/dev/null || true
    kill $FUSION_PID 2>/dev/null || true
    kill $SLAM_PID 2>/dev/null || true
    sleep 1
    echo "✓ All nodes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for all
wait
