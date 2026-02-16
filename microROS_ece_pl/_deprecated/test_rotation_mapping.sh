#!/bin/bash
# Quick test script to validate the mapping rotation fix

echo "╔════════════════════════════════════════════════════════════╗"
echo "║    MAPPING ROTATION FIX - VERIFICATION TEST               ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if ros2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Make sure ROS2 is sourced."
    exit 1
fi

echo "✓ ROS2 found"

# Check covariance values
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1️⃣  Checking motor_odom yaw covariance (should be >0.5)..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# This will timeout if motor_odom not running - that's OK
timeout 2 bash -c 'ros2 topic echo /odom/pose/covariance --once 2>/dev/null' | grep -A 35 "data:" | tail -1 || echo "  ⓘ Motor odometry not running yet (will start in stack)"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2️⃣  Checking SLAM config..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if grep -q "minimum_travel_heading: 0.15" /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/config/slam_toolbox_params.yaml; then
    echo "  ✅ SLAM minimum_travel_heading = 0.15 (correct)"
else
    echo "  ❌ SLAM minimum_travel_heading not 0.15 (check config)"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3️⃣  Files modified:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

files=(
    "motor_odom_node.py"
    "simple_ekf.py"
    "config/slam_toolbox_params.yaml"
)

for file in "${files[@]}"; do
    path="/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/$file"
    if [ -f "$path" ]; then
        echo "  ✅ $file ($(wc -l < "$path") lines)"
    else
        echo "  ❌ $file NOT FOUND"
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📋 NEXT STEPS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "1. Start the stack:"
echo "   $ ./start_stack.sh"
echo ""
echo "2. In another terminal, run diagnostics:"
echo "   $ python3 diagnostic_monitor.py"
echo ""
echo "3. In third terminal, test teleop:"
echo "   $ python3 teleop_keyboard.py"
echo ""
echo "4. Test sequence:"
echo "   - Press W (forward) → check mapping is good"
echo "   - Press D (rotate)  → check mapping stays good ✨"
echo "   - Watch RViz for continuous map updates"
echo ""
echo "5. Expected result:"
echo "   ✓ Position Error < 0.1m"
echo "   ✓ Yaw Error < 15°"
echo "   ✓ Continuous map in RViz (no jumps)"
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║         Ready to test! Run './start_stack.sh'             ║"
echo "╚════════════════════════════════════════════════════════════╝"
