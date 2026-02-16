#!/bin/bash
# Test script to verify static map SLAM configuration

echo "=== SLAM Static Map Configuration Test ==="
echo ""

# Check if ROS is sourced
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not sourced. Run:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✓ ROS2 sourced"
echo ""

# Check running nodes
echo "=== Checking Running Nodes ==="
echo ""

echo "Checking micro-ROS agent..."
if ros2 node list 2>/dev/null | grep -q micro_ros_agent; then
    echo "  ✅ micro_ros_agent running"
else
    echo "  ❌ micro_ros_agent NOT running"
fi

echo "Checking motor odometry node..."
if ros2 node list 2>/dev/null | grep -q motor_odom; then
    echo "  ✅ motor_odom running"
else
    echo "  ⚠️  motor_odom NOT running (required for static map)"
fi

echo "Checking SLAM node..."
if ros2 node list 2>/dev/null | grep -q simple_slam; then
    echo "  ✅ simple_slam running"
else
    echo "  ⚠️  simple_slam NOT running"
fi

echo ""
echo "=== Checking Topics ==="
echo ""

echo "Checking /odom topic..."
if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
    echo "  ✅ /odom topic published"
    echo "    Type: $(ros2 topic info /odom 2>/dev/null | grep "Type:" | cut -d: -f2)"
else
    echo "  ❌ /odom topic NOT found"
fi

echo "Checking /scan topic..."
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "  ✅ /scan topic published"
else
    echo "  ❌ /scan topic NOT found"
fi

echo "Checking /map topic..."
if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    echo "  ✅ /map topic published"
else
    echo "  ⚠️  /map topic NOT found (SLAM not running)"
fi

echo ""
echo "=== Checking Transforms ==="
echo ""

# Try to show transform tree
echo "Transform hierarchy:"
if command -v ros2 &> /dev/null; then
    ros2 run tf2_tools tf2_tree 2>/dev/null | head -10 || echo "  (Run 'ros2 run tf2_tools tf2_tree' to see full hierarchy)"
fi

echo ""
echo "=== Quick Verification ==="
echo ""

# Sample odom values
echo "Latest /odom message (first 1):"
timeout 2 ros2 topic echo /odom --once 2>/dev/null | head -15 || echo "  (No /odom data received)"

echo ""
echo "=== Configuration Summary ==="
echo ""
echo "For static map behavior, verify:"
echo "  1. motor_odom_node publishes /odom with frame_id='odom', child_frame_id='base_link'"
echo "  2. simple_slam subscribes to /odom for pose source"
echo "  3. simple_slam publishes /map with frame_id='map'"
echo "  4. simple_slam publishes map->odom transform"
echo ""
echo "If map still moves:"
echo "  1. Check RViz Fixed Frame is set to 'map' (not 'odom')"
echo "  2. Run: ros2 run tf2_tools tf2_tree"
echo "  3. Run: ros2 topic echo /odom (should show position changes)"
echo ""
echo "=== Test Complete ==="
