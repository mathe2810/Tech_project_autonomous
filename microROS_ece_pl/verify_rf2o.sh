#!/bin/bash
# Verify RF2O is working with simulated LIDAR

echo "=========================================="
echo "RF2O + LIDAR SIMULATION - FINAL CHECK"
echo "=========================================="
echo ""

# Check if ROS 2 is initialized
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 not found - source setup.bash first"
    exit 1
fi

echo "✅ ROS 2 found"

# Wait a bit for all nodes to stabilize
echo ""
echo "Checking nodes..."
sleep 2

# Check nodes
echo ""
NODES=$(ros2 node list)
echo "Nodes running:"
echo "$NODES"

# Check topics
echo ""
echo "Topics:"
ros2 topic list

# Check data flow
echo ""
echo "=========================================="
echo "DATA FLOW CHECK"
echo "=========================================="

echo ""
echo "1️⃣ /scan_raw status:"
if ros2 topic info /scan_raw 2>/dev/null | grep -q "Publisher count: 1"; then
    echo "   ✅ simu_bridge is publishing"
else
    echo "   ❌ No publisher on /scan_raw"
fi

echo ""
echo "2️⃣ /scan status:"
if ros2 topic info /scan 2>/dev/null | grep -q "Publisher count: 1"; then
    echo "   ✅ scan_restamper is publishing"
else
    echo "   ❌ No publisher on /scan"
fi

echo ""
echo "3️⃣ /odom status:"
ODOM_INFO=$(ros2 topic info /odom 2>/dev/null)
PUB_COUNT=$(echo "$ODOM_INFO" | grep "Publisher count:" | awk '{print $3}')
SUB_COUNT=$(echo "$ODOM_INFO" | grep "Subscription count:" | awk '{print $3}')

if [ "$PUB_COUNT" = "1" ]; then
    echo "   ✅ RF2O IS PUBLISHING ODOMETRY (publishers: $PUB_COUNT)"
    echo "   ✅ Data is flowing!"
else
    echo "   ❌ RF2O NOT publishing (publishers: $PUB_COUNT, subscribers: $SUB_COUNT)"
fi

# Check TF
echo ""
echo "4️⃣ Transform status:"
if ros2 tf2_py tf_echo base_link laser_link 2>/dev/null | grep -q "Translation"; then
    echo "   ✅ TF2 transform exists"
else
    echo "   ❌ TF2 transform missing"
fi

echo ""
echo "=========================================="
echo "Sample message from /odom (if publishing):"
ros2 topic echo /odom --only-n-messages=1 2>/dev/null | head -20 || echo "   (No messages yet)"

echo ""
echo "=========================================="
