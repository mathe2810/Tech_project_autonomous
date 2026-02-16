#!/bin/bash
# Super simple: test if /map gets published

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

echo "Waiting for /map..."
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    echo "✓ /map found at attempt $i"
    echo ""
    echo "Checking /map data..."
    ros2 topic echo /map --once 2>&1 | head -20
    exit 0
  fi
  echo "Attempt $i: no /map yet"
  sleep 1
done

echo "✗ /map never appeared after 30 seconds"
echo ""
echo "Available topics:"
ros2 topic list 2>/dev/null | grep -E "map|slam"
