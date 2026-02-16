#!/bin/bash
# Real-time SLAM debug script
# Shows all topics, TF tree, and SLAM Toolbox status

echo "=== SLAM TOOLBOX DEBUG DASHBOARD ==="
echo ""

while true; do
  clear
  echo "=== SLAM Toolbox Debug - $(date) ==="
  echo ""
  
  # 1. Topics status
  echo "[1] TOPICS STATUS:"
  echo "  /scan (LIDAR input):"
  timeout 2 ros2 topic hz /scan 2>&1 | head -2 || echo "    ❌ NO DATA"
  
  echo "  /odom (Odometry input):"
  timeout 2 ros2 topic hz /odom 2>&1 | head -2 || echo "    ❌ NO DATA"
  
  echo "  /map (SLAM output):"
  timeout 2 ros2 topic hz /map 2>&1 | head -2 || echo "    ❌ NO DATA"
  
  echo "  /slam_toolbox/pose (SLAM pose):"
  timeout 2 ros2 topic hz /slam_toolbox/pose 2>&1 | head -2 || echo "    ❌ NO DATA"
  
  echo ""
  
  # 2. TF Tree
  echo "[2] TRANSFORM TREE:"
  timeout 2 ros2 run tf2_tools view_frames.py 2>/dev/null && echo "  ✓ TF tree OK" || echo "  ❌ No TF tree"
  
  echo ""
  
  # 3. SLAM Node status
  echo "[3] SLAM TOOLBOX NODE STATUS:"
  pgrep -f "async_slam_toolbox_node" > /dev/null && echo "  ✓ Node running" || echo "  ❌ Node not running"
  
  echo ""
  
  # 4. Sample scan
  echo "[4] LATEST SCAN DATA:"
  timeout 2 ros2 topic echo /scan --once 2>&1 | head -20 || echo "  ❌ Cannot read scan"
  
  echo ""
  echo "Press Ctrl+C to exit. Updating every 5 seconds..."
  sleep 5
done
