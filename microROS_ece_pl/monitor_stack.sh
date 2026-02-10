#!/bin/bash
# Real-time monitoring dashboard for ROS2 stack

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/matheo/microros_ece_ws/install/setup.bash 2>/dev/null

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║          ROS2 NAVIGATION STACK - MONITORING DASHBOARD          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Select what to monitor:"
echo ""
echo "  1) Fused Odometry (MAIN OUTPUT) - /odom_filtered"
echo "  2) Raw Motor Odometry - /odom"
echo "  3) Filtered IMU Data - /imu/data_filtered"
echo "  4) LIDAR Scans - /scan"
echo "  5) SLAM Map - /map"
echo "  6) SLAM Pose Estimate - /slam/pose"
echo "  7) ALL TOPICS (quick view)"
echo "  8) Exit"
echo ""
read -p "Choose (1-8): " choice

case $choice in
  1)
    echo ""
    echo "📊 Monitoring /odom_filtered (Fused Odometry)"
    echo "   Press Ctrl+C to stop"
    echo ""
    ros2 topic echo /odom_filtered
    ;;
  2)
    echo ""
    echo "📊 Monitoring /odom (Raw Motor Odometry)"
    echo "   Press Ctrl+C to stop"
    echo ""
    ros2 topic echo /odom
    ;;
  3)
    echo ""
    echo "📊 Monitoring /imu/data_filtered (Filtered IMU)"
    echo "   Press Ctrl+C to stop"
    echo ""
    ros2 topic echo /imu/data_filtered
    ;;
  4)
    echo ""
    echo "📊 Monitoring /scan (LIDAR)"
    echo "   Press Ctrl+C to stop"
    echo ""
    ros2 topic echo /scan | head -50
    ;;
  5)
    echo ""
    echo "📊 Viewing /map (SLAM Occupancy Grid)"
    echo ""
    ros2 topic echo /map --once
    ;;
  6)
    echo ""
    echo "📊 Viewing /slam/pose (SLAM Estimated Pose)"
    echo ""
    ros2 topic echo /slam/pose --once
    ;;
  7)
    echo ""
    echo "═══ CURRENT STATE OF ALL TOPICS ═══"
    echo ""
    echo "--- ODOMETRY ---"
    echo "/odom:"
    ros2 topic echo /odom --once 2>/dev/null | head -15
    echo ""
    echo "/odom_filtered:"
    ros2 topic echo /odom_filtered --once 2>/dev/null | head -15
    echo ""
    echo "--- IMU ---"
    echo "/imu/data_filtered:"
    ros2 topic echo /imu/data_filtered --once 2>/dev/null | head -15
    echo ""
    echo "--- SLAM ---"
    echo "/slam/pose:"
    ros2 topic echo /slam/pose --once 2>/dev/null | head -10
    echo ""
    echo "/map info:"
    ros2 topic info /map 2>/dev/null
    ;;
  8)
    echo "Exiting..."
    exit 0
    ;;
  *)
    echo "Invalid choice"
    exit 1
    ;;
esac
