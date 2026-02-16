#!/bin/bash
# SLAM Toolbox + Motor Odometry Fusion (for static LIDAR)
# Strategy: Motor odometry + RF2O together give SLAM movement signals

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================================"
echo "SLAM Toolbox + Motor Odometry Fusion"
echo "================================================"

# Kill any existing processes
pkill -f "slam_toolbox|rf2o|scan_rest|motor_odom|teleop" 2>/dev/null || true
sleep 2

# 1. Static Transform (base_link -> laser_link)
echo "[1/5] Starting Static Transform Publisher..."
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.15 0 0 0 base_link laser_link &
TF_PID=$!
sleep 1

# 2. Scan Restamper (synchronize LIDAR timestamps)
echo "[2/5] Starting Scan Restamper..."
python3 scan_restamper_simple.py &
RESTAMPER_PID=$!
sleep 2

# 3. RF2O Laser Odometry (LIDAR-based odometry)
echo "[3/5] Starting RF2O Laser Odometry..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -p scan_topic:=/scan \
  -p odom_topic:=/odom_lidar \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p freq:=20.0 \
  -p publish_tf:=false \
  -r /odom_rf2o:=/odom_lidar &
RF2O_PID=$!
sleep 3

# 4. Motor Odometry (publishes /odom_motor based on cmd_vel)
echo "[4/5] Starting Motor Odometry..."
python3 motor_odom_simple.py &
MOTOR_PID=$!
sleep 2

# 5. Odometry Merger - Publishes /odom (fusion of motor + rf2o)
echo "[5/5] Starting Odometry Merger..."
python3 - <<'EOF' &
#!/usr/bin/env python3
"""Merge motor_odom + rf2o into /odom for SLAM"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomMerger(Node):
    def __init__(self):
        super().__init__('odom_merger')
        self.motor_odom = None
        self.lidar_odom = None
        
        # Subscribe to both sources
        self.create_subscription(Odometry, '/odom_motor', self.motor_cb, 10)
        self.create_subscription(Odometry, '/odom_lidar', self.lidar_cb, 10)
        
        # Publish merged odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.1, self.merge_callback)
        self.get_logger().info("Odometry Merger: motor + LIDAR -> /odom")
    
    def motor_cb(self, msg):
        self.motor_odom = msg
    
    def lidar_cb(self, msg):
        self.lidar_odom = msg
    
    def merge_callback(self):
        # MAXIMIZE LIDAR TRUST: Prefer RF2O (LIDAR-based odometry)
        # Use motor_odom only if RF2O unavailable
        if self.lidar_odom is not None:
            odom = self.lidar_odom
        elif self.motor_odom is not None:
            odom = self.motor_odom
        else:
            return
        
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        self.odom_pub.publish(odom)

rclpy.init()
node = OdomMerger()
rclpy.spin(node)
EOF
MERGER_PID=$!
sleep 2

# 6. SLAM Toolbox
echo "[6/6] Starting SLAM Toolbox (async mode)..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file config/slam_toolbox_rf2o.yaml &
SLAM_PID=$!
sleep 3

echo ""
echo "================================================"
echo "Stack running!"
echo "================================================"
echo "Topics:"
echo "  /scan          (LIDAR scans)"
echo "  /odom_motor    (Motor odometry)"
echo "  /odom_lidar    (RF2O odometry)"
echo "  /odom          (Merged - used by SLAM)"
echo "  /map           (Generated map)"
echo ""
echo "Test with:"
echo "  ros2 topic hz /scan /odom /map"
echo "  rviz2 -d rviz_config.rviz"
echo ""
echo "Send movement commands:"
echo "  python3 test_cmd_vel.py"
echo ""
echo "Quit: pkill -f 'slam_toolbox|rf2o|scan_rest|motor_odom|merger'"
echo "================================================"

# Keep script alive
wait
