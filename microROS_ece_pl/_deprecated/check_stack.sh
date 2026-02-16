#!/bin/bash
# Check ROS2 stack status

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash 2>/dev/null || true

echo "=== ROS2 Stack Status ==="
echo ""

# Check agent
if pgrep -f "micro_ros_agent" > /dev/null; then
  AGENT_PID=$(pgrep -f "micro_ros_agent" | head -1)
  echo "✓ Micro-ROS Agent: RUNNING (PID: $AGENT_PID)"
else
  echo "✗ Micro-ROS Agent: NOT RUNNING"
  echo "  → Start with: ./start_agent.sh"
fi

echo ""

# Check ROS nodes
echo "ROS2 Nodes:"
NODES=$(ros2 node list 2>/dev/null | grep -E "motor_odom|imu_fir|simple_slam|kalman_filter" || echo "")
if [ -z "$NODES" ]; then
  echo "  ✗ No navigation nodes running"
  echo "  → Start with: ./start_stack.sh"
else
  echo "$NODES" | while read node; do
    echo "  ✓ $node"
  done
fi

echo ""

# Check topics
echo "Active Topics (sample):"
TOPICS=$(ros2 topic list 2>/dev/null | grep -E "scan|imu|odom|map" || echo "")
if [ -z "$TOPICS" ]; then
  echo "  (none)"
else
  echo "$TOPICS" | while read topic; do
    echo "  • $topic"
  done
fi

echo ""
echo "=== Quick Commands ==="
echo "  ./start_agent.sh          - Start Micro-ROS Agent (do ONCE)"
echo "  ./start_stack.sh          - Start ROS2 nodes (can restart)"
echo "  ros2 topic list           - List all topics"
echo "  ros2 node list            - List all nodes"
echo "  pkill -f motor_odom       - Kill a specific node"
