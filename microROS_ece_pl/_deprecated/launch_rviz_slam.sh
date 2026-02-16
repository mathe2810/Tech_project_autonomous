#!/bin/bash
# Launch RViz with SLAM configuration

STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$STACK_DIR/rviz_config.rviz"

source /opt/ros/humble/setup.bash

echo "🎨 Launching RViz with SLAM Toolbox config..."
echo "📁 Config: $CONFIG"

rviz2 -d "$CONFIG"
