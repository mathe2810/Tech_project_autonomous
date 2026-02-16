#!/bin/bash
# Quick test to verify SLAM drift fix

echo "╔═════════════════════════════════════════════════════════════╗"
echo "║     SLAM DRIFT FIX - Configuration Verification             ║"
echo "╚═════════════════════════════════════════════════════════════╝"
echo ""

CONFIG="/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/config/slam_toolbox_params.yaml"

echo "Checking SLAM Toolbox Parameters..."
echo ""

# Extract and display key parameters
echo "Key Parameters:"
grep "map_update_interval:" "$CONFIG" | sed 's/^/  /'
grep "resolution:" "$CONFIG" | sed 's/^/  /'
grep "minimum_travel_distance:" "$CONFIG" | sed 's/^/  /'
grep "minimum_travel_heading:" "$CONFIG" | sed 's/^/  /'
grep "scan_buffer_size:" "$CONFIG" | sed 's/^/  /'

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Expected Improvements:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "✅ Map Updates: 5x FASTER (500ms → 100ms)"
echo "   → Less drift accumulation"
echo ""
echo "✅ Resolution: 2.5x FINER (2.5cm → 1cm)"
echo "   → Better ICP matching = less error"
echo ""
echo "✅ Scans Processed: 2.5x MORE (0.05m → 0.02m)"
echo "   → More anchors = better localization"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "🧪 TEST NOW:"
echo ""
echo "Terminal 1:"
echo "  $ ./start_stack.sh"
echo ""
echo "Terminal 2:"
echo "  $ python3 teleop_keyboard.py"
echo ""
echo "RViz: Do a square path:"
echo "  W → D → S → A → W (back to start)"
echo ""
echo "Expected: Minimal drift, return close to start ✨"
echo ""
echo "╔═════════════════════════════════════════════════════════════╗"
echo "║           Ready to test with new configuration!             ║"
echo "╚═════════════════════════════════════════════════════════════╝"
