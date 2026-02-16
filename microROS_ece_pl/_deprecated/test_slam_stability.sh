#!/bin/bash
# Test SLAM stability with calibration routines

echo "=== SLAM Stabilization Test Suite ==="
echo ""

if [ $# -eq 0 ]; then
    echo "Usage: $0 <test_name>"
    echo ""
    echo "Available tests:"
    echo "  micro        - Gentle micro oscillations (safe, quick)"
    echo "  scan         - In-place rotation calibration"
    echo "  spiral       - Slow expanding spiral"
    echo "  figure8      - Figure-8 pattern"
    echo "  full         - Full convergence dance (recommended first time)"
    echo ""
    echo "Example:"
    echo "  ./$0 micro"
    exit 1
fi

source /opt/ros/humble/setup.bash
source /home/matheo/microros_ece_ws/install/setup.bash

cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

case "$1" in
    micro)
        echo "Starting MICRO OSCILLATION test..."
        python3 calibration_routine.py --mode micro_oscillation --cycles 5
        ;;
    scan)
        echo "Starting SCAN CALIBRATION test..."
        python3 calibration_routine.py --mode scan_calibration --duration 45
        ;;
    spiral)
        echo "Starting SLOW SPIRAL test..."
        python3 calibration_routine.py --mode slow_spiral --duration 90
        ;;
    figure8)
        echo "Starting FIGURE-8 test..."
        python3 calibration_routine.py --mode figure_eight --duration 90
        ;;
    full)
        echo "Starting FULL CONVERGENCE DANCE (takes ~4 minutes)..."
        python3 calibration_routine.py --mode convergence_dance
        ;;
    *)
        echo "Unknown test: $1"
        exit 1
        ;;
esac

echo ""
echo "Test complete! Check RViz to see map quality."
