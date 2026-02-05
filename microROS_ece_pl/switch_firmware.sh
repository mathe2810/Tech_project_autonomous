#!/bin/bash
# Switch between firmware versions

set -e

CURRENT_DIR=$(pwd)
MAIN_FILE="src/main.cpp"
MAPPING_FILE="src/main_mapping.cpp"
BACKUP_FILE="src/main_sensor_fusion_backup.cpp"

echo "=== Firmware Version Switcher ==="
echo ""
echo "Available versions:"
echo "1) MAPPING (motor control + auto circuit)"
echo "2) SENSOR_FUSION (LIDAR + IMU, no motors) [BACKUP]"
echo ""

if [ "$1" == "mapping" ] || [ "$1" == "1" ]; then
    echo "Switching to: MAPPING"
    if [ -f "$BACKUP_FILE" ]; then
        # Save current as fusion
        cp "$MAIN_FILE" "$BACKUP_FILE" 2>/dev/null || true
    fi
    cp "$MAPPING_FILE" "$MAIN_FILE"
    echo "✅ Switched to main_mapping.cpp"
    echo ""
    echo "To compile: platformio run --target upload"
    
elif [ "$1" == "fusion" ] || [ "$1" == "2" ]; then
    echo "Switching to: SENSOR_FUSION (backup)"
    if [ -f "$BACKUP_FILE" ]; then
        cp "$BACKUP_FILE" "$MAIN_FILE"
        echo "✅ Switched to main_sensor_fusion_backup.cpp"
    else
        echo "❌ Backup file not found!"
        exit 1
    fi
    echo ""
    echo "To compile: platformio run --target upload"
    
elif [ "$1" == "status" ]; then
    echo "Current main.cpp:"
    if grep -q "MAPPING FIRMWARE" "$MAIN_FILE" 2>/dev/null; then
        echo "  🚀 MAPPING (Motor control)"
    elif grep -q "SENSOR FUSION" "$MAIN_FILE" 2>/dev/null; then
        echo "  📡 SENSOR_FUSION (LIDAR + IMU)"
    else
        echo "  ❓ Unknown"
    fi
    echo ""
    echo "Available files:"
    ls -lh src/main*.cpp
    
else
    echo "Usage:"
    echo "  $0 mapping     Switch to mapping firmware"
    echo "  $0 fusion      Switch to sensor fusion firmware"
    echo "  $0 status      Show current version"
    echo ""
    exit 1
fi
