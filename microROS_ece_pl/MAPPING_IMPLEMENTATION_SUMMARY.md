# 🚀 Motor Control & Autonomous Mapping - Implementation Summary

## What Was Done

### ✅ Step 1: Code Review
- Read **SIMPLE_ROVER** architecture & motor control implementation
- Analyzed **Motor pins** (TB6612 H-Bridge), **PWM configuration**, **differential drive**
- Reviewed **LIDAR orientation** (NOT rotated - points directly forward)
- Studied **PID controller** framework and **encoder feedback** system

### ✅ Step 2: Firmware Versions Created

#### Version A: main_sensor_fusion_backup.cpp
- **Backup** of previous stable version (LIDAR + IMU, no motors)
- Saved in case we need to roll back
- Still fully functional

#### Version B: main_mapping.cpp ⭐ **NEW**
- **Full motor control** with differential drive
- **Autonomous circuit mapping** (auto-starts 3s after boot)
- **LIDAR + IMU** data streams to ROS2 for SLAM
- **TB6612 H-Bridge** integration (GPIO 25/26 for PWM, GPIO 21/22/17/23 for direction)
- **State machine** for connection management (same as before)

### ✅ Step 3: Motor Control Functions

Three control methods available:

```cpp
// Method 1: Individual motors (-255 to +255)
motor_left(200);      // Left wheel forward
motor_right(-100);    // Right wheel reverse

// Method 2: Differential drive (RC-style, recommended)
motors_drive(200, 50);  // throttle, steering
// Left = 200+50 = 250, Right = 200-50 = 150
// Result: Forward + turn right

// Method 3: Emergency stop
motors_stop();
```

### ✅ Step 4: Autonomous Circuit Algorithm

**State Machine**:

```
IDLE
  ↓ (start_mapping_circuit)
MOVING_FORWARD (3 sec) → motors_drive(200, 0)
  ↓
TURNING (0.5 sec) → motors_drive(0, 200)
  ↓ (repeat 4 times)
IDLE (done!)
```

**Total time**: ~18 seconds to complete rectangular circuit

**Auto-starts** 3 seconds after boot with:
```
[MAPPING] Starting circuit...
```

### ✅ Step 5: Documentation

Created comprehensive guide: **MAPPING_MOTOR_CONTROL.md**
- Motor hardware pinout
- Control methods with examples
- LIDAR orientation (no rotation needed ✅)
- ROS2 topics available
- SLAM data flow diagram
- Testing procedures
- Troubleshooting guide

### ✅ Step 6: Firmware Switcher

Created **switch_firmware.sh** for easy version management:

```bash
./switch_firmware.sh mapping      # Switch to motor control version
./switch_firmware.sh fusion       # Switch to sensor fusion version
./switch_firmware.sh status       # Check current version
```

---

## 📋 File Structure

```
microROS_ece_pl/
├── src/
│   ├── main.cpp                           ← Use switch_firmware.sh to select
│   ├── main_mapping.cpp                   ← NEW: Motor control + circuit
│   └── main_sensor_fusion_backup.cpp      ← OLD: LIDAR + IMU only
│
├── imu_kalman_filter.py                  ← ROS2: IMU filtering (EMA)
├── simple_slam.py                        ← ROS2: SLAM mapping
├── start_stack.sh                        ← Launch all ROS2 nodes
├── switch_firmware.sh                    ← NEW: Version switcher
│
├── MAPPING_MOTOR_CONTROL.md              ← NEW: Motor control guide
├── SLAM_README.md                        ← SLAM implementation details
└── ...other docs
```

---

## 🔧 Hardware Verification

### Motor Pins (Check Your Wiring!)

| Function | ESP32 GPIO | H-Bridge Pin |
|----------|-----------|--------------|
| Left PWM | 25 | PWMA |
| Left Dir1 | 21 | AIN1 |
| Left Dir2 | 17 | AIN2 |
| Right PWM | 26 | PWMB |
| Right Dir1 | 22 | BIN1 |
| Right Dir2 | 23 | BIN2 |

⚠️ **VERIFY YOUR WIRING BEFORE UPLOADING**

If motors connected differently, edit GPIO pins in `main_mapping.cpp` lines 30-38.

---

## 🚀 Quick Start

### 1. Select Firmware Version
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
./switch_firmware.sh mapping
```

### 2. Compile & Upload
```bash
platformio run --target upload
```

### 3. Monitor Serial Output
```bash
platformio device monitor
```

**Expected output** (first 5 seconds):
```
=== MAPPING FIRMWARE ===
[MOTOR] Initialized
[LIDAR] Start
[IMU] Start
[WiFi] OK 172.20.10.2
[MAPPING] Starting circuit...
```

### 4. Start ROS2 Stack (on host)
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
./start_stack.sh
```

### 5. Watch in RViz
- Add **LaserScan** display → topic `/scan` (White points)
- Add **OccupancyGrid** display → topic `/map` (Gray occupied cells)
- Add **TF** display → topic `/slam/pose` (Axes showing robot position)
- Watch map build in real-time as robot drives circuit! 🗺️

---

## 🎯 Motor Control Examples

### Drive Forward at 50% Speed
```cpp
motors_drive(127, 0);  // 50% of 255 = 127
```

### Turn Right 30° while Moving
```cpp
motors_drive(200, 75);  // Forward 200, right turn 75
// Left wheel: 200+75=275→clamped 255 (faster)
// Right wheel: 200-75=125 (slower)
// = Forward with rightward arc
```

### Spin in Place (Left)
```cpp
motors_drive(0, -255);  // No forward, full left turn
// Left wheel: 0+(-255) = -255 (reverse)
// Right wheel: 0-(-255) = 255 (forward)
// = Counterclockwise rotation
```

### Emergency Stop
```cpp
motors_stop();  // Immediate zero PWM on all motors
```

---

## 📊 Performance Targets

| Metric | Value | Notes |
|--------|-------|-------|
| Motor response time | <20ms | PWM instant |
| Circuit completion | 18 seconds | Full rectangular loop |
| LIDAR publish rate | 30 Hz | ~33ms per scan |
| IMU publish rate | 20 Hz | ~50ms per message |
| WiFi latency | 10-50ms | Depends on router |
| **SLAM update rate** | 30 Hz | Real-time mapping |

---

## 🔍 Data Flow for SLAM

```
ESP32 ROVER
├─ Motor Control (PWM) ← autonomous circuit
├─ LIDAR (30 Hz) → publishes /scan (LaserScan)
├─ IMU (100 Hz) → publishes /imu/data (IMU message)
└─ Auto-detects agent @ 1s intervals

↓ WiFi UDP ↓

ROS2 HOST
├─ micro_ros_agent ← listens port 8888
├─ simple_slam.py
│   ├─ subscribes /scan (raw 360° points)
│   ├─ computes ICP scan matching
│   ├─ tracks robot pose
│   └─ publishes /map (occupancy grid) + /slam/pose
├─ imu_kalman_filter.py
│   ├─ subscribes /imu/data (raw)
│   └─ publishes /imu/data_filtered (EMA-filtered)
└─ RViz ← visualizes map + LIDAR + robot pose
```

---

## 🧪 Testing Procedure

### Phase 1: Motor Verification
```bash
# Upload main_mapping.cpp
./switch_firmware.sh mapping
platformio run --target upload
platformio device monitor

# Expected: Motors spin in circuit pattern for 18 seconds
# If motors don't move:
#  - Check power supply to motor driver
#  - Check GPIO pin configuration
#  - Test with fixed command: motors_drive(200, 0)
```

### Phase 2: ROS2 Connection
```bash
./start_stack.sh

# In another terminal:
ros2 topic list
# Should show: /scan, /imu/data, /map, /slam/pose, etc.

ros2 topic hz /scan
# Should show ~30 Hz
```

### Phase 3: SLAM Visualization
```bash
# In RViz (launched by start_stack.sh):
# Watch the map build in real-time as robot drives circuit
# - White points = LIDAR hits
# - Gray cells = occupied spaces
# - Red axes = robot position from SLAM
```

### Phase 4: Loop Closure Test
```bash
# After circuit completes (18 sec):
# Check if map closes properly
# - If perfect: no drift
# - If gap ~0.5m: good alignment
# - If gap >1m: need loop closure optimization
```

---

## 🐛 Troubleshooting

### Motors Don't Spin
- ❌ Check power connector to motor driver
- ❌ Verify ESP32 GPIO pins match actual wiring
- ❌ Test individual pins: `motor_left(100); delay(2000); motor_left(0);`

### Motors Spin Wrong Direction
- ❌ Swap DIR1/DIR2 pins in code, OR
- ❌ Reverse physical motor connector

### Robot Drives in Circle Instead of Straight
- ❌ Differential motor speeds (very common!)
- ❌ Add speed balancing in `motors_drive()`:
  ```cpp
  int left_pwm = (throttle + steering) * 0.95;  // Adjust 0.95
  int right_pwm = (throttle - steering);
  ```

### LIDAR Scans Are NaN
- ❌ UART baud rate mismatch (verify 230400)
- ❌ CRC parser error (check serial data with logic analyzer)
- ❌ Try reducing to 115200 baud

### Map Not Building in RViz
- ❌ Check simple_slam.py is running: `ros2 node list`
- ❌ Verify /scan topic has data: `ros2 topic echo /scan`
- ❌ Check /map topic exists: `ros2 topic hz /map`

---

## 🔄 Version Management

### Current Version
```bash
./switch_firmware.sh status
```

### Switch to Mapping
```bash
./switch_firmware.sh mapping
platformio run --target upload
```

### Switch to Sensor Fusion (Previous)
```bash
./switch_firmware.sh fusion
platformio run --target upload
```

### Backup System
- Original version always saved as `main_sensor_fusion_backup.cpp`
- Current `main.cpp` is symlinked to either mapping or fusion
- Easy to revert: `git checkout src/main.cpp`

---

## 📈 Next Enhancements

1. **Encoder-based PID** (framework ready)
   - Use wheel encoders to measure actual speed
   - Compensate for motor mismatch
   - Smooth acceleration/deceleration

2. **Gyro-based Heading** (from IMU)
   - Track rotation with gyroscope
   - Better turn accuracy
   - Deadreckoning backup

3. **Loop Closure Detection**
   - Detect when robot returns to start
   - Use scan correlation (place recognition)
   - Correct accumulated drift

4. **Reactive Obstacle Avoidance**
   - Check LIDAR front cone during movement
   - Auto-stop if obstacle <50cm
   - Or swerve around obstacles

5. **Multi-loop Mapping**
   - Complete circuit N times
   - Average multiple observations
   - Better map quality

---

## 📖 Reading Order

1. ✅ This file (overview)
2. ✅ MAPPING_MOTOR_CONTROL.md (detailed motor guide)
3. ✅ SLAM_README.md (mapping algorithm)
4. ✅ main_mapping.cpp code comments

---

## ✨ Summary

You now have:

✅ **Two firmware versions** for easy A/B testing  
✅ **Autonomous circuit mapping** that auto-starts  
✅ **Motor control ready** with 3 control methods  
✅ **Full ROS2 integration** (LIDAR + IMU + SLAM)  
✅ **Comprehensive documentation** & troubleshooting  
✅ **Easy version switching** with script  

**Next**: Upload main_mapping.cpp and watch your rover map the world! 🗺️🚀

---

**Last Updated**: February 5, 2026  
**Status**: Ready for testing ✅
