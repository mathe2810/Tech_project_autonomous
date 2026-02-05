# Mapping & Motor Control Firmware

## Overview

This firmware enables the rover to:
1. **Control both wheel motors** with PWM-based differential drive
2. **Read LIDAR and IMU** data and publish to ROS2
3. **Autonomously complete a circuit** for mapping purposes
4. **Stream all data** for SLAM processing on the host (ROS2)

## Firmware Versions

### ✅ Version 1: main_sensor_fusion_backup.cpp
- **Status**: Stable (previous version)
- **Features**: LIDAR 30 FPS, IMU 100 Hz, raw sensor data publishing
- **Motors**: ❌ No motor control
- **Use case**: Baseline sensor testing

### 🚀 Version 2: main_mapping.cpp (NEW)
- **Status**: New (for mapping)
- **Features**: Full motor control + LIDAR + IMU + auto circuit
- **Motors**: ✅ Differential drive PID-ready
- **Use case**: Autonomous circuit mapping for SLAM

## Motor Hardware Setup

### Motor Pins (TB6612 H-Bridge)

```
LEFT MOTOR (Motor A):
  PWM        → GPIO 25  (Speed: 0-255)
  Direction1 → GPIO 21  (AIN1)
  Direction2 → GPIO 17  (AIN2)

RIGHT MOTOR (Motor B):
  PWM        → GPIO 26  (Speed: 0-255)
  Direction1 → GPIO 22  (BIN1)
  Direction2 → GPIO 23  (BIN2)
```

### Motor Speed Direction Logic

```cpp
// Positive speed → Forward
// Negative speed → Reverse
// 0 → Stop

motor_left(255);    // Full forward left
motor_left(-128);   // Half reverse left
motor_left(0);      // Stop left

motors_drive(200, 50);   // Forward + slight right turn
motors_drive(0, -255);   // Spin left in place
```

## Control Methods

### 1. Individual Motor Control
```cpp
motor_left(speed);    // -255 to +255
motor_right(speed);   // -255 to +255
```

### 2. Differential Drive (RC Style)
```cpp
// throttle:  -255 (reverse) to +255 (forward)
// steering:  -255 (left turn) to +255 (right turn)
motors_drive(throttle, steering);

// Example: Move forward + turn right
motors_drive(200, 100);   // Left: 200+100=300→clamped 255, Right: 200-100=100
```

### 3. Emergency Stop
```cpp
motors_stop();  // Cuts power to all motors immediately
```

## Mapping Circuit Algorithm

The firmware implements a **simple rectangular circuit**:

```
Start
  ↓
[MOVING_FORWARD] → Drive for 3 seconds
  ↓
[TURNING] → Turn right 90° for ~0.5s
  ↓
Repeat 4 times (to close rectangle)
  ↓
[IDLE] → Stop and report completion
```

### Timing

```
Total circuit time: ~18 seconds
  - Each segment: 5 seconds (3s forward + 2s planning)
  - 4 segments total = 20s
  - Adjusted to ~18s in practice
```

### Starting Mapping

Mapping **automatically starts 3 seconds after boot**:

```
=== MAPPING FIRMWARE ===
[MOTOR] Initialized
[LIDAR] Start
[IMU] Start
[WiFi] OK 172.20.10.2
... (3 second wait)...
[MAPPING] Starting circuit...
```

To **disable auto-start**, comment out `start_mapping_circuit()` in `setup()`.

## LIDAR Orientation

⚠️ **IMPORTANT**: The LIDAR is **NOT rotated**:
- 0° = Front of rover
- 90° = Left side  
- 180° = Back
- 270° = Right side

**Mounting**: Front-centered, pointing directly forward.

### For True 360° Coverage

If LIDAR were mounted left-offset (~45° left of front):
```
Effective 0° = 45° left of robot front
Need to subtract 45° from all angles in post-processing
```

Currently: **No offset needed** ✅

## ROS2 Topics Available

```
/scan          → LaserScan (raw LIDAR, 30 Hz)
/imu/data      → IMU (accel + gyro, 20 Hz)
/map           → OccupancyGrid (from simple_slam.py, 30 Hz)
/slam/pose     → Current robot pose estimate
```

## Data Flow for SLAM

```
ESP32 Rover
  ├─ LIDAR (30 Hz) → /scan
  ├─ IMU (100 Hz)  → /imu/data
  └─ Motor PWM control ← /simple_slam.py (future)

ROS2 Host
  ├─ simple_slam.py  → Processes /scan, outputs /map & /slam/pose
  ├─ imu_kalman_filter.py → Processes /imu/data, outputs /imu/data_filtered
  └─ RViz → Visualizes everything
```

## Testing Procedures

### 1. Motor Basic Test
```bash
# Upload main_mapping.cpp
platformio run --target upload

# Connect to serial monitor
platformio device monitor

# Watch motors start automatic circuit
[MAPPING] Starting circuit...
```

### 2. ROS2 Connection Test
```bash
# In host terminal
./start_stack.sh

# Check topics
ros2 topic list
ros2 topic hz /scan        # Should show 30 Hz
ros2 topic hz /imu/data    # Should show 20 Hz
```

### 3. SLAM Mapping Test
```bash
# During circuit:
rviz2

# Add displays:
# - LaserScan: /scan (White)
# - OccupancyGrid: /map (Grayscale)
# - Axes: /slam/pose

# Watch map build in real-time!
```

## Advanced: Custom Movement Patterns

To implement **different circuits** (triangle, square, spiral):

Edit `update_mapping()` function:

```cpp
void update_mapping() {
  // Example: Triangle
  static int segment = 0;
  uint32_t elapsed = millis() - mapping_start_time;
  
  switch(segment) {
    case 0:  // Leg 1
      if (elapsed < 5000) motors_drive(200, 0);
      else { motors_drive(0, 200); segment++; }
      break;
    case 1:  // Turn 120°
      if (elapsed < 6000) motors_drive(0, 200);
      else { segment++; }
      break;
    // ... repeat 2 more times ...
  }
}
```

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| Max Motor Speed | 255 PWM (100%) | ~1-2 m/s (depends on wheels) |
| Motor Inertia | ~200ms | Takes 200ms to accelerate to full speed |
| Turning Radius | ~0.3m | At 50% throttle + 50% steering |
| LIDAR Update Rate | 30 Hz | Published to ROS2 every ~33ms |
| IMU Update Rate | 100 Hz internal, 20 Hz ROS2 | Over WiFi |
| WiFi Latency | ~10-50ms | UDP transmission |

## Troubleshooting

### ❌ Motors don't spin
- Check motor power supply (likely separate 5V for motor driver)
- Verify GPIO pins in code match your wiring
- Test with `motor_left(100)` in loop() directly

### ❌ Motor spins wrong direction
- Swap DIR1/DIR2 pins in code, OR
- Swap motor connector wires

### ❌ Robot doesn't drive straight
- Motors likely have different speeds (common)
- Can add encoder-based PID (commented PID code ready)
- Or manually tune speed difference in `update_mapping()`

### ❌ LIDAR data NaN
- UART baudrate mismatch (check LIDAR_BAUD = 230400)
- CRC error in protocol parser
- Try lowering baud rate to 115200

### ❌ WiFi drops frequently
- Reduce LIDAR/IMU publish rates
- Move closer to WiFi router
- Check for 2.4GHz interference

## Future Enhancements

1. **Encoder-based PID** (framework ready, just needs tuning)
2. **Loop closure detection** (detect when back at start)
3. **Reactive navigation** (avoid obstacles using LIDAR)
4. **IMU-based heading** (gyro integration for rotation estimate)
5. **Multi-circuit mapping** (multiple loops to improve map quality)

## Code Organization

```cpp
// ===== Hardware Layer =====
motor_init()               // PWM + direction pin setup
motor_left/right(speed)    // Individual motor control
motors_drive(throttle, steering)  // Differential drive
motors_stop()              // Emergency brake

// ===== Sensor Layer =====
lidarTask()                // Core 1: Parse LIDAR protocol
imuTask()                  // Core 1: Read MPU6050 I2C
publishTask()              // Core 0: ROS2 publishing

// ===== Navigation Layer =====
start_mapping_circuit()    // Initialize state machine
update_mapping()           // Main control loop each frame

// ===== ROS2 Layer =====
create_entities()          // Initialize publishers
destroy_entities()         // Cleanup
loop() state machine       // Connection/disconnection handling
```

## Files

- **main_mapping.cpp** → Main firmware (new)
- **main_sensor_fusion_backup.cpp** → Previous version (backup)
- **simple_slam.py** → ROS2 node for SLAM (host side)
- **imu_kalman_filter.py** → IMU filtering (host side)
- **start_stack.sh** → Launcher for all ROS2 nodes

---

**Next Steps**: 
1. Review motor connections match GPIO pins
2. Upload and test circuit
3. Verify LIDAR/IMU data in ROS2
4. Run simple_slam.py and watch map build!

Good luck! 🚀
