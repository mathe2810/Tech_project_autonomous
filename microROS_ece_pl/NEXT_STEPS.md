# 🎯 NEXT STEPS - Test & Optimize

## Immediate Actions (Today)

### 1. ✅ Verify Motor Wiring
```
Before uploading firmware, physically check:

LEFT MOTOR:
  PWM     → GPIO 25 (yellow wire)
  DIR1    → GPIO 21 (green wire)
  DIR2    → GPIO 17 (blue wire)

RIGHT MOTOR:
  PWM     → GPIO 26 (yellow wire)
  DIR1    → GPIO 22 (green wire)  
  DIR2    → GPIO 23 (blue wire)

GND     → ESP32 GND
5V (motor power) → Separate 5V supply
```

If pins don't match: Edit main.cpp lines 30-38 before compiling.

### 2. ✅ Compile & Upload
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

# Check compilation
platformio run

# If OK, upload
platformio run --target upload

# If compilation fails:
#  - Check #include paths
#  - Verify Arduino core library installed
#  - Check build_flags in platformio.ini
```

### 3. ✅ Monitor Serial Output
```bash
platformio device monitor

# Expected first 10 seconds:
=== MAPPING FIRMWARE ===
[MOTOR] Initialized
[LIDAR] Start
[IMU] Start
[WiFi] OK 172.20.10.2
..... (waiting for agent)
[MAPPING] Starting circuit...
```

**Motor behavior expected:**
- Motors should start spinning ~3 seconds after boot
- Left + Right wheels both spin forward (moving forward)
- After ~3 seconds: slight turn pattern (one wheel slower)
- Repeat 4 times over ~18 seconds total
- Then stop

If motors don't move: **See Troubleshooting section below**

---

## Testing Phase (First Hour)

### 4. 🚀 Start ROS2 Stack
```bash
# Terminal 1 (on host/robot):
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
./start_stack.sh

# Should show:
[1/4] Starting Micro-ROS Agent on port 8888...
[2/4] Starting RViz2...
[3/4] Starting IMU EMA Filter...
[4/4] Starting Simple SLAM Node...
=== All processes started ===
```

### 5. 📊 Verify ROS2 Topics
```bash
# Terminal 2:
ros2 topic list

# Should show (at minimum):
/map
/scan
/imu/data
/imu/data_filtered  
/slam/pose

# Check data is flowing:
ros2 topic hz /scan
# Should show: approximate frequency: 30.0 Hz

ros2 topic hz /imu/data
# Should show: approximate frequency: 20.0 Hz
```

### 6. 🗺️ Watch Mapping in RViz
```
In RViz window (should auto-open):

1. Left panel → Displays section
2. Click "Add" button
3. Search "LaserScan"
4. Select "LaserScan", click "OK"
5. In properties: Set Topic → /scan
6. Set "Color Transformer" → Intensity
7. Watch white points appear as robot moves!

Repeat for other displays:
- Add "PointCloud2" for /scan_cloud
- Add "OccupancyGrid" for /map (Set topic /map)
- Add "Axes" for /slam/pose (Reference Frame: map)
```

You should see:
- **White laser points** building up
- **Gray occupancy grid** showing where robot has been
- **Red/Green/Blue axes** showing estimated robot position

---

## Optimization Phase (Hours 1-4)

### 7. 🔧 Tune Motor Speed if Needed

**Problem**: Robot doesn't drive straight (one wheel faster)

**Solution**: Edit `motors_drive()` function in main.cpp

```cpp
// Current (line 143):
int left_pwm = throttle + steering;
int right_pwm = throttle - steering;

// If robot veers LEFT (right wheel slower):
// Reduce left speed:
int left_pwm = (throttle + steering) * 0.90;    // 90% of left
int right_pwm = (throttle - steering);

// If robot veers RIGHT (left wheel slower):
// Reduce right speed:
int left_pwm = (throttle + steering);
int right_pwm = (throttle - steering) * 0.90;   // 90% of right

// Adjust 0.90 up/down until straight
// 0.85 = stronger correction
// 0.95 = weaker correction
```

After editing: `platformio run --target upload`

### 8. 🗓️ Verify Circuit Timing

**Question**: Is 18 seconds enough for SLAM?

**Answer**: Check RViz during circuit:
- First 5-6 seconds: Map fills quickly (robot is moving)
- Seconds 6-12: Walls/features appear (corner detection)
- Seconds 12-18: Loop closure attempt (check if map aligns)

**If not enough detail:**
- Extend circuit: change `5000` to `8000` in `update_mapping()` (lines 168-172)
- Add more turns (5+ instead of 4)
- Reduce speed to allow LIDAR more dwell time

### 9. 📈 Monitor Performance Metrics

```bash
# Terminal 3:
watch -n 1 'ros2 topic hz /map'
watch -n 1 'ros2 topic hz /scan'
watch -n 1 'ros2 topic hz /imu/data'

# Targets:
# /scan: ~30 Hz
# /imu/data: ~20 Hz  
# /map: ~30 Hz (updates with each scan)
```

---

## Validation Phase (Hour 4-8)

### 10. ✅ Test Full Loop Closure

**Goal**: Circuit should close with <0.5m error

**Test**:
1. Let robot complete full circuit (18 sec)
2. In RViz: Look at map around starting point
3. Measure gap between start and end of wall

**Metrics**:
- **<0.2m gap**: Excellent (use in production)
- **0.2-0.5m gap**: Good (acceptable for SLAM)
- **0.5-1m gap**: Fair (needs odometry correction)
- **>1m gap**: Drift detected (add loop closure detection)

### 11. 📝 Document Results

Create `TEST_RESULTS.md`:

```markdown
# Motor Control Test Results

Date: [TODAY]
Rover: microROS ECE

## Hardware Verification
- [ ] Motor power connected
- [ ] GPIO pins verified
- [ ] Encoder wires checked (if used)

## Firmware Test
- [ ] Compilation successful
- [ ] Upload successful
- [ ] Serial monitor shows circuit starting

## Motor Behavior
- [ ] Left wheel spins
- [ ] Right wheel spins
- [ ] Both spin together (forward)
- [ ] Differential turning works

## ROS2 Integration
- [ ] Micro-ROS agent connects
- [ ] /scan topic publishing (30 Hz)
- [ ] /imu/data topic publishing (20 Hz)
- [ ] SLAM node receiving scans
- [ ] /map updating

## SLAM Quality
- [ ] Map visible in RViz
- [ ] Walls recognizable
- [ ] Loop closure gap: ±___cm
- [ ] Estimated position: accurate / acceptable / poor

## Performance
- [ ] Circuit completes in ~18 seconds
- [ ] No crashes or resets during test
- [ ] WiFi connection stable
- [ ] IMU data responsive

## Issues Found
1. [Issue 1 - status]
2. [Issue 2 - status]

## Next Optimization
- [ ] Motor speed tuning
- [ ] Circuit timing adjustment
- [ ] Loop closure detection
- [ ] Reactive obstacle avoidance
```

---

## Git Commit (When Stable)

Once everything works:

```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

# Check status
git status

# Add new files
git add -A

# Commit with detailed message
git commit -m "Implement motor control + autonomous circuit mapping

Hardware:
- TB6612 H-Bridge motor driver (GPIO 25/26 PWM, GPIO 17/21/22/23 direction)
- Differential drive control (throttle + steering)

Firmware:
- main_mapping.cpp: New motor control version
- main_sensor_fusion_backup.cpp: Previous stable version
- switch_firmware.sh: Easy version switching

Features:
- Autonomous rectangular circuit (18 sec completion)
- ROS2 LIDAR/IMU publishing (30 Hz / 20 Hz)
- Real-time SLAM mapping via simple_slam.py
- Auto-reconnect agent (no manual reset needed)

Testing:
- [✓] Motor control verified
- [✓] Circuit completes successfully
- [✓] SLAM map builds in real-time
- [✓] Loop closure <0.5m error

Documentation:
- MAPPING_MOTOR_CONTROL.md: Motor control guide
- MAPPING_IMPLEMENTATION_SUMMARY.md: Implementation overview
- NEXT_STEPS.md: Testing & optimization procedures"
```

---

## Future Enhancements (Week 2+)

### Enhancement 1: Encoder-Based PID Speed Control

```cpp
// Framework already exists in SIMPLE_ROVER
// Enable by adding:
#define USE_ENCODER_FEEDBACK 1

// Then use:
pid_set_left_target(1.0);   // Target 1.0 m/s
pid_set_right_target(1.0);
pid_compute();              // Calculates PWM corrections
```

### Enhancement 2: Gyro-Based Heading

```cpp
// Use MPU6050 gyroscope for rotation estimate
float heading = 0;
void update_heading() {
  heading += imu->gz * dt;  // Integrate gyro Z
}
// Use heading for turn accuracy
```

### Enhancement 3: Obstacle Avoidance

```cpp
// Check LIDAR front cone during movement
float front_distance = lidar_get_front_distance();
if (front_distance < 0.5) {  // 50cm
  motors_stop();              // Emergency brake
  // Or: motors_drive(0, 100);  // Swerve right
}
```

### Enhancement 4: Loop Closure Detection

```cpp
// In simple_slam.py:
// Detect when last scan matches first scan
// Apply graph optimization to correct drift
```

---

## Success Criteria ✅

Your mapping implementation is **successful** when:

- [ ] **Motor Control**: Both wheels spin together, turning works smoothly
- [ ] **Circuit**: Robot completes autonomous circuit without crashing
- [ ] **SLAM**: Map visible in RViz, walls recognizable
- [ ] **Precision**: Loop closure error <0.5m
- [ ] **Stability**: 30 Hz LIDAR, 20 Hz IMU, no data drops
- [ ] **ROS2**: All topics publishing correctly
- [ ] **Documentation**: Code commented, README updated

---

## Timeline Estimate

| Phase | Duration | Goal |
|-------|----------|------|
| **Motor Test** | 30 min | Verify motor spinning |
| **ROS2 Setup** | 30 min | Verify topics publishing |
| **RViz Visualization** | 30 min | Watch map build |
| **SLAM Validation** | 1 hour | Verify accuracy |
| **Optimization** | 2-4 hours | Tune for best performance |
| **Documentation** | 1 hour | Document findings |
| **Git Commit** | 15 min | Save to repository |
| **TOTAL** | ~6-8 hours | Full implementation |

---

## Emergency Fallback

If things go wrong:

```bash
# Revert to previous version
./switch_firmware.sh fusion
platformio run --target upload

# Or restore from git
git checkout HEAD -- src/main.cpp
platformio run --target upload

# If WiFi issues: Reduce WiFi update rate
# Edit main.cpp, change WiFi.setSleep(false) → WiFi.setSleep(true)

# If LIDAR noise: Increase CRC validation threshold
# Or reduce LIDAR publish rate from 30 Hz → 15 Hz
```

---

**You've got this! 🚀 Go test the mapping!**

Questions? Check:
1. MAPPING_MOTOR_CONTROL.md (motor guide)
2. SLAM_README.md (SLAM algorithm)
3. main_mapping.cpp comments (code details)
4. ROS2 troubleshooting (network issues)
