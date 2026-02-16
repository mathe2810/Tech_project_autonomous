# Micro-ROS Firmware Workspace

## Overview
This workspace contains only the **ESP32 firmware** code for micro-ROS communication.

## Directory Structure

```
microROS_ece_pl/
├── src/
│   └── main.cpp          ← Main firmware code with micro-ROS setup
├── include/              ← C++ header files
├── motor_control.h       ← Motor driver abstraction (Waveshare TB6612)
├── platformio.ini        ← PlatformIO configuration
└── README.md             ← Firmware documentation
```

## What Goes HERE

✅ **Firmware Code:**
- `main.cpp` - micro-ROS initialization, publishers, subscribers
- `motor_control.h` - Low-level motor control library
- `platformio.ini` - Build configuration

✅ **Firmware Documentation:**
- README.md, QUICKSTART.md, SLAM_README.md
- Architecture diagrams and technical guides

✅ **Firmware Scripts:**
- `imu_fft_analysis.py` - Sensor data analysis tools
- `imu_test_node.py` - Basic sensor testing

❌ **What Does NOT Go HERE:**
- Motor test scripts (→ `/home/matheo/ros_test/ros2_ece_ws/`)
- ROS2 application nodes (→ ros2_ece_ws)
- ROS2 launch files (→ ros2_ece_ws)

---

## Workspace Separation

The project uses **two separate workspaces** to maintain clean separation of concerns:

### Workspace 1: Micro-ROS Firmware (THIS DIRECTORY)
```
/home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl/
├── ESP32 firmware code (C++)
├── motor_control.h (hardware drivers)
├── Sensor integration (LIDAR, IMU)
└── Publisher setup for /scan, /imu/data
```

**Purpose:** Low-level device drivers and firmware
**Build:** PlatformIO `platformio run --target upload`
**Output:** Binary uploaded to ESP32

---

### Workspace 2: ROS2 Application (ros2_ece_ws)
```
/home/matheo/ros_test/ros2_ece_ws/
├── Motor test scripts (test_motor_control.py, etc.)
├── SLAM nodes (simple_slam.py)
├── Odometry nodes (motor_odom_node.py)
├── Motor testing guide (MOTOR_TESTING_GUIDE.md)
└── Launch files and configurations
```

**Purpose:** High-level ROS2 applications and tests
**Build:** Colcon `colcon build`
**Output:** ROS2 nodes running on PC

---

## Communication Flow

```
┌─────────────┐                    ┌──────────────────┐
│   ESP32     │ ◄─── WiFi UDP ──► │  micro-ROS Agent │
│  Firmware   │      (Port 8888)   │      (on PC)      │
│             │                    │                   │
│ main.cpp    │                    │  Bridges to ROS2  │
└─────────────┘                    └──────────────────┘
       ↓                                     ↓
   Publishes:                          ROS2 Network:
   - /scan                             - /scan
   - /imu/data                         - /imu/data
   - /odom                             - /odom
                                       - /map
   Subscribes:                         - /cmd_vel ◄─────┐
   - /cmd_vel                                           │
       ↑                                                 │
       └────────────────────────────────────────────────┘
                                    ROS2 Test Scripts:
                                    - test_motor_control.py
                                    - test_motor_ramp.py
                                    - interactive_motor_control.py
```

---

## Getting Started

### 1. Compile Firmware
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
platformio run --target upload
```

### 2. Start micro-ROS Agent
```bash
cd /home/matheo/microros_ece_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 3. Run ROS2 Applications
```bash
cd /home/matheo/ros_test/ros2_ece_ws
source install/setup.bash

# Run motor tests
python3 test_motor_control.py

# Or run SLAM
ros2 run simple_slam simple_slam
```

---

## Key Files

| File | Purpose | Location |
|------|---------|----------|
| `main.cpp` | Firmware entry point | `src/` |
| `motor_control.h` | Motor driver library | Root |
| `platformio.ini` | Build configuration | Root |
| Motor test scripts | ROS2 motor testing | `../../../ros_test/ros2_ece_ws/` |

---

## Important Note

**Keep firmware and application code SEPARATE!**

- Test scripts and ROS2 nodes belong in **ros2_ece_ws**
- Motor control tests are published to `/cmd_vel` topic
- Firmware just needs to receive these commands and apply them
- Don't mix ROS2 app code with micro-ROS firmware code

This separation:
- ✅ Keeps firmware lean and focused
- ✅ Makes testing easier (can swap test scripts)
- ✅ Allows multiple PC-side applications to control the robot
- ✅ Simplifies debugging (firmware vs. application issues)

---

## References

- **Micro-ROS Setup:** See README.md in this directory
- **Motor Testing:** See MOTOR_TESTING_GUIDE.md in ros2_ece_ws
- **SLAM Integration:** See SLAM_README.md in this directory
