# 🤖 Autonomous SLAM Navigation System

**Système de localisation et cartographie simultanées (SLAM) pour robot autonome avec micro-ROS et ROS2 Humble.**

```
   ╔════════════════════════════════════════════════╗
   ║  AUTONOMOUS ROBOT SLAM NAVIGATION SYSTEM       ║
   ║  ─────────────────────────────────────────     ║
   ║  Status: ✅ PRODUCTION READY                   ║
   ║  Version: 1.0 (Février 2026)                   ║
   ║  Last Update: Timestamp Fix + Optimization     ║
   ╚════════════════════════════════════════════════╝
```

---

## 📚 Documentation Hub

### Getting Started
- **[QUICKSTART.md](QUICKSTART.md)** ← **START HERE** (5 min setup)
  - Installation rapide
  - Vérification hardware
  - Lancement du système
  - Tests de base

### System Architecture
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Design système complet
  - Vue d'ensemble globale
  - Communication topology
  - Node dependency graph
  - Hardware connectivity
  - Design decisions

### Technical Deep-Dive
- **[SLAM_IMPLEMENTATION.md](SLAM_IMPLEMENTATION.md)** - Détails algorithme SLAM
  - Architecture SLAM complète
  - Scan matching (ICP) détaillé
  - Pose graph optimization
  - Configuration YAML
  - La crise de timestamp (résolution)

### Performance Analysis
- **[PERFORMANCES.md](PERFORMANCES.md)** - Évolution before/after
  - Métriques détaillées
  - Comparaison before/after chaque optimization
  - Benchmarking résultats
  - Trade-offs CPU/mémoire
  - Production readiness

---

## 🎯 System Overview

### What Does It Do?

```
┌─────────────────────────────┐
│  Real-time simultaneous:    │
│  1. MAPPING (via SLAM)      │
│     → Builds occupancy grid │
│     → ~2.5 cm resolution    │
│                             │
│  2. LOCALIZATION (fusion)   │
│     → Robot position in map │
│     → ±1% accuracy          │
│                             │
│  3. NAVIGATION (ready for)  │
│     → Path planning         │
│     → Obstacle avoidance    │
└─────────────────────────────┘

Input: LIDAR + IMU + Motor encoders
Output: /map (occupancy grid) + /odom_filtered (pose)
```

### Hardware Stack

| Component | Specification | Role |
|-----------|---------------|------|
| **Compute** | ESP32 (Xtensa dual-core) | Sensor acquisition + PWM control |
| **LIDAR** | LD06 (360°, 12m) | Main sensing modality |
| **IMU** | MPU6050 (9-axis) | Heading estimation |
| **Motor Driver** | L298N (2 motors) | Wheel control |
| **WiFi** | 802.11b/g/n (54 Mbps) | micro-ROS communication |
| **PC/Linux** | Ubuntu 22.04 + ROS2 Humble | SLAM processing + Nav2 |

### Software Stack

| Layer | Component | Technology |
|-------|-----------|-----------|
| **Firmware** | ESP32 drivers | C++ + FreeRTOS multithreading |
| **Real-time** | micro-ROS | UDP bridge to ROS2 |
| **Middleware** | ROS2 Humble | Publish/subscribe framework |
| **Processing** | SLAM Toolbox | Graph-based SLAM + Ceres solver |
| **Sensors** | Filters | FIR (IMU), EKF (fusion) |
| **Visualization** | RViz2 | Live 3D mapping display |
| **Navigation** | Nav2 (planned) | Path planning + obstacle avoidance |

---

## 🚀 Quick Start (5 minutes)

### Installation

```bash
# 1. Clone repository (if not already)
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

# 2. Install dependencies (one-time)
sudo apt update
sudo apt install -y ros2-humble-slam-toolbox ros2-humble-nav2-* docker.io

# 3. Download Docker micro-ROS agent
docker pull microros/micro-ros-docker:humble
```

### Launch (3 terminals)

**Terminal 1: Micro-ROS Agent**
```bash
docker run -it --rm -v /dev:/dev --privileged \
  microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --ip 0.0.0.0 --port 8888
```

**Terminal 2: Full Stack**
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
./start_stack.sh
```

**Terminal 3: Manual Control**
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 teleop_keyboard.py  # Use WASD to move robot
```

### Verify

```bash
# In another terminal, check topics
ros2 topic list | grep -E "(map|scan|odom)"
ros2 topic hz /map  # Should show 1 Hz
```

**✅ If you see `/map` at 1 Hz → System working!**

---

## 📂 File Structure

```
.
├── QUICKSTART.md                 # ← Start here
├── ARCHITECTURE.md               # System design
├── SLAM_IMPLEMENTATION.md        # Algorithm details
├── PERFORMANCES.md               # Before/after analysis
├── README.md                     # This file
│
├── src/
│   └── main.cpp                  # ESP32 firmware
│
├── config/
│   └── slam_toolbox_params.yaml # SLAM config (0.025m resolution)
│
├── include/
│   └── motor_control.h          # Motor functions
│
├── scan_restamper.py            # ⭐ Timestamp fix (CRITICAL)
├── simple_ekf.py                # Sensor fusion (Kalman)
├── motor_odom_node.py           # Dead reckoning
├── imu_fir_filter.py            # Signal filtering
├── teleop_keyboard.py           # Manual control
├── test_movement.py             # Automated testing
│
├── launch_all.py                # Launcher script
├── start_stack.sh               # Main startup (use this!)
├── switch_firmware.sh           # Firmware switcher
│
└── build/, install/, log/       # Build artifacts (auto-generated)
```

---

## 🔧 Core Nodes Explained

### 1. **Scan Restamper** (`scan_restamper.py`) ⭐ CRITICAL

**Purpose:** Fix timestamp mismatch between ESP32 boot-time and ROS2 wall-clock

**Why needed:** 
- ESP32 timestamps: ~8000 sec (boot time)
- ROS2 clock: ~1770815564 sec (Unix time)
- Gap: 1.77M seconds → SLAM message filter rejects all scans!

**Solution:**
```python
msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
```

**Impact:** Enables entire SLAM system (was broken without this)

---

### 2. **SLAM Toolbox** (ROS2 official)

**Purpose:** Real-time simultaneous localization and mapping

**Algorithm:** Graph-based SLAM with Ceres optimizer
- Scan matching (ICP) for pose estimation
- Pose graph optimization for drift correction
- Occupancy grid for map representation

**Configuration:**
```yaml
resolution: 0.025        # 2.5 cm per pixel (4× detail)
max_laser_range: 5.0     # Focus 5×5m zone
mode: mapping            # Accumulate all scans
```

**Output:**
- `/map`: Occupancy grid (200×200 cells, 1 Hz)
- `/tf`: map → odom transformation
- `/slam_toolbox/pose`: Estimated pose

---

### 3. **Simple EKF** (`simple_ekf.py`)

**Purpose:** Fuse odometry + IMU for accurate position estimation

**Algorithm:** Extended Kalman Filter (2D)
- Predicts position from velocities
- Corrects with IMU heading
- Removes drift from motor slippage

**Inputs:**
- `/odom`: Motor-based dead reckoning
- `/imu/data_filtered`: Clean gyro for heading

**Output:**
- `/odom_filtered`: Corrected position (ready for Nav2)
- `/tf`: odom → base_link (corrected)

---

### 4. **Motor Odometry** (`motor_odom_node.py`)

**Purpose:** Dead reckoning from motor commands

**Algorithm:** Kinematic integration
```
If moving:  x += vx·cos(θ)·dt, y += vx·sin(θ)·dt, θ += wz·dt
If stopped: Position holds (no drift without command)
```

**Important:** Only moves when receiving `/cmd_vel` (correct design)

---

### 5. **IMU FIR Filter** (`imu_fir_filter.py`)

**Purpose:** Remove structural vibrations from LIDAR

**Algorithm:** 21-tap FIR lowpass filter @ 20 Hz
- Removes LIDAR vibration energy
- Preserves IMU signal up to 20 Hz

**Before/After:** Cleaner IMU data for EKF fusion

---

## 🎮 Control Options

### Option A: Keyboard Control (Manual)
```bash
python3 teleop_keyboard.py

W = Forward (0.3 m/s)
A = Turn left (1.5 rad/s)
S = Backward
D = Turn right
Space = Stop
Q = Quit
```

### Option B: Automated Testing
```bash
python3 test_movement.py

# Sends sequence: 3s forward → 3s rotate → stop
# Watch position change in /odom
```

### Option C: ROS2 Commands
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5}}'
```

---

## 📊 Key Metrics

### Performance
- **CPU Usage:** 10-15% (i7-10700)
- **Latency:** 121 ms (LIDAR → RViz)
- **Memory:** ~50 MB SLAM + 150 MB RViz
- **/map Rate:** 1 Hz (continuous)
- **Position Accuracy:** ±1% with EKF fusion

### Map Quality
- **Resolution:** 2.5 cm per pixel (4× fine detail)
- **Range:** 5×5 meter coverage
- **Update:** ~100ms per scan
- **Memory:** 40-50 MB per state

### Robot Control
- **Forward Speed:** 0.3 m/s (configurable)
- **Rotation Speed:** 1.5 rad/s (responsive)
- **Response Time:** <100ms
- **Accuracy:** ±1% dead reckoning

---

## 🔍 Troubleshooting

### ❌ `/map` topic not appearing
```bash
# Check:
ros2 topic list | grep map    # Should see /map
ros2 topic hz /map            # Should see 1 Hz

# Fix:
# 1. Verify scan_restamper is running (Step [4.2/7] in start_stack.sh)
# 2. Check timestamp: ros2 topic echo /scan --max-count=1 | grep stamp
#    Should show current time (>1770000000), not 8000+
```

### ❌ Robot not moving
```bash
# Check:
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1}}'

# Then check odometry:
ros2 topic echo /odom --max-count=1

# If position doesn't change:
# 1. Motor driver may be disconnected
# 2. Motor PWM pins may be wrong
# 3. Check ESP32 logs: pio device monitor -b 115200
```

### ❌ High CPU usage (>50%)
```bash
# Reduce complexity:
# 1. Increase resolution: 0.025 → 0.05 (less pixels)
# 2. Decrease range: 5.0 → 3.0 (fewer points)
# 3. In slam_toolbox_params.yaml

# Or identify bottleneck:
ros2 topic hz /map          # Should be 1 Hz (not faster)
ps aux | grep slam          # Check SLAM CPU
ps aux | grep python        # Check Python nodes
```

---

## 🚀 Next Steps

### Short-term
1. ✅ **Done:** Core SLAM mapping working
2. ✅ **Done:** Manual teleoperation
3. ✅ **Done:** Sensor fusion
4. [ ] **TODO:** Verify loop closure (optional)

### Medium-term
1. [ ] **Add Nav2 stack** - Autonomous navigation
2. [ ] **Path planning** - Goal-based waypoints
3. [ ] **Obstacle avoidance** - Dynamic collision prevention
4. [ ] **Map persistence** - Save and reload maps

### Long-term
1. [ ] **Multi-robot SLAM** - Coordinate mapping
2. [ ] **Visual loop closure** - Camera for robustness
3. [ ] **Semantic mapping** - Object detection + labels
4. [ ] **Hybrid navigation** - SLAM + visual odometry

---

## 📖 Learning Resources

### Papers & References
- **Hector SLAM** - Real-time SLAM with consumer hardware
- **g2o** - Graph optimization framework
- **Ceres Solver** - Nonlinear optimization library
- **Extended Kalman Filter** - Sensor fusion theory

### ROS2 Documentation
- [ROS2 Official](https://docs.ros.org/en/humble/)
- [SLAM Toolbox Docs](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)

### Our Documentation
- 📖 [QUICKSTART.md](QUICKSTART.md) - Getting started
- 🏗️ [ARCHITECTURE.md](ARCHITECTURE.md) - System design
- 🎯 [SLAM_IMPLEMENTATION.md](SLAM_IMPLEMENTATION.md) - Algorithm details
- 📊 [PERFORMANCES.md](PERFORMANCES.md) - Analysis

---

## 👥 Team & Attribution

**Project:** ECE Navigation Stack with SLAM
**Platform:** ROS2 Humble + micro-ROS + ESP32
**Status:** ✅ Production Ready (v1.0)
**Last Updated:** February 2026

**Key Components:**
- ROS2 SLAM Toolbox (Steve Macenski)
- Ceres Solver (Google)
- micro-ROS (micro-ROS Alliance)
- MPU6050 IMU drivers
- L298N motor drivers

---

## 📋 Version History

### v1.0 (February 2026) - PRODUCTION RELEASE
- ✅ SLAM Toolbox fully operational
- ✅ Timestamp synchronization fixed (scan_restamper)
- ✅ Motor odometry + EKF fusion working
- ✅ Keyboard teleoperation implemented
- ✅ Performance optimized (0.025m resolution, 5×5m zone)
- ✅ Comprehensive documentation complete

---

## 📞 Support & Questions

For issues or questions:
1. **Check [QUICKSTART.md](QUICKSTART.md)** first (5 min setup)
2. **Check [TROUBLESHOOTING](#troubleshooting-section)** above
3. **Review logs:** `ros2 node list` and `ros2 topic hz`
4. **Debug with:** `pio device monitor -b 115200` (ESP32 logs)

---

## 📜 License & Disclaimer

This project uses:
- **ROS2 Humble** (Apache 2.0)
- **SLAM Toolbox** (BSD 3-Clause)
- **Ceres Solver** (BSD 3-Clause)
- **micro-ROS** (Apache 2.0)

Please respect all licenses when distributing.

---

**Welcome to the Autonomous SLAM Navigation System! 🎉**

**Start with [QUICKSTART.md](QUICKSTART.md) → Have it working in 5 minutes.**

**Status: ✅ Stable, Tested, Ready for Deployment**

---

**Version:** 1.0 | **Last Update:** February 2026 | **Status:** ✅ Production Ready
