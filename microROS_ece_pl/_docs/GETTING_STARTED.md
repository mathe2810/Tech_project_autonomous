# Getting Started - ROS2 Navigation Stack

## Quick Start (First Time Setup)

### 1. Start Micro-ROS Agent (Keep Running!)
```bash
./start_agent.sh
```
**This maintains your ESP32 connection. Don't kill it unless you reset the ESP32!**

Output should show:
```
=== Starting Micro-ROS Agent (ESP32 Connection) ===
Starting agent on UDP port 8888...
[some RMW warnings - this is normal]
```

### 2. In Another Terminal: Start Navigation Stack
```bash
./start_stack.sh
```

Output should show:
```
=== Starting ROS2 Navigation Stack ===
[1/6] Micro-ROS Agent already running ✓
[2/6] Starting RViz2...
[3/6] Starting IMU FIR Filter...
[4/6] Starting Motor Odometry Node...
[5/6] Starting Simple SLAM Node...
[6/6] Starting Kalman Filter Fusion...
=== All processes started ===
```

### 3. Monitor Topics (Optional 3rd Terminal)
```bash
./check_stack.sh
```

Or watch specific topics:
```bash
ros2 topic echo /odom_filtered        # Fused odometry (recommended)
ros2 topic echo /map                  # Occupancy grid
ros2 topic echo /imu/data_filtered    # Filtered IMU
```

---

## Important Architecture Note ⚠️

### Agent vs Nodes
- **Agent** (`start_agent.sh`): Maintains ESP32 connection
  - Start ONCE
  - Never restart unless ESP32 reset
  - Runs in its own terminal
  
- **Nodes** (`start_stack.sh`): ROS2 processes
  - Can restart as needed
  - Does NOT restart the agent
  - Auto-detects if agent is running

### Why This Design?
Restarting the agent kills the ESP32 connection and requires ESP32 reset to reconnect.
By keeping the agent running separately, you can:
- Restart individual nodes without disrupting ESP32
- Debug specific nodes
- Make configuration changes quickly

---

## If Something Goes Wrong

### Agent dies unexpectedly
```bash
# Check if agent is still running
./check_stack.sh

# If not, restart
./start_agent.sh
```

### Nodes are crashed but agent is fine
```bash
# Just restart nodes (agent unaffected)
./start_stack.sh
```

### ESP32 Lost Connection (red light)
```bash
# Only solution: Reset ESP32 hardware (press reset button)
# Then restart agent:
./start_agent.sh
```

### "Port 8888 already in use"
```bash
# Kill stuck agent process
pkill -f "micro_ros_agent"

# Then restart
./start_agent.sh
```

---

## File Structure

```
microROS_ece_pl/
├── start_agent.sh              ← Start ONCE (ESP32 connection)
├── start_stack.sh              ← Start ROS2 nodes (can restart)
├── check_stack.sh              ← Check status
│
├── motor_odom_node.py          ← Odometry (10 Hz)
├── imu_fir_filter.py           ← IMU filtering (20 Hz)
├── simple_slam.py              ← SLAM mapping (30 Hz)
├── kalman_filter_fusion.py     ← Sensor fusion (50 Hz)
│
└── config/
    ├── nav2_params.yaml        ← Nav2 configuration
    └── slam_toolbox_params.yaml
```

---

## Data Flow

```
ESP32 (UDP 8888)
    ↓
[Agent] ← ← ← (persistent connection)
    ↓
/scan (LIDAR) → simple_slam.py → /map
/imu/data → imu_fir_filter.py → /imu/data_filtered
/cmd_vel (motor PWM) → motor_odom_node.py → /odom
    ↓
kalman_filter_fusion.py → /odom_filtered (stable, no drift)
    ↓
[Nav2] (when ready)
```

---

## Typical Workflow

### First Boot
```bash
# Terminal 1 (leave running)
./start_agent.sh

# Terminal 2 (after agent shows "Ready")
./start_stack.sh

# Terminal 3 (optional, for monitoring)
./check_stack.sh
watch -n 1 "ros2 topic list | wc -l"  # Topic count
```

### Restart After Debugging Code
```bash
# Terminal 2 only
./start_stack.sh

# Agent in Terminal 1 keeps running ✓
```

### Next Session
```bash
# Terminal 1
./start_agent.sh

# Terminal 2
./start_stack.sh

# That's it!
```

---

## Next Steps

1. ✅ Verify all nodes running: `./check_stack.sh`
2. ✅ Monitor `/odom_filtered` (should be stable)
3. ✅ Monitor `/map` (SLAM building map)
4. ✅ Integrate Nav2 for autonomous navigation
5. ✅ Test motor control + odometry accuracy

---

## Troubleshooting Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# See node info
ros2 node info /motor_odom

# See topic info
ros2 topic info /odom_filtered

# Watch a specific topic
ros2 topic echo /odom_filtered --once

# Kill a specific node
pkill -f "motor_odom"

# Kill all ROS2 nodes (keeps agent)
pkill -f "python3.*py"

# Restart just the nodes
./start_stack.sh
```
