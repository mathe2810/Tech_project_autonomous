# Testing Protocol - Before Nav2 Integration

## Step-by-Step Validation

### Phase 1: Stack Startup (5 min)

#### 1.1 Start Agent
```bash
./start_agent.sh
```
**Expected output:**
```
running.
...
session established | client_key: 0x..., address: 172.20.10.3:...
```
✅ **PASS** if you see "session established"

---

#### 1.2 Start ROS2 Nodes (in different terminal)
```bash
./start_stack.sh
```
**Expected output:**
```
=== Starting ROS2 Navigation Stack ===
[1/6] Micro-ROS Agent already running ✓
[2/6] Starting RViz2...
[3/6] Starting IMU FIR Filter...
[4/6] Starting Motor Odometry Node...
[5/6] Starting Simple SLAM Node...
[6/6] Starting Kalman Filter Fusion...
```
✅ **PASS** if all 5 processes start

---

### Phase 2: System Validation (10 min)

#### 2.1 Run Validation Script
```bash
python3 validate_stack.py
```
**Expected output:**
```
🔌 [1/5] CHECKING MICRO-ROS AGENT
    ✅ Agent running

🤖 [2/5] CHECKING ROS2 NODES
    ✅ IMU Filter
    ✅ Motor Odometry
    ✅ SLAM
    ✅ Kalman Fusion

📡 [3/5] CHECKING DATA FLOW
    ✅ Raw IMU                     /imu/data
    ✅ Filtered IMU                /imu/data_filtered
    ✅ LIDAR                       /scan
    ✅ Raw Odometry                /odom
    ✅ Fused Odometry (IMPORTANT)  /odom_filtered
    ✅ SLAM Map                    /map
    ✅ SLAM Pose                   /slam/pose
```
✅ **PASS** if all checks show ✅

---

#### 2.2 Run Diagnostics
```bash
python3 diagnose_stack.py
```
**Expected output example:**
```
ODOMETRY COMPARISON
  Raw (/odom):             X=+0.1234  Y=-0.0567
  Filtered (/odom_filtered): X=+0.1240  Y=-0.0562
  Difference:            ΔX=0.0006  ΔY=0.0005
  Status:                ✅ Fusion working (minimal drift)

IMU FILTERING VALIDATION
  Raw accel X:           +0.2345 m/s²
  Filtered accel X:      +0.1234 m/s²
  Filtering reduction:   47.4% ✅

SLAM STATUS
  SLAM Pose:             X=+0.5678  Y=-0.1234
  vs Fused Odom:         ΔX=0.0123  ΔY=0.0087
  Consistency:           ✅ SLAM and odom agree
  Status:                ✅ SLAM producing poses
```
✅ **PASS** if:
- Fusion difference < 0.01m (excellent) or < 0.1m (good)
- IMU filtering > 20%
- SLAM and odom ΔX,ΔY < 0.5m

---

### Phase 3: Real-time Monitoring (15-30 min)

#### 3.1 Monitor Main Output (Fused Odometry)
```bash
ros2 topic echo /odom_filtered
```
**Expected behavior:**
- Position X, Y increasing smoothly
- No sudden jumps
- Orientation (quaternion) changing smoothly
- Frequency: ~50 Hz

**Checklist:**
- [ ] Values update in real-time
- [ ] Smooth transitions (no jerky movements)
- [ ] Position makes sense with robot motion
- [ ] Timestamps incrementing

✅ **PASS** if position updates smoothly without jumps

---

#### 3.2 Monitor SLAM Map
```bash
# In another terminal, in a loop:
while true; do
  ros2 topic echo /map --once
  sleep 2
done
```
**Expected behavior:**
- `info.width` and `info.height` > 0
- `data` array populated with occupancy values
- New obstacles appearing as robot moves

✅ **PASS** if map dimensions > 0 and contains data

---

#### 3.3 Monitor IMU Filtering
```bash
# Watch raw vs filtered in parallel
# Terminal 1:
watch -n 0.1 "ros2 topic echo /imu/data --once | grep linear_acceleration -A 3"

# Terminal 2:
watch -n 0.1 "ros2 topic echo /imu/data_filtered --once | grep linear_acceleration -A 3"
```
**Expected behavior:**
- Raw IMU: oscillating values (25-50 Hz noise)
- Filtered IMU: smooth values
- Filtering should reduce amplitude by 30-50%

✅ **PASS** if filtered values are visibly smoother

---

### Phase 4: Optional Advanced Testing

#### 4.1 Transform Tree
```bash
ros2 tf2_tree.py
```
**Expected output:**
```
/map
 /odom
  /base_link
```
✅ **PASS** if hierarchy is correct

---

#### 4.2 Topic Bandwidth
```bash
ros2 topic hz /odom_filtered
ros2 topic hz /imu/data_filtered
ros2 topic hz /scan
```
**Expected frequencies:**
- `/odom_filtered`: ~50 Hz
- `/imu/data_filtered`: ~100 Hz (after filtering at 20 Hz)
- `/scan`: ~10 Hz (LIDAR rotation)

---

#### 4.3 Data Size Check
```bash
ros2 topic bw /odom_filtered
```
**Expected:** < 1 KB/s (very low bandwidth, good for network)

---

### Phase 5: Manual Robot Test (Optional)

1. **Move robot forward slowly**
   ```bash
   # Watch odometry position change
   ros2 topic echo /odom_filtered | grep -A 3 position
   ```
   ✅ **PASS** if X increases (or Y depending on orientation)

2. **Rotate robot 360°**
   ```bash
   # Watch orientation (yaw) change
   ros2 topic echo /odom_filtered | grep orientation -A 4
   ```
   ✅ **PASS** if quaternion rotates and returns to start

3. **Check SLAM Map**
   ```bash
   # Should see obstacles/walls mapped
   ros2 topic echo /map --once | less
   ```
   ✅ **PASS** if map shows scanned environment

---

## Success Criteria Checklist

Before moving to Nav2, verify:

- [ ] Agent connected to ESP32
- [ ] All 4 ROS2 nodes running
- [ ] All 7 topics publishing data
- [ ] TF transforms present (map→odom→base_link)
- [ ] `/odom_filtered` publishing at 50 Hz
- [ ] Odometry position updates smoothly
- [ ] Fusion difference < 0.1m (no drift)
- [ ] IMU filtering reduces noise by > 20%
- [ ] SLAM producing consistent poses
- [ ] Map building correctly (obstacles visible)

**If all ✅ → Ready for Nav2 integration!**

---

## Troubleshooting

### No data on /odom_filtered
- Check: `ros2 topic hz /odom`
- If /odom missing: Motor controller not running
- Solution: Verify ESP32 firmware publishes /cmd_vel

### SLAM pose not moving
- Check: `ros2 topic hz /scan`
- If /scan has no data: LIDAR not publishing
- Solution: Verify LIDAR connection to ESP32

### Large fusion difference (> 0.5m)
- SLAM might be initializing (needs 5-10 seconds)
- Or odometry drifting fast (check IMU quality)
- Solution: Let system run for 30 seconds, then test again

### IMU filtering not working (> 70% difference)
- Check cutoff frequency (should be 20 Hz)
- Run: `ros2 node info /imu_fir_filter`
- Solution: May need to adjust `--cutoff` parameter

---

## Quick Restart

If something breaks:
```bash
# Kill nodes (keeps agent alive)
pkill -f "motor_odom|imu_fir|simple_slam|kalman_filter"

# Restart
./start_stack.sh

# Revalidate
python3 validate_stack.py
```

---

## Next: Nav2 Integration

Once all checks pass, you're ready to:
1. Create Nav2 launch file
2. Configure costmaps
3. Set up local/global planners
4. Test autonomous navigation

See: [NAV2_SETUP.md](NAV2_SETUP.md) (coming next)
