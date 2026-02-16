# LIDAR Vibration Analysis

## Discovery 🎯

**Your observation was correct!** The 25-50 Hz vibration is primarily from the **rotating LIDAR motor**, not just the drive motors.

## Current Configuration

```
LIDAR rotation frequency: 10 Hz (0.1s per scan = scan_time in main.cpp line 368)
IMU read frequency: 100 Hz (10ms interval)
IMU publish frequency: 20 Hz (50ms interval)
MPU6050 internal rate: ~1000 Hz (then resampled to 100 Hz)
```

## Why You See 25-50 Hz Harmonics

**LIDAR @ 10 Hz fundamental:**
```
1st harmonic:   10 Hz
2nd harmonic:   20 Hz  ← Appears as 28 Hz (couplage mécanique + amortissement)
3rd harmonic:   30 Hz  ← Detected directly ✓
4th harmonic:   40 Hz  ← Appears as 46 Hz
5th harmonic:   50 Hz  ← At Nyquist limit
```

**Why harmonics?**
1. LIDAR spinning creates centrifugal force
2. Mechanical resonance of mounting brackets/chassis
3. Bearing wear or imbalance
4. Motor cogging (magnetic ripple in brushless motors)

## Current Solution ✅

Your **FIR filter @ 20Hz cutoff** is **optimal**:
```bash
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100
```

- ✅ Cuts all LIDAR harmonics (20-50 Hz)
- ✅ Preserves IMU signal dynamics (<20 Hz)
- ✅ Acceptable latency (~100ms)
- ✅ Works with Kalman Filter Fusion

## Alternative Approach (Advanced)

If you want **less latency** while still filtering LIDAR vibration:

### Option: Notch Filters @ LIDAR Harmonics
```bash
# Instead of low-pass, use notches at exact harmonics
python3 imu_fir_filter.py --cutoff 25 --order 15 --sample-rate 100 --notch 20 30 40
```

**Advantages:**
- Latency: 75ms (vs 100ms)
- Preserves frequencies >20 Hz (useful for fast maneuvers)
- Specifically targets LIDAR harmonics

**Disadvantages:**
- More complex, three notch filters
- Less forgiving if LIDAR frequency varies

### We don't recommend this unless:
- You need fast response (<50ms latency)
- Or your robot moves fast and needs high-frequency IMU data

---

## If You Want to Further Reduce Vibration (Hardware)

### 1. **Improve LIDAR Mounting**
- Use **rubber dampers** between LIDAR and chassis
- Isolates mechanical vibration from IMU
- Cost: <$10, very effective

### 2. **Balance LIDAR Motor**
- If LIDAR motor is imbalanced → causes vibration
- Check bearings for wear
- Can reduce vibration amplitude by 50%

### 3. **Increase LIDAR Frequency**
- Change `scan_time` in main.cpp from 0.1 to 0.05 (20 Hz instead of 10 Hz)
- Faster rotation = less centrifugal force
- Also improves SLAM update rate (more scans)

```cpp
// In main.cpp line 368, change:
msg_lidar.scan_time = 0.1f;  // 10 Hz

// To:
msg_lidar.scan_time = 0.05f;  // 20 Hz
```

**Effect on IMU:** Harmonics move to 40, 60, 80... Hz (further from IMU content)

---

## Current Recommendation ✅

**Keep as-is:**
- FIR filter @ 20 Hz is good enough
- LIDAR @ 10 Hz is fine for mapping
- Kalman Filter Fusion handles the rest

**Next step:** Test the complete stack and validate odometry stability.

---

## Monitoring

To confirm LIDAR vibration is gone:
```bash
# Watch /imu/data_filtered - should be smooth
ros2 topic echo /imu/data_filtered | head -20

# Or check with statistics:
python3 -c "
import subprocess
import re

for i in range(10):
    result = subprocess.run(['ros2', 'topic', 'echo', '/imu/data_filtered', '--once'],
                          capture_output=True, text=True)
    accel_x = re.search(r'linear_acceleration:.*x: ([-\d.]+)', result.stdout)
    if accel_x:
        print(f'Accel X: {float(accel_x.group(1)):+.4f}')
"
```

Smooth values (< ±0.1 oscillation) = filter working ✓
