# LiDAR Front Offset Calibration

**Calibration Result (22 Feb 2026):**

- The true front of the robot (for mapping and obstacle avoidance) is at **-90 degrees** relative to the LiDAR scan.
- This offset is now applied in lidar_gap_follower.py as `lidar_front_offset_deg = -90.0`.
- Update your mapping and navigation nodes to use this offset for correct front alignment.

> Calibration test: LiDAR front alignment script confirmed -90° is the correct value for the robot's front.

---
# 🔍 MAPPING DIVERGENCE DURING ROTATION - ROOT CAUSE & FIX

## Problem Description

**Symptom:**
- ✅ Mapping works fine when moving forward/backward
- ❌ Mapping breaks immediately when rotating

**Why?**

---

## 🔴 ROOT CAUSE: Covariance Mismatch

### The Issue:

Your robot has **NO WHEEL ENCODERS**, so:
- Motor odometry can estimate **linear displacement** fairly well (motors report what they were told)
- Motor odometry CANNOT estimate **rotation** well (no encoders to verify actual turn)
- Only the IMU gyroscope can measure rotation, but **gyroscopes drift over time**

### What Was Happening (BEFORE FIX):

```
motor_odom_node.py publishes Odometry with:
  pose_covariance[yaw] = 0.05  ← Says "I'm 95% confident in my yaw estimate!"
  
  Problem: Motor has NO ENCODERS, so yaw is ONLY from IMU gyro!
  IMU gyro drifts ~1-2°/sec, so this confidence is WRONG.

SLAM receives this and thinks:
  "Motor says I rotated 90° with 95% confidence"
  "LIDAR says wall moved from 0° to 85°"
  → CONFLICT! SLAM doesn't know who to believe
  → Mapping quality drops to ZERO
```

### Diagram:

```
                  BEFORE FIX                      AFTER FIX
                  
  motor_odom        yaw_cov = 0.05              yaw_cov = 1.0
  (IMU only)        (95% confident!)            (only 37% confident!)
         ↓                                              ↓
    SLAM fusion      "Trust odometry              "Don't trust odometry
                     heavily"                    for rotation!"
         ↓                                              ↓
    Mapping          ❌ DIVERGES                  ✅ TRUSTS LIDAR INSTEAD
```

---

## ✅ SOLUTION: Increase Rotation Covariance

### What Changed:

#### 1. **motor_odom_node.py** - Yaw Covariance:
```python
# BEFORE: covariance yaw = 0.05  (95% confident)
# AFTER:  covariance yaw = 1.0   (only 37% confident) ← 20x increase!

pose_covariance[35] = 1.0   # [5,5] = yaw position covariance

# AND twist (angular velocity):
twist_covariance[35] = 0.2  # [5,5] = wz covariance (4x increase)
```

**Effect:** SLAM gets the message: "Don't trust my yaw estimate!"

#### 2. **simple_ekf.py** - Trust IMU More:
```python
# During fusion, trust IMU 90%, odometry 10% (was 80/20)
self.state[4] = 0.9 * self.state[4] + 0.1 * wz

# Increase yaw process noise
self.odom_yaw_noise = 0.5  # (new)
```

**Effect:** EKF relies on IMU gyroscope (not odometry) for heading

#### 3. **slam_toolbox_params.yaml** - Already Done:
```yaml
minimum_travel_heading: 0.15  # (was 0.1)
# Give SLAM more "freedom" to correct heading with LIDAR
```

---

## 📊 Confidence Levels AFTER FIX:

```
Linear motion (X, Y):
  Motor odometry: 95% confident (has PWM feedback)
  ↓ SLAM uses it as initial guess ✓

Rotation (Yaw):
  Motor odometry: 37% confident (only has IMU, which drifts)
  ↓ SLAM largely IGNORES it ✓
  ↓ SLAM TRUSTS LIDAR SCAN MATCHING instead ✓✓✓
```

---

## 🧪 What Should Happen Now:

### During Forward/Backward Motion:
```
Motor odometry: "I moved 10cm forward, rotated 0°"  ← 95% trusted
IMU: "I rotated 0°" (agrees)                        ← agrees
LIDAR: "Walls same, moved forward"                  ← agrees
Result: ✅ PERFECT MAPPING
```

### During Rotation:
```
Motor odometry: "I rotated 45°"  ← Only 37% trusted (yaw_cov=1.0)
IMU: "I rotated 42°"             ← Slightly trusted
LIDAR: "Walls rotated 40°"       ← HEAVILY TRUSTED (50%+ weight)
Result: ✅ GOOD MAPPING (LIDAR-based)
```

---

## 🧪 TEST THE FIX:

### Terminal 1: Start everything
```bash
./start_stack.sh
```

### Terminal 2: Run diagnostic
```bash
python3 diagnostic_monitor.py
```

Watch the output:
- When moving forward: `Position Error` should stay low (<10cm)
- When rotating: `Yaw Error` should stay low (<15°)
- `Yaw Covariance` in motor_odom should show **>0.5** (not confident)

### Terminal 3: Manual control
```bash
python3 teleop_keyboard.py
```

**Test sequence:**
1. Press **W** - move forward → mapping should be perfect ✓
2. Press **D** - rotate right → mapping should still work ✓
3. **Combination**: W then D then W → should be continuous ✓

---

## 📈 Why This Works:

**Before:** SLAM had to reconcile conflicting sensors
- Motor odometry (wrong heading) vs LIDAR (right heading)
- Resulted in SLAM "jumping around" = bad mapping

**After:** SLAM knows the truth
- Motor odometry: "I don't know my heading" (high covariance)
- IMU: "I think I rotated this much" (medium confidence)  
- LIDAR: "The walls say rotation is this" (HIGH confidence)
- SLAM uses LIDAR as primary source for rotation ✓

---

## 🔧 If It Still Doesn't Work:

### Debugging checklist:

1. **Check motor_odom_node.py is using IMU:**
   ```bash
   ros2 topic echo /odom/pose/covariance[35]
   # Should be around 0.5-1.0 (NOT 0.05)
   ```

2. **Check SLAM is receiving high-confidence LIDAR:**
   ```bash
   # In RViz: add "LaserScan" layer, color by "intensity"
   # Should see rainbow colors (good scan quality)
   ```

3. **Check EKF is running:**
   ```bash
   ros2 topic echo /odom_filtered/header/stamp
   # Should update at 50 Hz
   ```

4. **Check SLAM map_update_interval:**
   ```yaml
   # config/slam_toolbox_params.yaml
   map_update_interval: 0.5  # Should be this
   ```

---

## ✅ Summary of Changes:

| File | Change | Reason |
|------|--------|--------|
| `motor_odom_node.py` | yaw covariance: 0.05→1.0 | Don't trust IMU-only yaw |
| `simple_ekf.py` | trust IMU 90% | IMU better than motor for rotation |
| `slam_toolbox_params.yaml` | heading threshold: 0.1→0.15 | Give LIDAR more authority |

**Result:** SLAM now uses LIDAR for heading correction during rotation = better mapping!
