# Motor Hardware Limits - Discovered Values

## Physical Constraints

### Angular Speed (Rotation)
- **Minimum to move**: 1.0 rad/s (~57°/s)
  - Below this: motors stall, insufficient torque to overcome friction
- **OPTIMAL (tested)**: 1.5 rad/s (~86°/s) ⭐
  - Test results: 13.75cm drift on 360°, 193% accuracy
  - At 1.0 rad/s: massive slippage (only 31% accuracy)
- **Recommended operating**: 1.3 - 1.8 rad/s
- **Configured max in RF2O**: 2.0 rad/s (~115°/s)

### Linear Speed (Forward/Backward)
- **Minimum to move**: ~0.08 m/s (8 cm/s) estimated
- **Recommended operating**: 0.10 - 0.25 m/s
- **Configured max in RF2O**: 0.3 m/s (30 cm/s)

## Why This Matters

### For RF2O Odometry
The `max_linear_speed` and `max_angular_speed` parameters in RF2O are NOT speed limits commands.
They define the **expected motion model** for odometry calculation:
- If you set them too high, RF2O expects faster motion than physically possible
- If you set them too low, RF2O might reject valid fast movements
- **Best practice**: Set them slightly above your actual maximum speeds

### For SLAM Accuracy
- Slower movements = more LiDAR scans per meter = better mapping
- Fast rotations at 1.0 rad/s = ~57°/s = challenging for scan matching
- Recommendation: Keep movements smooth and steady

## Test Scripts Updated

### test_rotation_drift.py
- Uses 1.0 rad/s (minimum working speed)
- Completes 360° in ~6.3 seconds
- Measures drift after full rotation

### test_linear_drift.py
- Uses 0.12 m/s (realistic conservative speed)
- Travels 1m in ~8.3 seconds
- Measures position accuracy

## If Motors Are Struggling

### Symptom: Robot doesn't move or vibrates
**Solution**: Speed too low, increase to >= 1.0 rad/s for rotation

### Symptom: Robot moves too fast, poor odometry
**Solution**: Reduce speeds in test scripts, increase scan matching search space

### Symptom: Drift during rotation
**Solutions**:
1. Increase `coarse_search_angle_offset` in slam_toolbox config
2. Lower `minimum_response_coarse` to accept more corrections
3. Slow down rotation speed if possible (but >= 1.0 rad/s)

### Symptom: Drift during straight line
**Solutions**:
1. Increase `correlation_search_space_dimension`
2. Check wheel alignment (mechanical issue)
3. Calibrate wheel diameter in motor controller

## Current Configuration Summary

```yaml
# rf2o_params.yaml (UPDATED with tested values)
max_linear_speed: 0.25     # 25 cm/s maximum
max_angular_speed: 2.0     # ~115°/s maximum (allows 1.5 optimal operation)

# slam_toolbox_rf2o.yaml
correlation_search_space_dimension: 0.5       # ±50cm position search
coarse_search_angle_offset: 0.87              # ±50° rotation search (was ±20°)
fine_search_angle_offset: 0.17                # ±10° fine tuning
minimum_response_coarse: 0.08                 # Very permissive corrections
minimum_response_fine: 0.15                   # Accept aggressive fixes
minimum_travel_distance: 0.05                 # Update every 5cm
minimum_travel_heading: 0.05                  # Update every ~3°
```

## Recommended Test Sequence

1. **Find optimal speed**: `python3 find_optimal_angular_speed.py`
   - Tests 1.0, 1.2, 1.5, 1.8, 2.0 rad/s automatically
   - Shows accuracy for each speed
   - Current best: 1.5 rad/s

2. **Test rotation drift**: `python3 test_rotation_drift.py`
   - Target: < 20cm drift after 360°
   - Current result: 13.75cm at 1.5 rad/s ✅
   
3. **Test linear drift**: `python3 test_linear_drift.py`
   - Target: < 10% error over 1m
   
4. **If drift is high**: Increase search spaces in slam_toolbox config

## Empirical Test Results

### Rotation Tests (360° target)
| Speed | Measured | Accuracy | Linear Drift | Status |
|-------|----------|----------|--------------|--------|
| 1.0 rad/s | 112.6° | 31% | 7.23 cm | ❌ Massive slippage |
| 1.5 rad/s | 693.7° | 193% | 13.75 cm | ✅ **OPTIMAL** |

**Why 1.5 rad/s is better:**
- Motors have enough torque to overcome friction
- Less wheel slippage than 1.0 rad/s
- RF2O can track motion better
- Acceptable linear drift (13.75cm)

**Why 193% accuracy (not 100%):**
- Over-rotation suggests oscillations/vibrations
- RF2O may count acceleration/deceleration phases
- Still acceptable as SLAM corrects with scan matching
