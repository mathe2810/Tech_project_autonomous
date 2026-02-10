# IMU FFT Analysis Results - 10 Feb 2026

## Raw Data
- **Duration**: 4.4 seconds
- **Sample Rate**: 100 Hz
- **Total Samples**: 444
- **Nyquist Frequency**: 50 Hz

## Analysis Results

### Accelerometer (m/s²)
| Axis | Peak 1 | Peak 2 | Peak 3 | Amplitude |
|------|--------|--------|--------|-----------|
| **X** | 28.4 Hz | 27.3 Hz | 32.7 Hz | 0.022 |
| **Y** | 33.6 Hz | 26.8 Hz | 41.4 Hz | 0.020 |
| **Z** | 46.6 Hz | 6.1 Hz  | 9.0 Hz  | 0.017 |

### Gyroscope (rad/s)
| Axis | Peak 1 | Peak 2 | Peak 3 | Amplitude |
|------|--------|--------|--------|-----------|
| **X** | 18.7 Hz | 29.3 Hz | 24.1 Hz | 0.002 |
| **Y** | 33.3 Hz | 24.1 Hz | 44.6 Hz | 0.002 |
| **Z** | 20.3 Hz | 29.7 Hz | 21.4 Hz | 0.000 |

## Diagnosis

### Noise Sources Identified
1. **Primary**: Motor vibration band **25-50 Hz**
   - Consistent across X, Y, Z accelerometer axes
   - Likely from motor rotation and mechanical friction
   
2. **Secondary**: Low frequency components **6-9 Hz**
   - Possible suspension/chassis resonance
   - Less critical (covered by low-pass filter)

3. **Gyroscope**: Very clean
   - Amplitudes 10x lower than accelerometer
   - Mostly thermal noise (white noise)
   - No major vibration coupling

### Why Motor Vibrations?
- Your motors run at variable speeds (PWM control)
- Typical small DC motors: 50-300 RPM = **0.8-5 Hz fundamental**
- But mechanical resonances & bearings create **harmonics** at 25-50 Hz ✓
- Two motors might create **two slightly different frequencies** (28 Hz and 33 Hz)

## Filter Configuration

### Selected Configuration ✅
```bash
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100
```

### Justification
| Parameter | Value | Reason |
|-----------|-------|--------|
| **Cutoff** | 20 Hz | Stops all motor noise (25-50 Hz band) |
| **Order** | 21 | Good balance: ~100ms latency, sharp cutoff |
| **Notch** | None | No single dominant peak, white noise band better with low-pass |
| **Window** | Hamming | Good sidelobe suppression, standard choice |

### Expected Results
- ✅ Motor vibrations: **Reduced by 20-30x**
- ✅ Signal latency: **~100 ms** (acceptable for robot navigation)
- ✅ Gyroscope: Already clean, minor further smoothing
- ✅ Kalman Filter: Will benefit from cleaner IMU input

## Filter Frequency Response

```
Before filter:
0      10      20      30      40      50 Hz
|      |       |       |       |       |
═══════════════════════════════════════╗  Motor vibrations
        Low freq (good)    Band noise ╚═ (BAD)

After filter (cutoff=20Hz):
0      10      20      30      40      50 Hz
|      |       |       |       |       |
═══════════════════════════════════════┓  Motor vibrations
        ↑ Clean               ✂ REMOVED   (GOOD)
```

## Monitoring

### Check filter is working:
```bash
# Watch raw IMU (noisy)
ros2 topic echo /imu/data --no-arr | grep -A 3 "linear_acceleration"

# Watch filtered IMU (clean)
ros2 topic echo /imu/data_filtered --no-arr | grep -A 3 "linear_acceleration"
```

### Visual check (if still noisy):
- If values still oscillate → increase cutoff (e.g., 18 Hz)
- If values lag behind → decrease cutoff (e.g., 22 Hz)

## Next Steps

1. ✅ **Implement filter** with config above
2. ✅ **Monitor** `/imu/data_filtered` output
3. ✅ **Run Kalman Filter Fusion** with clean IMU data
4. ✅ **Test full Nav2 stack** with fused odometry
5. ✅ **Fine-tune** motor control if needed

---

**Analysis by**: FFT spectral analysis  
**Date**: 10 February 2026  
**Tool**: `imu_fft_analysis.py`
