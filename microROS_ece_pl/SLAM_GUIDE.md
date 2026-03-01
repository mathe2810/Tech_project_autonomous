# 🚀 RF2O + SLAM Architecture Guide

## Architecture Overview

```
┌─────────────────────────────────────┐
│  Pygame LIDAR Simulator (30 FPS)   │
│  - Generates 180-ray scans         │
│  - Publishes via socket (port 5005)│
│  - Sends ground truth odometry     │
└────────────────┬────────────────────┘
                 │
            (Socket)
                 │
        ┌────────▼──────────┐
        │ simu_bridge.py    │
        │ - Recv socket     │
        │ - Publish /scan   │
        │ - Publish /odom_  │
        │    ground_truth   │
        └────────┬──────────┘
                 │
        ┌────────▼──────────┐
        │ scan_restamper   │
        │ - Retimestamp    │
        │ - Publish /scan  │
        └────────┬──────────┘
                 │
    ┌────────────┼────────────┐
    │            │            │
    │       ┌────▼────┐       │
    │       │  RF2O   │       │
    │       │ Odometry│       │
    │       │/odom_   │       │
    │       │rf2o     │       │
    │       └────┬────┘       │
    │            │            │
    │       ┌────▼────┐       │
    │       │  SLAM   │       │
    │       │Toolbox  │       │
    │       │/odom    │       │
    │       │/map     │       │
    │       └────┬────┘       │
    │            │            │
    └────────────┼────────────┘
                 │
         ┌───────▼────────┐
         │  RViz Display  │
         │  - /map        │
         │  - /scan       │
         │  - /odom       │
         └────────────────┘
```

## Quick Start

### 1️⃣ **RF2O ONLY** (Raw Laser Odometry)
```bash
cd microROS_ece_pl/
./start_slam_lidar_simu.sh
```
Result: `position error ~4.4m` (no loop closure)

### 2️⃣ **RF2O + SLAM** (Closed-Loop)
```bash
./start_rf2o_slam.sh
```
Result: `position error reduced by closed-loop correction`

### 3️⃣ **Compare Both Approaches**
In another terminal:
```bash
python3 compare_rf2o_vs_slam.py
```
Shows real-time comparison:
```
🎯 GROUND TRUTH:
   Position: x=3.456m  y=4.789m
   
🔴 RF2O (Raw):
   Position: x=-0.938m  y=5.064m
   Error: dist=4.403m  Δθ=0.9°

🟢 SLAM (Closed-Loop):
   Position: x=3.412m  y=4.801m
   Error: dist=0.065m  Δθ=0.1°

📈 IMPROVEMENT: 4.403m → 0.065m (98.5% better!)
```

## Configuration Files

### `config/rf2o_params.yaml`
Controls RF2O odometry estimation:
- `laser_scan_topic`: `/scan`
- `max_iterations`: 30 (lower = faster but less accurate)
- `motion_filter_*`: Motion thresholds
- `max_laser_range`: 6.0m (from simulator)

### `config/slam_toolbox_closed_loop.yaml`
Controls SLAM behavior:
- `do_loop_closing`: **true** → enables closed-loop correction
- `loop_search_maximum_distance`: 3.0m → search for revisited areas
- `solver_plugin`: CeresSolver (optimization)
- Key penalties: `angle_variance_penalty=0.001`, `distance_variance_penalty=0.001`
  - **Low values** = SLAM trusts scan matching over odometry
  - **High values** = SLAM trusts odometry more

## How It Works

### Without SLAM (RF2O only):
1. Scan arrives → RF2O runs ICP alignment
2. Estimates movement between scans
3. **Accumulates errors** over time (drift)
4. No way to correct when revisiting areas

### With SLAM (RF2O + closed-loop):
1. Scan arrives → RF2O estimates movement
2. SLAM adds scan to pose graph
3. **Offline optimizer** continuously refines all poses
4. When loop detected (≈revisit area) → **reoptimizes entire map**
5. **Eliminates drift** retroactively

## When to Use Each

| Method | Use Case | Pros | Cons |
|--------|----------|------|------|
| **RF2O only** | Fast local nav | Real-time, CPU-light | Drifts over time |
| **RF2O+SLAM** | Exploration/Mapping | Accurate global map | Higher CPU, delayed correction |
| **SLAM alone** | Loops are important | Best accuracy | Needs good visual features |

## Advanced Tuning

For **better loop detection**:
```yaml
loop_match_minimum_response_coarse: 0.25  # ↓ easier to detect loops
do_loop_closing: true
loop_search_maximum_distance: 5.0  # ↑ search farther
```

For **faster processing**:
```yaml
minimum_travel_distance: 0.1  # ↑ process fewer scans
minimum_time_interval: 0.1    # ↑ longer between scans
max_iterations: 5             # ↓ fewer solver iterations
```

For **more accurate positioning**:
```yaml
minimum_travel_distance: 0.02  # ↓ process more scans
coarse_angle_resolution: 0.00175  # ↓ finer angle search
fine_angle_resolution: 0.00175    # ↓ finer angles
```

## Monitoring

```bash
# Watch map being built in real-time
ros2 run rviz2 rviz2 -c rviz_simu_nav2.rviz

# Monitor topics
ros2 topic hz /scan        # Should be ~30 Hz
ros2 topic hz /odom_rf2o   # Should be ~20 Hz  
ros2 topic hz /odom        # SLAM publishes less frequently
ros2 topic hz /map         # Map updates as scans come in

# Check TF tree
ros2 run tf2_tools view_frames
```

## Troubleshooting

### SLAM not correcting positions
→ Loop detection failing (uniform environment)
→ Solution: Add markers/obstacles to corridor, or reduce `loop_match_minimum_response_coarse`

### SLAM bouncing/oscillating
→ Penalties too low or too high
→ Solution: Adjust `angle_variance_penalty` / `distance_variance_penalty` (try values 0.001-0.01)

### High CPU with SLAM
→ Too many scans being processed
→ Solution: Increase `minimum_travel_distance` to 0.05-0.1m

## Next Steps

1. **Test RF2O-only**: See raw odometry drift
2. **Test with SLAM**: See loop closure correction
3. **Adjust parameters** for your environment
4. **Integrate motor encoders** for better initial estimate
5. **Add IMU** for orientation correction

