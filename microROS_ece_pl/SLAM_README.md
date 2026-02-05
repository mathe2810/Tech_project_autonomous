# SLAM Implementation - Mapping with LIDAR

## Overview
This is a **simple but functional SLAM system** that maps the environment using only LIDAR scans. It works in two stages:

### Stage 1: Motion Estimation (ICP Scan Matching)
- **Input**: Two consecutive LIDAR scans (N points each)
- **Algorithm**: Iterative Closest Point (ICP)
  1. Find nearest neighbors between scans
  2. Compute rotation + translation using SVD
  3. Iterate until convergence (usually 3-5 iterations)
- **Output**: Estimated robot motion (dx, dy, dθ)

### Stage 2: Occupancy Grid Mapping
- **Grid**: 50m × 50m with 10cm resolution = 500 × 500 cells
- **Update**: Transform each scan to world frame using current pose
- **Occupancy**: Mark cells containing LIDAR hits as occupied (100)
- **Publishing**: `/map` topic (OccupancyGrid) at LIDAR rate (~30 Hz)

## Usage

```bash
# Start complete stack (agent + RViz + IMU filter + SLAM)
./start_stack.sh

# Or run SLAM separately
python3 simple_slam.py
```

## ROS2 Topics

### Inputs
- `/scan` - LaserScan from ESP32 LIDAR (raw, polar coordinates)
- `/imu/data` - Raw IMU from ESP32

### Outputs
- `/map` - OccupancyGrid (50×50m, 10cm/cell)
- `/slam/pose` - Current estimated robot pose (PoseStamped)
- `/slam/local_scans` - Last N scans in world frame (PointCloud2)

### Transforms
- `map` → `base_link` - Robot position/orientation estimated by SLAM

## Parameters

Launch with custom parameters:
```bash
python3 simple_slam.py --ros-args \
  -p grid_size:=100 \
  -p resolution:=0.05 \
  -p max_range:=15.0
```

- `grid_size`: Map extent in meters (default: 50m)
- `resolution`: Cell size in meters (default: 0.1m = 10cm)
- `max_range`: LIDAR max range to use (default: 12m)

## Accuracy & Limitations

**Pros:**
- Simple, fast (runs at 30 FPS)
- No loop closure yet (drift accumulates)
- Works well for short trajectories (<100m)
- Real-time occupancy grid for navigation

**Cons:**
- Pure odometry → drift over long distances
- No loop-closure detection
- Fails in featureless areas (open space)

## Next Steps: Full SLAM

1. **Loop Closure**: Detect when robot returns to known location
2. **Graph Optimization**: Use g2o/ceres to correct accumulated drift
3. **Uncertainty Propagation**: Covariance matrices for each pose
4. **Dynamic Obstacles**: Separate moving objects from static map
5. **Relocalization**: Recover from global localization loss

## Testing Procedure

1. **Circuit Test**: Drive full circle, check if map aligns at start/end
2. **Drift Measurement**: Compare loop closure error vs distance
3. **Occupancy Grid**: Verify walls/obstacles appear correctly
4. **RViz Visualization**: Watch map building in real-time

## Debugging

Check SLAM output:
```bash
# Monitor pose estimation
ros2 topic echo /slam/pose

# Check map updates
ros2 topic hz /map

# View occupancy grid
# (Add Map display in RViz, set topic to /map)
```

## References
- ICP Algorithm: https://en.wikipedia.org/wiki/Iterative_closest_point
- ROS2 Nav2: Uses similar concepts at production scale
- Cartographer: Google's sophisticated SLAM system
