# RViz Configuration for Static Map Display

## Quick Setup

For the static map to display correctly in RViz2:

### 1. Set Global Fixed Frame to 'map'
```
Global Options
├── Fixed Frame: map
└── (NOT odom, NOT base_link)
```

**Why?** The `map` frame is static. Setting it as fixed keeps the map centered and stationary while the robot moves across it.

### 2. Add Map Display
```
Displays
├── Add Display
├── Type: Map
├── Topic: /map
└── Enable: ✓
```

### 3. Add Robot Pose Display
```
Displays
├── Add Display
├── Type: Pose
├── Topic: /slam/pose
└── Enable: ✓
```

### 4. Add TF Transform Display (Optional)
```
Displays
├── Add Display
├── Type: TF
├── Show Names: ✓
└── Show Arrows: ✓
```

## Correct Frame Hierarchy

After proper setup, RViz will show:

```
map (origin at center, DOES NOT MOVE)
  ├── Grid: 5m x 5m (or configured size)
  └── Occupancy: Red/black cells showing obstacles
     
robot (moves across the map)
  ├── Pose: Updates position as robot moves
  └── TF Frame: Shows odom->base_link transform
```

## Expected Behavior

### When Robot Moves Forward
- ✅ Robot marker moves toward top of map
- ✅ Map stays centered (doesn't scroll)
- ❌ NOT: Map scrolls, robot stays centered

### When Robot Rotates
- ✅ Robot orientation updates
- ✅ Map does NOT rotate
- ✅ New LIDAR scans added in correct location
- ❌ NOT: Map rotates with robot

### When Robot Makes a Circle
- ✅ Robot traces circle on static map
- ✅ Map shows walls/obstacles in fixed positions
- ❌ NOT: Map rotates to follow robot

## Troubleshooting

### Problem: Map is moving/rotating with robot

**Check 1: Fixed Frame Setting**
- In RViz: Global Options → Fixed Frame
- Should be: `map` (not `odom` or `base_link`)

**Check 2: /odom topic is publishing**
```bash
ros2 topic echo /odom | head -20
```
Should show position values changing

**Check 3: Transform tree is correct**
```bash
ros2 run tf2_tools tf2_tree
```
Should show: map → odom → base_link

**Check 4: SLAM is running**
```bash
ros2 node list | grep simple_slam
```
Should show `/simple_slam` node

### Problem: Robot not visible on map

**Check 1: /scan topic exists**
```bash
ros2 topic list | grep scan
```

**Check 2: Motor odometry running**
```bash
ros2 node list | grep motor_odom
```

**Check 3: Add robot model to RViz**
```
Displays → Add Display
Type: RobotModel
Fixed Frame: base_link
```

## Default RViz Config

For convenience, a saved RViz config is available:
```
./lidar_config.rviz
```

To use it:
```bash
rviz2 -d lidar_config.rviz
```

## RViz Settings File (rviz2.yaml)

Minimal static map config:

```yaml
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_common/Grid
      Name: Grid
      Enabled: true
      
    - Class: rviz_common/Map
      Name: Map
      Enabled: true
      Topic: /map
      
    - Class: rviz_common/Pose
      Name: Pose
      Enabled: true
      Topic: /slam/pose
      
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
```

## Complete Startup Sequence

```bash
# Terminal 1: Start stack (agent, nodes, RViz)
./start_stack.sh

# Terminal 2: Monitor transforms (optional)
ros2 run tf2_tools tf2_tree --watch

# Terminal 3: Monitor odometry (optional)
ros2 topic echo /odom

# In RViz:
# 1. Global Options → Fixed Frame: map
# 2. Add Map display, topic: /map
# 3. Add Pose display, topic: /slam/pose
# 4. Observe robot moving on static map
```

## Advanced: Custom Frame Alignment

If you want the map origin at a different location:

Edit `simple_slam.py`:
```python
# Currently: map origin at center of grid
center_idx = self.grid_width // 2

# To align with robot start:
center_idx = 0  # or other offset
```

This affects the `origin` field in the OccupancyGrid message.

## Notes

- **Frame convention:** NED (North-East-Down) in ROS2
  - X: Forward
  - Y: Left  
  - Z: Up

- **Map resolution:** 0.05m per cell (5cm) → 100x100 cells = 5m x 5m
  - Adjust in `simple_slam.py` → `grid_size` parameter

- **Transform publication rate:** 10Hz (0.1s)
  - Matching the map publication rate

## See Also

- `SLAM_STATIC_MAP_FIX.md` - Technical explanation
- `simple_slam.py` - SLAM implementation
- `motor_odom_node.py` - Odometry source
