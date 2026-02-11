# Static Map SLAM Configuration

## Problem Fixed
Previously, the map was moving when the robot rotated. This happened because the SLAM node was independently estimating the robot's motion using scan matching, creating competing pose estimates.

## Solution
The SLAM node now uses **motor odometry as the primary pose source** and only maintains the occupancy grid map. This ensures:

1. **Map stays static** - The map frame is the global reference frame
2. **Robot moves on map** - Only the robot position changes relative to the map
3. **Single pose source** - Motor odometry is the authoritative pose source

## Frame Hierarchy

```
map (STATIC - maintained by SLAM)
  ↓
  └─→ map -> odom (published by SLAM for drift correction)
       ↓
       └─→ odom -> base_link (published by motor_odom_node)
            ↓
            └─→ Robot position (continuously updated from motor odometry)
```

### Frame Definitions

| Frame | Purpose | Source | Movement |
|-------|---------|--------|----------|
| `map` | Global static reference | SLAM node | Does NOT move |
| `odom` | Motor odometry frame | motor_odom_node | Accumulates drift |
| `base_link` | Robot center | motor_odom_node | Follows odometry |

## How It Works

### Previous Behavior (BROKEN)
1. LIDAR scans come in
2. SLAM calculates motion via scan matching (ICP)
3. SLAM updates robot position independently
4. Map gets rebuilt at each step
5. **Result:** Map moves with robot ❌

### New Behavior (FIXED)
1. LIDAR scans come in
2. Motor odometry provides robot position (from `/odom` topic)
3. SLAM receives the pose via `odom_callback()`
4. Scans are added to occupancy grid at the odometry position
5. Map stays fixed, only robot position changes
6. **Result:** Map is static, robot moves on it ✅

## Code Changes

### Added: Odometry Subscriber
```python
self.sub_odom = self.create_subscription(
    Odometry,
    '/odom',
    self.odom_callback,
    10
)

def odom_callback(self, msg: Odometry):
    """Receive raw motor odometry as pose source"""
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    quat = msg.pose.pose.orientation
    rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    yaw = rot.as_euler('xyz')[2]
    
    # Update robot pose from motor odometry
    self.robot_pose = np.array([x, y, yaw])
```

### Modified: Scan Callback
**Before:** Calculated motion from scan matching, updated robot pose
**After:** Uses motor odometry for pose, just adds scans to grid

```python
def scan_callback(self, msg: LaserScan):
    # Convert scan to points
    points = self.scan_to_points(msg)
    
    # Robot pose is updated via odom_callback()
    # Just store the scan at current position
    self.scan_history.append({
        'points': points,
        'time': msg.header.stamp,
        'pose': self.robot_pose.copy()  # From motor odometry!
    })
    
    # Update map with scan at correct position
    self.update_occupancy_grid()
```

### Removed: Scan Matching
- Deleted `estimate_motion()` function
- No more ICP-based motion estimation
- Completely relies on motor odometry

### Updated: Pose Publishing
- Changed `header.frame_id` from `'map'` to `'odom'`
- Pose now represents odometry frame, not SLAM's own estimate

## Configuration Checklist

✅ **Motor Odometry Node**
- Publishes `/odom` topic with `odom -> base_link` transform
- Should already be running in `start_stack.sh`

✅ **SLAM Node**
- Now subscribes to `/odom` for pose
- Publishes map in fixed `map` frame
- Publishes `map -> odom` transform for drift correction

✅ **RViz Configuration**
- Set fixed frame to `map` (Global Options → Fixed Frame: map)
- Robot will move on static map
- Transforms will show proper hierarchy

## Testing

### Visual Verification
1. Start the stack: `./start_stack.sh`
2. Open RViz2
3. Add visualization:
   - Add Map topic: Display `/map`
   - Add Pose: Display `/slam/pose`
4. Command robot to move in a circle
5. **Expected:** Map stays centered, robot rotates on the map

### Issue: Map Still Moving?
If the map still moves after these changes:

1. Check `/odom` topic is being published:
   ```bash
   ros2 topic echo /odom | head -20
   ```
   Should show position changing, not zero

2. Verify `motor_odom_node.py` is running:
   ```bash
   ros2 node list | grep motor_odom
   ```

3. Check frame setup in RViz:
   ```bash
   ros2 tf2_tools.py print_tree
   ```
   Should show: map → odom → base_link

4. If motor_odom is not running, manually start it:
   ```bash
   cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
   python3 motor_odom_node.py &
   ```

### Debug Commands
```bash
# Check if SLAM is receiving odom
ros2 node info /simple_slam

# Check transforms
ros2 tf2_tools.py print_tree

# Verify map updates
ros2 topic echo /map/info | head -5

# Check robot pose
ros2 topic echo /slam/pose
```

## Performance Impact

- **Better:** SLAM no longer wastes CPU on scan matching
- **Faster:** Direct odometry use is instant
- **Cleaner:** Single pose source eliminates contradictions
- **Realistic:** Simulates proper SLAM (map is reference frame)

## Future Improvements

1. **Scan-to-map correction:** Use ICP to correct odometry drift
2. **Loop closure:** Detect when robot revisits areas
3. **Pose graph:** Optimize all poses when loop detected
4. **Dynamic map:** Update map size based on exploration

For now, this static-map approach provides clean, understandable behavior.

## References

- Motor Odometry: `motor_odom_node.py`
- SLAM Node: `simple_slam.py`
- Transforms: `/tf` topic
- Map Visualization: RViz2
