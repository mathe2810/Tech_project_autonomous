#!/bin/bash

# Lance RViz avec config robuste (ignore les problèmes d'horloge ESP32)

source /opt/ros/humble/setup.bash
source /home/matheo/ros_test/ros2_ece_ws/install/setup.bash

# Crée une config RViz qui ignore les timestamps
cat > /tmp/slam_view.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Map1
        - /LaserScan1
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 0.1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Line
      Name: Grid
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 0.8
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 0
      Color Transformer: Intensity
      Decay Time: 0.2
      Enabled: true
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Tolerance: 0.1
      Queue Size: 20
      Size (Pixels): 4
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Transformation: raw
      Use Fixed Frame: true
      Use Rainbow: true
      Value: true
    - Angle Tolerance: 0.1
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 0; 0
          Enabled: false
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.3
          Color: 204; 51; 204
          Enabled: false
          Offset: 1
          Scale: 1
          Value: true
        Value: false
      Enabled: true
      Keep: 50
      Name: Odometry (Filtered)
      Position Tolerance: 0.1
      Shape:
        Alpha: 1
        Axes Length: 0.3
        Axes Radius: 0.03
        Color: 0; 255; 0
        Head Length: 0.3
        Head Radius: 0.1
        Shaft Length: 0.3
        Shaft Radius: 0.05
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /odom_filtered
      Value: true
    - Class: rviz_default_plugins/TF
      Enable Light: true
      Enabled: true
      Frame Timeout: 200
      Frames:
        All Enabled: true
      Marker Scale: 0.3
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        map:
          odom:
            base_link: {}
      Update Interval: 0.05
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 15
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 1.5708
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0
Window Geometry:
  Displays:
    collapsed: false
  Height: 1080
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd0000000400000000000002f30000037afc0200000008fc0000000000000250000002b5fc0100000001fb0000000a0056006900650077007301000000000000038a0000010000fffffffb000000100054006f006f006c002000500072006f00700065007200740069006500730000000000ffffffff0000006700ffffff000000030000078000000041fc0100000001fb0000001400540072006100630065006200610063006b0000000000000007800000000000000000000004c00000037a00000001000000020000000100000002fc0000000100000002000000010000000a000000000000000000000000000000000000000000fbfc00000000
Width: 1920
Wrapping: 200
EOF

echo "🚀 Lançant RViz (Frame Timeout: 200s, ignore horloge ESP32)..."
rviz2 -d /tmp/slam_view.rviz --ros-args --remap __node:=rviz2_slam
