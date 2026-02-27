#!/bin/bash
pkill -f "slam_toolbox|simu2d" || true
sleep 1
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &
python3 simu2d_robot_lidar.py &
sleep 2
python3 simu2d_ros2_publisher.py &
sleep 1
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file config/slam_toolbox_rf2o.yaml -p use_sim_time:=false