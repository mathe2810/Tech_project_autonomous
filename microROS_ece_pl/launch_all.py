#!/usr/bin/env python3
"""
Launch file pour démarrer:
1. Micro-ROS agent
2. RViz avec la config LIDAR/IMU
3. Nœud Kalman filter IMU
"""

import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    cpp_pubsub_share = get_package_share_directory('cpp_pubsub')
    rviz_config = os.path.join(cpp_pubsub_share, 'launch', 'lidar_buffer_imu.rviz')
    
    # 1. Micro-ROS Agent (DDS-XRCE bridge)
    agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen',
        name='micro_ros_agent'
    )
    
    # 2. RViz with config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        name='rviz2'
    )
    
    # 3. IMU Kalman Filter node
    imu_filter = Node(
        package='cpp_pubsub',  # Or your actual package
        executable='imu_kalman_filter.py',
        output='screen',
        name='imu_kalman_filter'
    )
    
    # 4. Scan Accumulator node (720 points from 60 frames)
    scan_accumulator = Node(
        package='cpp_pubsub',
        executable='scan_accumulator.py',
        output='screen',
        name='scan_accumulator'
    )
    
    return LaunchDescription([
        agent,
        rviz,
        imu_filter,
        scan_accumulator,
    ])
