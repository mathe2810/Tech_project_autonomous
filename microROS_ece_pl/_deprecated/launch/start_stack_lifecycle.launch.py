#!/usr/bin/env python3
"""
Launch file for SLAM stack with Lifecycle management
Based on Nav2 lifecycle architecture + ldrobot-lidar-ros2 pattern

Provides clean startup/shutdown with ordered transitions:
1. Configure all nodes
2. Activate in sequence
3. Graceful shutdown on Ctrl+C
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    """Generate launch description for SLAM stack with lifecycle management"""
    
    # Get current package directory
    pkg_dir = get_package_share_directory('microros_ece_pl') if os.path.exists(
        os.path.expanduser('~/microros_ece_ws/install/microros_ece_pl')
    ) else os.path.dirname(os.path.abspath(__file__))
    
    # Config directory
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Lifecycle manager config
    lc_mgr_config = os.path.join(config_dir, 'lifecycle_mgr.yaml')
    
    # SLAM Toolbox config
    slam_config = os.path.join(config_dir, 'slam_toolbox_params.yaml')
    
    # Declare launch arguments
    declare_node_namespace_cmd = DeclareLaunchArgument(
        'node_namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Nav2 Lifecycle Manager
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config]
    )
    
    # Motor Odometry Lifecycle Node
    motor_odom_node = LifecycleNode(
        package='micro_ros_setup',
        executable='motor_odom_lifecycle.py',
        name='motor_odom',
        output='screen',
        parameters=[
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'wheel_base': 0.2},
            {'publish_rate': 50.0}
        ]
    )
    
    # Scan Restamper Lifecycle Node
    scan_restamper_node = LifecycleNode(
        package='micro_ros_setup',
        executable='scan_restamper_lifecycle.py',
        name='scan_restamper',
        output='screen'
    )
    
    # Odometry Relay Lifecycle Node
    odom_relay_node = LifecycleNode(
        package='micro_ros_setup',
        executable='simple_ekf_lifecycle.py',
        name='simple_ekf',
        output='screen'
    )
    
    # Static TF for laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link']
    )
    
    # SLAM Toolbox Lifecycle Node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(config_dir, 'slam_rviz.rviz')],
        on_exit=None
    )
    
    # Build description
    ld = LaunchDescription()
    
    # Launch arguments
    ld.add_action(declare_node_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Core nodes
    ld.add_action(lifecycle_mgr)
    ld.add_action(motor_odom_node)
    ld.add_action(scan_restamper_node)
    ld.add_action(odom_relay_node)
    ld.add_action(static_tf)
    ld.add_action(slam_node)
    
    # Optional: RViz
    ld.add_action(rviz_node)
    
    return ld
