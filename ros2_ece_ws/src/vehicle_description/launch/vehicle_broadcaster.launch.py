import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    
    # Get package directory
    vehicle_description_dir = get_package_share_directory('vehicle_description')
    
    # URDF file path
    urdf_file = os.path.join(vehicle_description_dir, 'urdf', 'vehicle.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    ))
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}],
        output='screen'
    )
    
    # Vehicle TF Broadcaster (publishes base_link pose)
    vehicle_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    ld.add_action(robot_state_publisher)
    ld.add_action(vehicle_broadcaster)
    
    return ld
