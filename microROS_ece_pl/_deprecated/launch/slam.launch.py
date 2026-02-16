from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('microros_ece_pl'),
        'config',
        'slam_toolbox_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),
    ])
