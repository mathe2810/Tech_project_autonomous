from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    potentiometer_processor = Node(
        package='cpp_pubsub',
        executable='ultrasonic_processor',
        name='potentiometer_processor',
        output='screen'
    )

    return LaunchDescription([potentiometer_processor])