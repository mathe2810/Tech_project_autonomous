from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
import os

def generate_launch_description():
    """
    Launch file complet : LIDAR + ESP32 Agent micro_ros + RViz
    """
    
    # Configuration
    ros_distro = "humble"
    
    return LaunchDescription([
        # ==================== LOGS ====================
        LogInfo(msg="\n" + "="*60),
        LogInfo(msg="📊 Système Complet LIDAR + micro_ros"),
        LogInfo(msg="="*60),
        
        # ==================== PUBLISHER LIDAR ====================
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='lidar_publisher',
            output='screen',
        ),
        
        # ==================== SUBSCRIBER LIDAR ====================
        Node(
            package='cpp_pubsub',
            executable='listener',
            name='lidar_subscriber',
            output='screen',
        ),
        
        # ==================== RVIZ VISUALIZER ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(
                os.path.expanduser('~'),
                '.rviz2/default.rviz'
            )],
            # Optionnel: remplacer par config custom
        ),
        
        LogInfo(msg="✅ Système lancé avec succès!"),
        LogInfo(msg="  📡 Publisher: /scan (LaserScan) + /data (Int32)"),
        LogInfo(msg="  📥 Subscriber: Affichage console des données"),
        LogInfo(msg="  🎨 RViz: Visualisation 3D"),
        LogInfo(msg="="*60 + "\n"),
    ])
