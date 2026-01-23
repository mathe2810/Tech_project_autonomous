import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file pour agent micro_ros + RViz pour ESP32 LIDAR REEL
    Affiche tous les 360 points du LIDAR en temps reel
    Inclut le modele 3D de la voiture
    """
    
    # Get vehicle description package directory
    vehicle_description_dir = get_package_share_directory('vehicle_description')
    vehicle_launch = os.path.join(
        vehicle_description_dir, 'launch', 'vehicle_broadcaster.launch.py'
    )
    
    # Chemin vers la config RViz (visualisation 3D optimisee)
    rviz_config = os.path.join(
        os.path.dirname(__file__),
        'lidar_buffer.rviz'  # Utiliser la config RViz pour PointCloud2
    )
    
    # Delai avant de lancer RViz (laisser le temps aux topics de se creer)
    rviz_delayed = TimerAction(
        period=5.0,  # 5 secondes de delai
        actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen',
            ),
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg="\n" + "="*70),
        LogInfo(msg="[LAUNCH] ESP32 LIDAR - Configuration Micro ROS Agent + VOITURE 3D (RViz)"),
        LogInfo(msg="="*70 + "\n"),
        
        # ==================== VEHICLE DESCRIPTION (modele 3D de la voiture) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vehicle_launch),
            launch_arguments={}.items(),
        ),
        
        # ==================== TF BROADCASTER (publie la frame lidar_link) ====================
        Node(
            package='cpp_pubsub',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen',
        ),
        
        # ==================== BUFFER LIDAR (accumule les scans) ====================
        Node(
            package='cpp_pubsub',
            executable='lidar_buffer',
            name='lidar_buffer',
            output='screen',
            parameters=[
                {'buffer_size': 20},  # Garde 20 scans (plus rapide a effacer)
            ],
        ),
        
        # ==================== LISTENER (recoit et log les donnees) ====================
        Node(
            package='cpp_pubsub',
            executable='listener',
            name='lidar_subscriber',
            output='screen',
        ),
        
        # ==================== RVIZ VISUALIZER (avec delai) ====================
        rviz_delayed,
        
        LogInfo(msg="\n[OK] Listener demarré - RViz se lancera dans 5 secondes\n"),
        LogInfo(msg="[CONFIG] CONFIGURATION REQUISE:"),
        LogInfo(msg="   Lancez l'agent micro_ros dans un AUTRE terminal:"),
        LogInfo(msg="   "),
        LogInfo(msg="   docker run -it --rm microros/micro-ros-docker:humble \\"),
        LogInfo(msg="     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"),
        LogInfo(msg="   "),
        LogInfo(msg="[TOPICS] TOPICS RECUS DE L'ESP32:"),
        LogInfo(msg="   - /scan (sensor_msgs/LaserScan) - 360 points LIDAR"),
        LogInfo(msg="   - /data (std_msgs/Int32) - Compteur"),
        LogInfo(msg=""),
        LogInfo(msg="[VISUAL] VISUALISATION RVIZ (3D):"),
        LogInfo(msg="   - Modele 3D de la voiture au centre"),
        LogInfo(msg="   - Points LIDAR en rouge autour du vehicule"),
        LogInfo(msg="   - Boîte bleue = base du vehicule"),
        LogInfo(msg="   - Cylindre rouge = LIDAR sur le toit"),
        LogInfo(msg="   - Cylindres noirs = roues"),
        LogInfo(msg="   - Grille XY pour orientation spatiale"),
        LogInfo(msg="   - Frame: base_link (centre de la voiture)"),
        LogInfo(msg=""),
        LogInfo(msg="[TIPS] ASTUCES RVIZ:"),
        LogInfo(msg="   - Utilisez le panneau 'Displays' pour activer/desactiver LaserScan"),
        LogInfo(msg="   - Scroller pour zoomer, click-droit pour tourner la vue"),
        LogInfo(msg="   - Augmentez 'Decay Time' pour voir l'historique"),
        LogInfo(msg="   - Changez la couleur via 'Color Transformer'"),
        LogInfo(msg="\n" + "="*70 + "\n"),
    ])
