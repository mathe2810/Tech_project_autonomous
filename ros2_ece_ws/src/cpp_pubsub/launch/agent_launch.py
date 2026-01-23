import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file pour agent micro_ros + Foxglove Studio pour ESP32 LIDAR REEL
    Affiche tous les 360 points du LIDAR en temps reel dans Foxglove
    Inclut le modele 3D de la voiture
    """
    
    # Get vehicle description package directory
    vehicle_description_dir = get_package_share_directory('vehicle_description')
    vehicle_launch = os.path.join(
        vehicle_description_dir, 'launch', 'vehicle_broadcaster.launch.py'
    )
    
    # Delai avant de lancer Foxglove (laisser le temps aux topics de se creer)
    foxglove_delayed = TimerAction(
        period=2.0,  # 2 secondes de delai (plus court)
        actions=[
            ExecuteProcess(
                cmd=['/usr/bin/foxglove-studio'],
                output='screen',
            ),
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg="\n" + "="*70),
        LogInfo(msg="[LAUNCH] ESP32 LIDAR - Configuration Micro ROS Agent + VOITURE 3D (Foxglove)"),
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
        
        # ==================== FOXGLOVE VISUALIZER (avec delai) ====================
        foxglove_delayed,
        
        LogInfo(msg="\n[OK] Listener demarré - Foxglove Studio se lancera dans 3 secondes\n"),
        LogInfo(msg="[CONFIG] CONFIGURATION REQUISE:"),
        LogInfo(msg="   Lancez l'agent micro_ros dans un AUTRE terminal:"),
        LogInfo(msg="   "),
        LogInfo(msg="   docker run -it --rm microros/micro-ros-docker:humble \\"),
        LogInfo(msg="     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"),
        LogInfo(msg="   "),
        LogInfo(msg="[TOPICS] TOPICS RECUS DE L'ESP32:"),
        LogInfo(msg="   - /scan (sensor_msgs/LaserScan) - 360 points LIDAR"),
        LogInfo(msg="   - /data (std_msgs/Int32) - Compteur"),
        LogInfo(msg="   - /scan_cloud (sensor_msgs/PointCloud2) - Points 3D"),
        LogInfo(msg=""),
        LogInfo(msg="[VISUAL] VISUALISATION FOXGLOVE (3D):"),
        LogInfo(msg="   - Interface web moderne et intuitive"),
        LogInfo(msg="   - Modele 3D de la voiture au centre"),
        LogInfo(msg="   - Points LIDAR affichés en temps reel"),
        LogInfo(msg="   - Panneau gauche pour selectionner les topics a afficher"),
        LogInfo(msg="   - Affichage du compteur /data"),
        LogInfo(msg=""),
        LogInfo(msg="[TIPS] FOXGLOVE STUDIO:"),
        LogInfo(msg="   1. Cliquez sur 'Add panel' (haut droit)"),
        LogInfo(msg="   2. Selectionnez '3D' pour la vue 3D"),
        LogInfo(msg="   3. Selectionnez '/scan_cloud' dans la liste des topics"),
        LogInfo(msg="   4. Utilisez le panneau 'Telemetry' pour voir les valeurs"),
        LogInfo(msg="   5. Naviguez avec la souris: scroll=zoom, click-droit=rotation"),
        LogInfo(msg="   6. Utilisez 'Time scrubbing' pour rejouer l'historique"),
        LogInfo(msg="\n" + "="*70 + "\n"),
    ])
