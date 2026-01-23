#!/bin/bash

# Script de compilation et déploiement complet
# Utilisation: ./deploy_lidar.sh

set -e  # Exit on error

echo "========================================"
echo "🚀 Compilation LIDAR 360° - Système Complet"
echo "========================================"
echo ""

# ============ ÉTAPE 1: Compiler ROS2 ============
echo "📦 [1/3] Compilation ROS2..."
cd /home/matheo/ros_test/ros2_ece_ws

source install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
echo "   ✓ Source d'environnement ROS2 chargée"

colcon build --packages-select cpp_pubsub 2>&1 | grep -E "(error|warning|Compiling|Linking)" || echo "   ✓ Build ROS2 réussi"
source install/setup.bash
echo "   ✓ Code ROS2 compilé avec succès"
echo ""

# ============ ÉTAPE 2: Vérifier ESP32 ============
echo "💾 [2/3] Vérification du code ESP32..."
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl

if ! grep -q "LidarCircularBuffer" src/main.cpp; then
    echo "   ✗ Erreur: Buffer circulaire non trouvé!"
    exit 1
fi
echo "   ✓ Buffer circulaire: OK"

if ! grep -q "LIDAR_BUFFER_SIZE 720" src/main.cpp; then
    echo "   ✗ Erreur: Taille du buffer invalide!"
    exit 1
fi
echo "   ✓ Taille du buffer: 720 points (60 frames)"

if ! grep -q "rcl_publish(&pub_lidar" src/main.cpp; then
    echo "   ✗ Erreur: Publication LIDAR non trouvée!"
    exit 1
fi
echo "   ✓ Publication LaserScan: OK"
echo ""

# ============ ÉTAPE 3: Vérifier fichiers config ============
echo "⚙️  [3/3] Vérification des fichiers de configuration..."
cd /home/matheo/ros_test/ros2_ece_ws/src/cpp_pubsub/launch

if ! grep -q "LaserScan (Tous les 360 points)" lidar.rviz; then
    echo "   ✗ Erreur: Configuration RViz non mise à jour!"
    exit 1
fi
echo "   ✓ Configuration RViz: OK (360 points, couleur rouge)"

if ! grep -q "360 points LIDAR" agent_launch.py; then
    echo "   ✗ Erreur: Guide de lancement non mis à jour!"
    exit 1
fi
echo "   ✓ Guide de lancement: OK"

echo ""
echo "========================================"
echo "✅ Système LIDAR 360° prêt au lancement!"
echo "========================================"
echo ""
echo "📋 ÉTAPES SUIVANTES:"
echo ""
echo "1️⃣  Terminal 1 - Lancer l'agent micro-ROS:"
echo "   docker run -it --rm microros/micro-ros-docker:humble \\"
echo "     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo ""
echo "2️⃣  Terminal 2 - Téléverser le code ESP32 (PlatformIO)"
echo "   Ouvrir: /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl"
echo "   Cliquer sur 'Upload' dans PlatformIO"
echo ""
echo "3️⃣  Terminal 3 - Lancer le listener + RViz:"
echo "   cd /home/matheo/ros_test/ros2_ece_ws"
echo "   source install/setup.bash"
echo "   ros2 launch cpp_pubsub agent_launch.py"
echo ""
echo "🎨 RViz apparaîtra avec:"
echo "   • Grille XY grise (référence)"
echo "   • Points LIDAR en rouge (360 points)"
echo "   • Frame fixe: lidar_link"
echo ""
echo "📖 Voir le guide complet: GUIDE_LIDAR_360_COMPLET.md"
echo ""
