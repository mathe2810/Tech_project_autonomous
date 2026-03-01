#!/bin/bash
# Version debug du lancement avec logs détaillés

echo "=========================================="
echo "Lancement COMPLET DEBUG: LIDAR + RF2O"
echo "=========================================="

# --- Nettoyage radical ---
echo "🧹 Nettoyage des processus précédents..."
pkill -9 rf2o 2>/dev/null || true
pkill -9 slam_toolbox 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
pkill -9 scan_restamper 2>/dev/null || true
pkill -9 simu_lidar 2>/dev/null || true
pkill -9 simu_bridge 2>/dev/null || true
sleep 2

# --- Lancement avec logs ---
echo ""
echo "1️⃣ Démarrage du SIMULATEUR LIDAR (simu_lidar.py)..."
python3 simu_lidar.py 2>&1 | sed 's/^/[SIMU_LIDAR] /' &
sleep 2

echo "✅ Simulateur démarré"

echo ""
echo "2️⃣ Démarrage du BRIDGE SOCKET->ROS2 (simu_bridge.py)..."
python3 simu_bridge.py 2>&1 | sed 's/^/[SIMU_BRIDGE] /' &
sleep 1

echo "✅ Bridge démarré"

echo ""
echo "3️⃣ Démarrage du PUBLISHER TF2..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link 2>&1 | sed 's/^/[TF2] /' &

echo "✅ TF2 démarré"

echo ""
echo "4️⃣ Démarrage du RESTAMPER (scan_restamper_simu.py)..."
python3 scan_restamper_simu.py 2>&1 | sed 's/^/[RESTAMPER] /' &
sleep 1

echo "✅ Restamper démarré"

echo ""
echo "5️⃣ Démarrage du TEST LISTENER..."
python3 test_rf2o_lidar.py 2>&1 | sed 's/^/[TEST_LISTENER] /' &
sleep 1

echo "✅ Test listener démarré"

echo ""
echo "=========================================="
echo "Tous les composants sont en cours d'exécution!"
echo "=========================================="
echo ""

echo "Topics visibles (dans 3 secondes):"
sleep 3
ros2 topic list

echo ""
echo "En attente d'activation de RF2O dans 5 secondes..."
echo "Une fois que tout est ✅, tu peux configurer RF2O"
sleep 5

echo ""
echo "=========================================="
echo "⏸️ Stack prêt - RF2O prêt à être lancé"
echo "=========================================="
echo ""
echo "Commandes utiles:"
echo "  ros2 topic echo /scan_raw   # Affiche les scans bruts"
echo "  ros2 topic echo /scan       # Affiche les scans restampés"
echo "  ros2 topic echo /odom       # Affiche l'odométrie RF2O"
echo "  ./diagnose_rf2o.sh          # Diagnostic complet"
echo ""

# Keep the script running
wait
