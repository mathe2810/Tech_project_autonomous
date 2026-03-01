#!/bin/bash
# Script de diagnostic pour vérifier que RF2O peut voir les données LIDAR

echo "=========================================="
echo "DIAGNOSTIC STACK RF2O + LIDAR SIMULÉ"
echo "=========================================="
echo ""

echo "1️⃣ Vérification des topics ROS 2..."
echo "--- Topics disponibles ---"
ros2 topic list 2>/dev/null || echo "❌ ROS 2 pas initialisé"

echo ""
echo "2️⃣ Vérification du topic /scan_raw..."
if ros2 topic list 2>/dev/null | grep -q "scan_raw"; then
    echo "✅ /scan_raw existe"
    echo "--- Info du topic ---"
    ros2 topic info /scan_raw
else
    echo "❌ /scan_raw n'existe pas!"
fi

echo ""
echo "3️⃣ Vérification du topic /scan..."
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "✅ /scan existe"
    echo "--- Info du topic ---"
    ros2 topic info /scan
else
    echo "❌ /scan n'existe pas!"
fi

echo ""
echo "4️⃣ Vérification du topic /odom..."
if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
    echo "✅ /odom existe (RF2O publie)"
    echo "--- Info du topic ---"
    ros2 topic info /odom
else
    echo "❌ /odom n'existe pas! RF2O ne publie pas"
fi

echo ""
echo "5️⃣ Vérification des TF..."
ros2 tf2_py tf_echo base_link laser_link 2>/dev/null && echo "✅ Transform base_link -> laser_link OK" || echo "❌ Transform manquante"

echo ""
echo "6️⃣ Diagnostic RF2O..."
ps aux | grep rf2o_laser_odometry | grep -v grep && echo "✅ RF2O node est en cours d'exécution" || echo "❌ RF2O node ne s'exécute pas"

echo ""
echo "=========================================="
echo "Fin du diagnostic"
echo "=========================================="
