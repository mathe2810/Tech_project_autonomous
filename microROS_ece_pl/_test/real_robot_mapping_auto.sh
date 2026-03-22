#!/bin/bash
# REAL_ROBOT_MAPPING_AUTO.SH - Script de lancement du système complet
# Lance tous les nœuds du système de mapping autonome en bon ordre

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/mapping_auto_logs"

# Créer dossier logs
mkdir -p "$LOG_DIR"

echo "🚀 =========================================="
echo "🚀 REAL ROBOT AUTONOMOUS MAPPING SYSTEM"
echo "🚀 =========================================="
echo ""

# Fonction de nettoyage (Ctrl+C)
cleanup() {
    echo ""
    echo "⏹️  Arrêt détecté (Ctrl+C)"
    echo "🔴 Fermeture de tous les nœuds..."
    
    # Tuer tous les processus enfants
    jobs -p | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    jobs -p | xargs -r kill -KILL 2>/dev/null || true
    
    echo "✅ Arrêt complet"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Vérifier que SLAM Toolbox est lancé
echo "⏳ Vérification SLAM Toolbox..."
if ! timeout 2 ros2 topic list 2>/dev/null | grep -q "/map"; then
    echo "❌ ERREUR: SLAM Toolbox ne semble pas lancé"
    echo "   Veuillez d'abord lancer: ./start_robot_mapping_auto.sh"
    exit 1
fi
echo "✅ SLAM Toolbox détecté"
echo ""

# Étape 1: Lancer wall_centering_node (autonomie)
echo "📍 Step 1/4: Lancement wall_centering_node (autonomie)..."
python3 "$SCRIPT_DIR/wall_centering_node.py" \
    > "$LOG_DIR/wall_centering.log" 2>&1 &
WALL_PID=$!
sleep 2
echo "   ✅ PID: $WALL_PID"
echo ""

# Étape 2: Lancer loop_closure_detector
echo "📍 Step 2/4: Lancement loop_closure_detector (détection boucle)..."
python3 "$SCRIPT_DIR/loop_closure_detector.py" \
    > "$LOG_DIR/loop_closure.log" 2>&1 &
LOOP_PID=$!
sleep 1
echo "   ✅ PID: $LOOP_PID"
echo ""

# Étape 3: Lancer trajectory_saver
echo "📍 Step 3/4: Lancement trajectory_saver (sauvegarde trajec)..."
python3 "$SCRIPT_DIR/trajectory_saver.py" \
    > "$LOG_DIR/trajectory_saver.log" 2>&1 &
SAVER_PID=$!
sleep 1
echo "   ✅ PID: $SAVER_PID"
echo ""

# Étape 4: Lancer return_path_generator
echo "📍 Step 4/4: Lancement return_path_generator (A* planning)..."
python3 "$SCRIPT_DIR/return_path_generator.py" \
    > "$LOG_DIR/return_path.log" 2>&1 &
RETURN_PID=$!
sleep 1
echo "   ✅ PID: $RETURN_PID"
echo ""

# Lancer orchestrateur principal
echo "🎬 =========================================="
echo "🎬 Lancement orchestrateur principal..."
echo "🎬 =========================================="
echo ""

python3 "$SCRIPT_DIR/real_robot_mapping_auto.py"

# Attendre fin
wait

echo ""
echo "✅ Système arrêté proprement"
