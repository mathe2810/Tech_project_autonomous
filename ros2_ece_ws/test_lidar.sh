#!/bin/bash

# ============================================================
# Script de test rapide - ROS2 LIDAR PubSub
# ============================================================

set -e

WORKSPACE="$HOME/ros_test/ros2_ece_ws"
PACKAGE="cpp_pubsub"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${BLUE}🧪 Test ROS2 LIDAR Publisher/Subscriber${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}\n"

# Source ROS2
echo -e "${YELLOW}📦 Sourcing ROS2 environment...${NC}"
source "$WORKSPACE/install/setup.bash"

# Vérification directories
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}✗ Workspace not found: $WORKSPACE${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Workspace found${NC}\n"

# Fonction: Test Publishers
test_publisher() {
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}Test 1: PUBLISHER (talker)${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}\n"
    
    echo "Lançage du publisher dans 5 secondes..."
    echo "Arrêtez avec: Ctrl+C\n"
    
    sleep 2
    
    timeout 5 ros2 run $PACKAGE talker || true
    
    echo -e "\n${GREEN}✓ Publisher test terminé${NC}\n"
}

# Fonction: Test Subscribers
test_subscriber() {
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}Test 2: SUBSCRIBER (listener)${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}\n"
    
    echo "Note: Lancez le publisher dans un autre terminal d'abord!"
    echo "Puis lancez ce test.\n"
    
    timeout 10 ros2 run $PACKAGE listener || true
    
    echo -e "\n${GREEN}✓ Subscriber test terminé${NC}\n"
}

# Fonction: Test Topics
test_topics() {
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}Test 3: ROS2 TOPICS${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}\n"
    
    echo -e "${YELLOW}Topics disponibles:${NC}"
    ros2 topic list || true
    
    echo ""
    echo -e "${YELLOW}Info sur /scan:${NC}"
    ros2 topic info /scan || true
    
    echo ""
    echo -e "${YELLOW}Info sur /data:${NC}"
    ros2 topic info /data || true
    
    echo -e "\n${GREEN}✓ Topics test terminé${NC}\n"
}

# Fonction: Test Launch
test_launch() {
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}Test 4: LAUNCH FILE${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}\n"
    
    echo "Lançage du launch file pour 10 secondes..."
    echo "Arrêtez avec: Ctrl+C\n"
    
    sleep 2
    
    timeout 10 ros2 launch $PACKAGE lidar_launch.py || true
    
    echo -e "\n${GREEN}✓ Launch file test terminé${NC}\n"
}

# Fonction: Full System Test
test_full_system() {
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}Test 5: FULL SYSTEM (Publisher + Subscriber)${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}\n"
    
    echo -e "${YELLOW}Lançage du système complet...${NC}\n"
    
    # Lancer publisher en arrière-plan
    ros2 run $PACKAGE talker &
    PUB_PID=$!
    
    sleep 2
    
    # Lancer subscriber (5 secondes)
    timeout 5 ros2 run $PACKAGE listener || true
    
    # Tuer publisher
    kill $PUB_PID 2>/dev/null || true
    
    echo -e "\n${GREEN}✓ Full system test terminé${NC}\n"
}

# Menu
case $1 in
    1|publisher)
        test_publisher
        ;;
    2|subscriber)
        test_subscriber
        ;;
    3|topics)
        test_topics
        ;;
    4|launch)
        test_launch
        ;;
    5|full)
        test_full_system
        ;;
    all)
        echo -e "${YELLOW}Exécution de tous les tests...${NC}\n"
        test_publisher
        echo "Appuyez sur Entrée pour continuer..."
        read
        test_topics
        test_launch
        ;;
    *)
        cat << EOF
${BLUE}Usage: $0 [TEST]${NC}

Tests disponibles:
  ${GREEN}1, publisher${NC}    Tester le Publisher
  ${GREEN}2, subscriber${NC}   Tester le Subscriber
  ${GREEN}3, topics${NC}       Tester les Topics ROS2
  ${GREEN}4, launch${NC}       Tester le Launch File
  ${GREEN}5, full${NC}        Tester le système complet
  ${GREEN}all${NC}            Exécuter tous les tests

Exemples:
  $0 publisher
  $0 launch
  $0 all

EOF
        exit 0
        ;;
esac

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Tests terminés avec succès!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
