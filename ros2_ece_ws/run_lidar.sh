#!/bin/bash

# ============================================================
# Script Helper pour lancer le système LIDAR complet
# ============================================================

set -e

WORKSPACE_PATH="$HOME/ros_test/ros2_ece_ws"
PACKAGE="cpp_pubsub"

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Fonctions
print_header() {
    echo -e "${BLUE}════════════════════════════════════════════${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}════════════════════════════════════════════${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# Vérifications
check_requirements() {
    print_header "🔍 Vérification des dépendances"
    
    if ! command -v colcon &> /dev/null; then
        print_error "colcon non installé"
        echo "Installation: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    print_success "colcon installé"
    
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 non trouvé"
        echo "Assurez-vous d'avoir source install/setup.bash"
        exit 1
    fi
    print_success "ROS2 trouvé"
    
    if [ ! -d "$WORKSPACE_PATH" ]; then
        print_error "Workspace non trouvé: $WORKSPACE_PATH"
        exit 1
    fi
    print_success "Workspace trouvé"
}

# Build
build_package() {
    print_header "🔨 Construction du package"
    
    cd "$WORKSPACE_PATH"
    
    if colcon build --packages-select $PACKAGE 2>&1 | grep -i error; then
        print_error "Compilation échouée"
        exit 1
    fi
    
    print_success "Package compilé avec succès"
}

# Source
source_setup() {
    print_header "📦 Configuration de l'environnement"
    
    source "$WORKSPACE_PATH/install/setup.bash"
    print_success "Environment configuré"
}

# Lancer le système
run_system() {
    local mode=$1
    
    print_header "🚀 Lancement du système"
    
    case $mode in
        publisher)
            print_warning "Lancement du Publisher uniquement"
            ros2 run $PACKAGE talker
            ;;
        subscriber)
            print_warning "Lancement du Subscriber uniquement"
            ros2 run $PACKAGE listener
            ;;
        both)
            print_warning "Lancement du Publisher et Subscriber"
            echo "Note: Ouvrez 2 terminaux séparés:"
            echo "  Terminal 1: ros2 run $PACKAGE talker"
            echo "  Terminal 2: ros2 run $PACKAGE listener"
            ;;
        launch)
            print_warning "Lancement avec launch file"
            ros2 launch $PACKAGE lidar_launch.py
            ;;
        launch-complete)
            print_warning "Lancement complet (Publisher + Subscriber + RViz)"
            ros2 launch $PACKAGE lidar_complete_launch.py
            ;;
        agent)
            print_warning "Lancement avec agent micro_ros"
            ros2 launch $PACKAGE agent_launch.py
            ;;
        monitor)
            print_warning "Monitoring des topics"
            echo "Topics disponibles:"
            ros2 topic list
            echo ""
            echo "Fréquence des topics:"
            for topic in /scan /data; do
                if ros2 topic list | grep -q $topic; then
                    echo "  $topic: $(ros2 topic hz $topic --window 3 2>/dev/null | tail -1)"
                fi
            done
            ;;
        *)
            print_error "Mode inconnu: $mode"
            print_usage
            exit 1
            ;;
    esac
}

# Afficher l'aide
print_usage() {
    cat << EOF
${BLUE}Usage: $0 [OPTION]${NC}

Options:
    ${GREEN}publisher${NC}       Lancer uniquement le Publisher
    ${GREEN}subscriber${NC}      Lancer uniquement le Subscriber
    ${GREEN}both${NC}           Afficher les commandes pour les 2 nodes
    ${GREEN}launch${NC}         Lancer avec launch file (Publisher + Subscriber)
    ${GREEN}launch-complete${NC} Lancer complet (Publisher + Subscriber + RViz)
    ${GREEN}agent${NC}          Lancer pour recevoir du ESP32 Agent
    ${GREEN}monitor${NC}        Monitor les topics ROS2
    ${GREEN}build${NC}          Compiler le package seulement
    ${GREEN}clean${NC}          Nettoyer la compilation
    ${GREEN}-h, --help${NC}      Afficher cette aide

Exemples:
    $0 launch              # Lancer simple
    $0 publisher           # Lancer le publisher
    $0 monitor             # Voir les topics actifs

Topics ROS2:
    /scan                  LaserScan (données LIDAR)
    /data                  Int32 (compteur)

Fichiers importants:
    Launch files: $WORKSPACE_PATH/src/$PACKAGE/launch/
    Package config: $WORKSPACE_PATH/src/$PACKAGE/CMakeLists.txt
    README: $WORKSPACE_PATH/src/$PACKAGE/README.md
EOF
}

# Main
main() {
    if [ $# -eq 0 ]; then
        print_usage
        exit 0
    fi
    
    case $1 in
        -h|--help)
            print_usage
            exit 0
            ;;
        build)
            check_requirements
            build_package
            exit 0
            ;;
        clean)
            print_header "🧹 Nettoyage"
            cd "$WORKSPACE_PATH"
            rm -rf build install log
            print_success "Nettoyage terminé"
            exit 0
            ;;
        *)
            check_requirements
            build_package
            source_setup
            run_system "$1"
            ;;
    esac
}

# Execution
main "$@"
