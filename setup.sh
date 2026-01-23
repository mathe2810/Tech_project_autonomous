#!/bin/bash

###############################################################################
# SETUP GUIDE - ECE ROS2 + microROS Project
# 
# This script installs all dependencies for the project
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║ $1${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}→ $1${NC}"
}

# Check OS
if [[ ! "$OSTYPE" == "linux-gnu"* ]]; then
    print_error "This script is designed for Linux. Please run it on Ubuntu 22.04+"
    exit 1
fi

print_header "ECE ROS2 + microROS Setup"

# Update system
print_info "Mise à jour du système..."
sudo apt update
sudo apt upgrade -y

# Install ROS2 Humble
print_header "Installation ROS2 Humble"

if [ ! -d "/opt/ros/humble" ]; then
    print_info "Ajout du repository ROS2..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    print_info "Installation de ROS2 Humble (cela peut prendre quelques minutes)..."
    sudo apt install ros-humble-desktop -y
    print_success "ROS2 Humble installé"
else
    print_success "ROS2 Humble déjà installé"
fi

# Install colcon
print_header "Installation Colcon"
print_info "Installation de colcon..."
sudo apt install python3-colcon-common-extensions -y
print_success "Colcon installé"

# Install RViz2
print_header "Installation RViz2"
sudo apt install ros-humble-rviz2 -y
print_success "RViz2 installé"

# Install PlatformIO
print_header "Installation PlatformIO"
if ! command -v platformio &> /dev/null; then
    print_info "Installation de PlatformIO..."
    pip install platformio
    print_success "PlatformIO installé"
else
    print_success "PlatformIO déjà installé"
fi

# Install Docker
print_header "Installation Docker"
if ! command -v docker &> /dev/null; then
    print_info "Installation de Docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    print_success "Docker installé"
    print_info "⚠️  Déconnectez-vous et reconnectez-vous pour que les permissions docker prennent effet"
else
    print_success "Docker déjà installé"
fi

# Pull microROS Docker image
print_header "Image Docker microROS"
print_info "Téléchargement de l'image microROS..."
docker pull microros/micro-ros-docker:humble
print_success "Image microROS téléchargée"

# Install Python dependencies
print_header "Dépendances Python"
print_info "Installation des dépendances Python..."
pip install numpy matplotlib pygame
print_success "Dépendances Python installées"

# Create sourcing script
print_header "Script de Configuration"
cat > ~/.bashrc_ros2_ece << 'EOF'
# ECE ROS2 + microROS Setup

# ROS2 Humble
source /opt/ros/humble/setup.bash

# Workspace
if [ -f ~/ros_test/ros2_ece_ws/install/setup.bash ]; then
    source ~/ros_test/ros2_ece_ws/install/setup.bash
fi

# Aliases
alias build_ros2='cd ~/ros_test/ros2_ece_ws && colcon build && source install/setup.bash'
alias run_lidar='cd ~/ros_test/ros2_ece_ws && ros2 run cpp_pubsub listener'
alias run_rviz='cd ~/ros_test/ros2_ece_ws && ros2 launch rviz2 rviz2 -d lidar_config.rviz'
alias run_agent='docker run -it --rm microros/micro-ros-docker:humble ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888'
alias build_pio='cd ~/Documents/PlatformIO/Projects/microROS_ece_pl && platformio run'
alias upload_pio='cd ~/Documents/PlatformIO/Projects/microROS_ece_pl && platformio run --target upload'
EOF

# Add to bashrc
if ! grep -q "ros2_ece" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Load ECE ROS2 configuration" >> ~/.bashrc
    echo "if [ -f ~/.bashrc_ros2_ece ]; then" >> ~/.bashrc
    echo "    source ~/.bashrc_ros2_ece" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
fi

print_success "Script de configuration créé"

# Build workspace
print_header "Compilation du Workspace ROS2"
print_info "Cela peut prendre 5-10 minutes..."
cd ~/ros_test/ros2_ece_ws
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct+
print_success "Workspace compilé"

# Summary
print_header "Installation Complétée! ✓"
echo ""
echo "Vérifiez votre installation:"
echo ""
print_info "Source la configuration ROS2:"
echo "  source ~/.bashrc_ros2_ece"
echo ""
print_info "Ou utiliser les aliases créés:"
echo "  build_ros2    - Compiler le workspace ROS2"
echo "  run_lidar     - Lancer le listener LIDAR"
echo "  run_rviz      - Lancer RViz"
echo "  run_agent     - Lancer l'agent microROS"
echo "  build_pio     - Compiler le firmware PlatformIO"
echo "  upload_pio    - Uploader vers l'ESP32"
echo ""
print_info "Commandes de test:"
echo "  ros2 --version"
echo "  platformio --version"
echo "  docker --version"
echo ""
print_info "Prochaines étapes:"
echo "  1. Déconnectez-vous et reconnectez-vous (pour les permissions Docker)"
echo "  2. Ouvrez un nouveau terminal"
echo "  3. Lancez: source ~/.bashrc_ros2_ece"
echo "  4. Suivez le QUICKSTART.md pour démarrer le projet"
echo ""

print_success "Installation prête! Bon développement! 🚀"
