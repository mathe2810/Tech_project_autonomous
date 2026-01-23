# 🚀 INSTALLATION RAPIDE - 5 MINUTES

Guide pour mettre en place rapidement le projet ECE ROS2 + microROS LIDAR 360°.

## 📋 Étapes

### 1️⃣ Cloner le Repository

```bash
git clone https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git
cd ece-ros2-lidar
```

### 2️⃣ Installer les Dépendances (Optionnel - Automatique)

```bash
chmod +x setup.sh
./setup.sh
```

**Ou installer manuellement:**

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop ros-humble-rviz2 -y

# PlatformIO
pip install platformio

# Docker (pour l'agent microROS)
sudo apt install docker.io -y
docker pull microros/micro-ros-docker:humble
```

### 3️⃣ Compiler le Workspace ROS2

```bash
cd ros2_ece_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4️⃣ Lancer le Système (3 Terminaux)

#### Terminal 1: Agent microROS
```bash
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### Terminal 2: Listener ROS2
```bash
cd ros2_ece_ws
source install/setup.bash
ros2 run cpp_pubsub listener
```

#### Terminal 3: RViz Visualisation
```bash
cd ros2_ece_ws
source install/setup.bash
ros2 launch rviz2 rviz2 -d lidar_config.rviz
```

## 🔌 Uploading Firmware ESP32

```bash
cd microROS_ece_pl
platformio run --target upload

# Ou via VS Code:
# 1. Ouvrir le dossier microROS_ece_pl
# 2. Installer l'extension PlatformIO
# 3. Cliquer "Upload" dans la barre de statut
```

## ✅ Vérification

Vous devriez voir:
- ✅ Terminal 1: Agent connecté et attentif
- ✅ Terminal 2: Messages LIDAR publiés (720 points)
- ✅ Terminal 3: Visualisation LIDAR en temps réel dans RViz

## 📚 Documentation

- [README.md](README.md) - Overview complet
- [GITHUB_GUIDE.md](GITHUB_GUIDE.md) - Guide GitHub détaillé
- [ros2_ece_ws/QUICKSTART.md](ros2_ece_ws/QUICKSTART.md) - ROS2 workflow
- [ros2_ece_ws/GUIDE_LIDAR_360_COMPLET.md](ros2_ece_ws/GUIDE_LIDAR_360_COMPLET.md) - Détails techniques

## 🐛 Problèmes?

```bash
# Vérifier que ROS2 est sourcé
echo $ROS_DISTRO  # Devrait afficher: humble

# Vérifier les topics
ros2 topic list

# Voir les nœuds actifs
ros2 node list

# Voir les messages en direct
ros2 topic echo /scan
```

---

**Prêt!** Vous avez maintenant le projet complet en local. 🎉
