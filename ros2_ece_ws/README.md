# ROS2 Workspace - LIDAR 360° Visualisation

Workspace ROS2 complet pour la visualisation et le traitement des données LIDAR 360°.

## 📁 Structure

```
ros2_ece_ws/
├── src/
│   ├── cpp_pubsub/               # Package de publication/souscription
│   │   ├── src/
│   │   │   ├── publisher_member_function.cpp
│   │   │   ├── listener_member_function.cpp
│   │   │   └── [autres sources]
│   │   └── CMakeLists.txt
│   │
│   └── vehicle_description/      # Description URDF du véhicule
│       ├── urdf/
│       ├── meshes/
│       └── CMakeLists.txt
│
├── build/                        # Artefacts compilés (généré par colcon)
├── install/                      # Packages installés (généré par colcon)
├── log/                          # Logs de compilation (généré par colcon)
│
├── lidar_config.rviz             # Configuration RViz pré-optimisée
├── QUICKSTART.md                 # Guide démarrage rapide
├── 00_SYNTHESE_FINALE.md         # Synthèse complète du projet
├── GUIDE_LIDAR_360_COMPLET.md    # Guide technique détaillé
├── GUIDE_RVIZ_CONFIGURATION.md   # Configuration RViz expliquée
│
├── run_lidar.sh                  # Script lancement listener
├── run_rviz.sh                   # Script lancement RViz
├── deploy_lidar.sh               # Déploiement complet
├── setup_rviz_display.sh         # Configuration RViz auto
├── test_lidar.sh                 # Tests de communication
│
├── lidar_simple_plot.py          # Visualisation Python simple
└── visualize_lidar.py            # Visualisation Python avancée
```

## 🎯 Packages ROS2

### cpp_pubsub

**Description**: Publication/souscription des messages LaserScan

**Nœuds:**
- `listener` - Souscrit à `/scan` et affiche les statistiques
- `publisher` - Publie des données de test

**Topics:**
- `/scan` - Données LIDAR (LaserScan)

**Files:**
- `listener_member_function.cpp` - Code du listener
- `publisher_member_function.cpp` - Code du publisher
- `CMakeLists.txt` - Configuration CMake

### vehicle_description

**Description**: Description URDF du véhicule pour RViz

**Fichiers:**
- `urdf/vehicle.urdf` - Modèle URDF principal
- `meshes/` - Ressources 3D (optionnel)

## ⚙️ Installation & Compilation

### Prérequis

```bash
# ROS2 Humble (Ubuntu 22.04)
curl https://repo.ros2.org/ros.key | sudo apt-key add -
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Dépendances optionnelles
sudo apt install ros-humble-rviz2
```

### Compilation

```bash
# Se placer dans le workspace
cd ros2_ece_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Compiler tous les packages
colcon build

# Ou compiler un package spécifique
colcon build --packages-select cpp_pubsub

# Compiler avec verbose
colcon build --event-handlers console_direct+

# Source du workspace
source install/setup.bash
```

### Vérifier l'installation

```bash
# Lister les packages
ros2 pkg list | grep cpp_pubsub

# Lister les exécutables
ros2 pkg executables cpp_pubsub
```

## 🚀 Lancement

### Démarrage complet (3 terminaux)

**Terminal 1: Agent microROS**
```bash
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2: Workspace ROS2 + Listener**
```bash
cd ros2_ece_ws
source install/setup.bash
ros2 run cpp_pubsub listener
```

**Terminal 3: RViz Visualisation**
```bash
cd ros2_ece_ws
source install/setup.bash
ros2 launch rviz2 rviz2 -d lidar_config.rviz
```

### Utiliser les scripts

```bash
# Lancer listener avec statistiques
./run_lidar.sh

# Lancer RViz avec config optimisée
./run_rviz.sh

# Déploiement complet (tous les services)
./deploy_lidar.sh

# Tests de communication
./test_lidar.sh

# Configuration automatique RViz
./setup_rviz_display.sh
```

## 📊 Topics et Messages

### Topic: `/scan`

**Type**: `sensor_msgs/LaserScan`

**Description**: Données LIDAR brutes (360°)

**Structure**:
```
frame_id: "lidar"
angle_min: 0.0 (radians)
angle_max: 6.28 (2π radians)
angle_increment: 0.0087 (0.5°)
range_min: 0.0 (m)
range_max: 10.0 (m)
ranges: [float32 x 720]  # Distance pour chaque angle
intensities: [float32 x 720]  # Intensité (optionnel)
```

**Fréquence**: ~10 Hz

**Exemple de lecture**:
```bash
ros2 topic echo /scan
ros2 topic hz /scan
ros2 topic info /scan
```

## 🔧 Configuration RViz

### Éléments préconfigurés

Le fichier `lidar_config.rviz` inclut:

1. **LaserScan Display**
   - Topic: `/scan`
   - Color Transformer: Intensity
   - Min/Max Intensity: 0/255

2. **Global Options**
   - Fixed Frame: `lidar`
   - Background Color: Noir

3. **Camera**
   - Positionnement optimal pour visualiser le LIDAR

### Personnaliser RViz

```bash
# Ouvrir RViz et configurer manuellement
ros2 launch rviz2 rviz2

# Sauvegarder la configuration
# Menu: File > Save Config As > lidar_config_custom.rviz
```

Pour plus de détails, voir [GUIDE_RVIZ_CONFIGURATION.md](GUIDE_RVIZ_CONFIGURATION.md).

## 📈 Monitoring et Debugging

### Topics actifs

```bash
# Lister les topics
ros2 topic list

# Afficher les messages en temps réel
ros2 topic echo /scan

# Fréquence de publication
ros2 topic hz /scan

# Informations du topic
ros2 topic info /scan

# Bande passante utilisée
ros2 topic bw /scan
```

### Nœuds actifs

```bash
# Lister les nœuds
ros2 node list

# Infos sur un nœud
ros2 node info /listener

# Services disponibles
ros2 service list
```

### Logs et debugging

```bash
# Afficher les logs colcon
cat log/latest_build/cpp_pubsub/stdout.log

# Lancer avec debug verbeux
ros2 run cpp_pubsub listener --ros-args --log-level debug
```

## 🐍 Scripts Python

### `lidar_simple_plot.py`

Visualisation matplotlib simple des données LIDAR

```bash
python3 lidar_simple_plot.py
```

### `visualize_lidar.py`

Visualisation avancée avec OpenGL/Pygame

```bash
python3 visualize_lidar.py
```

## 🧹 Cleaning

```bash
# Supprimer les artefacts de compilation
rm -rf build/ install/ log/

# Nettoyer complètement
colcon clean all
```

## 🔄 Mise à jour

```bash
# Pull latest changes
git pull

# Recompiler
colcon build

# Source nouvelle installation
source install/setup.bash
```

## 📝 CMakeLists.txt (Structure Standard)

Chaque package contient un `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(listener src/listener_member_function.cpp)
ament_target_dependencies(listener rclcpp sensor_msgs)

install(TARGETS listener DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## 🚨 Troubleshooting

### "Package not found"
```bash
source install/setup.bash
```

### RViz ne se lance pas
```bash
sudo apt install ros-humble-rviz2
```

### Pas de messages `/scan`
1. Vérifier que l'agent microROS est lancé
2. Vérifier que l'ESP32 publie (`rostopic echo /scan`)
3. Vérifier la connexion réseau

### Erreur de compilation CMake
```bash
colcon build --event-handlers console_direct+
# Voir le log complet pour les erreurs
```

## 📚 Ressources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [LaserScan Message Format](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/LaserScan.html)
- [RViz2 Documentation](https://docs.ros.org/en/humble/p/rviz2/index.html)
- [Colcon Build Documentation](https://colcon.readthedocs.io/)

## 📖 Documentation Locale

| Document | Contenu |
|----------|---------|
| [QUICKSTART.md](QUICKSTART.md) | Démarrage rapide (2 min) |
| [00_SYNTHESE_FINALE.md](00_SYNTHESE_FINALE.md) | Synthèse projet complète |
| [GUIDE_LIDAR_360_COMPLET.md](GUIDE_LIDAR_360_COMPLET.md) | Guide technique détaillé |
| [GUIDE_RVIZ_CONFIGURATION.md](GUIDE_RVIZ_CONFIGURATION.md) | Configuration RViz expliqu ée |
| [MODIFICATIONS_SUMMARY.md](MODIFICATIONS_SUMMARY.md) | Historique modifications |

## 🏃 Quick Commands

```bash
# Build
colcon build

# Build + Install in one go
colcon build && source install/setup.bash

# Run listener
ros2 run cpp_pubsub listener

# Run RViz
ros2 launch rviz2 rviz2 -d lidar_config.rviz

# Echo topic
ros2 topic echo /scan

# Check topic frequency
ros2 topic hz /scan

# List all topics
ros2 topic list

# List all nodes
ros2 node list
```

---

**Dernière mise à jour:** Janvier 2026  
**Version ROS**: Humble  
**Protocole**: microROS via UDP  
**Plateforme**: ESP32 + LIDAR 360°
