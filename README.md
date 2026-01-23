# ECE ROS2 & microROS Lidar 360° Project

Projet complet de visualisation LIDAR 360° intégrant ROS2, microROS et une carte ESP32.

## 📁 Structure du Projet

```
ros2_ece_ws/                      # Workspace ROS2 principal
├── src/
│   ├── cpp_pubsub/               # Node C++ publication/souscription
│   └── vehicle_description/      # Description du véhicule URDF
├── build/                        # Artefacts compilés
├── install/                      # Packages installés
└── [Documentation et scripts]

microROS_ece_pl/                  # Firmware ESP32 microROS
├── src/main.cpp                  # Code principal
├── platformio.ini                # Configuration PlatformIO
└── [Dépendances et libs]
```

## 🎯 Fonctionnalités

✅ **LIDAR 360° circulaire** - Buffer circulaire ESP32 (720 points)  
✅ **Publication LaserScan ROS2** - Format standard compatible  
✅ **Visualisation RViz** - Configuration pré-optimisée  
✅ **Communication microROS** - Via UDP sur serial  
✅ **Documentation complète** - Guides détaillés intégrés  

## ⚡ Démarrage Rapide

### Prérequis

```bash
# ROS2 Humble (Ubuntu 22.04)
curl https://repo.ros2.org/ros.key | sudo apt-key add -
sudo apt update && sudo apt install ros-humble-desktop

# PlatformIO CLI
pip install platformio

# Docker (pour l'agent microROS)
sudo apt install docker.io
```

### 1. Lancer l'agent microROS (Terminal 1)

```bash
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. Configurer et compiler le workspace ROS2 (Terminal 2)

```bash
cd ros2_ece_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cpp_pubsub vehicle_description
source install/setup.bash
```

### 3. Téléverser le firmware ESP32 (Terminal 3)

```bash
cd microROS_ece_pl
platformio run --target upload
# Ou ouvrir le dossier dans VS Code + PlatformIO extension
```

### 4. Lancer les nœuds ROS2 (Terminal 2)

```bash
# Lancer le listener avec visualisation stats
ros2 run cpp_pubsub listener

# Ou visualiser dans RViz dans un autre terminal
ros2 launch rviz2 rviz2 -d ros2_ece_ws/lidar_config.rviz
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [00_SYNTHESE_FINALE.md](ros2_ece_ws/00_SYNTHESE_FINALE.md) | Synthèse complète du projet |
| [QUICKSTART.md](ros2_ece_ws/QUICKSTART.md) | Démarrage rapide en 2 minutes |
| [GUIDE_LIDAR_360_COMPLET.md](ros2_ece_ws/GUIDE_LIDAR_360_COMPLET.md) | Guide technique LIDAR |
| [GUIDE_RVIZ_CONFIGURATION.md](ros2_ece_ws/GUIDE_RVIZ_CONFIGURATION.md) | Configuration RViz détaillée |
| [MODIFICATIONS_SUMMARY.md](ros2_ece_ws/MODIFICATIONS_SUMMARY.md) | Historique des modifications |

## 🔧 Architecture

```
┌─────────────────────────────────────────────────────────┐
│               ROS2 Workspace (Ubuntu)                    │
│  ┌────────────────────────────────────────────────────┐  │
│  │  cpp_pubsub (Listener + Stats)                     │  │
│  │  vehicle_description (URDF)                        │  │
│  │  RViz (Visualisation)                              │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────┬──────────────────────────────────────┘
                   │ LaserScan ROS2 (UDP)
┌──────────────────▼──────────────────────────────────────┐
│         microROS Agent (Docker)                          │
│         Port: 8888                                       │
└──────────────────┬──────────────────────────────────────┘
                   │ micro-XRCE-DDS
┌──────────────────▼──────────────────────────────────────┐
│          ESP32 + LIDAR (microROS)                        │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Firmware PlatformIO                               │  │
│  │  Buffer circulaire 720 points                      │  │
│  │  Publication LaserScan                             │  │
│  └────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## 📦 Packages ROS2

- **cpp_pubsub** - Publication/souscription LaserScan
- **vehicle_description** - Description URDF du véhicule

## 🎮 Scripts Utiles

```bash
# Compiler et lancer
./run_lidar.sh          # Lance listener + stats
./run_rviz.sh           # Lance RViz avec config

# Tests
./test_lidar.sh         # Test de communication
./visualize_lidar.py    # Visualisation Python basique

# Configuration
./setup_rviz_display.sh # Configure RViz automatiquement
./deploy_lidar.sh       # Déploie complètement le système
```

## 📊 Spécifications Techniques

| Paramètre | Valeur |
|-----------|--------|
| Résolution LIDAR | 360° / 0.5° = 720 points |
| Fréquence de publication | ~10 Hz |
| Format | LaserScan standard ROS2 |
| Communication | UDP (port 8888) |
| Plateforme | ESP32 |
| Firmware | Arduino + microROS + PlatformIO |

## 🐛 Troubleshooting

### L'agent microROS ne se lance pas
```bash
docker pull microros/micro-ros-docker:humble
docker run -it --rm microros/micro-ros-docker:humble bash
```

### Pas de données LIDAR
1. Vérifier la connexion USB
2. Vérifier le port série: `ls /dev/tty*`
3. Relancer PlatformIO upload

### RViz affiche les données incorrectement
1. Vérifier le frame "lidar" dans la configuration RViz
2. Consulter [GUIDE_RVIZ_CONFIGURATION.md](ros2_ece_ws/GUIDE_RVIZ_CONFIGURATION.md)

## 👤 Auteur

Matheo - Projet ECE ROS2

## 📝 Licence

MIT License

## 🔗 Ressources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [microROS Documentation](https://micro.ros.org/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [RViz User Guide](https://wiki.ros.org/rviz)

---

**Dernière mise à jour:** Janvier 2026
