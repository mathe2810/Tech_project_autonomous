# 🚀 Guide Complet - ROS2 + LIDAR + ESP32 + micro_ros

## 📋 Vue d'Ensemble Complète

```
┌─────────────────────────────────────────────────────────────┐
│                   SYSTÈME COMPLET LIDAR                     │
├──────────────────────┬──────────────────┬──────────────────┤
│   ESP32 + LD06       │   ROS2 Humble    │   Visualisation  │
│                      │                  │                  │
│  ┌────────────────┐  │  ┌────────────┐  │  ┌────────────┐  │
│  │ LIDAR LD06     │  │  │ Publisher  │  │  │   RViz2    │  │
│  │ (230400 baud)  ├──┼─►│ talker     ├──┼─►│ LaserScan  │  │
│  │                │  │  │            │  │  │ Viewer     │  │
│  └────────────────┘  │  └────────────┘  │  └────────────┘  │
│         │            │         │        │         │        │
│  ┌────────────────┐  │  ┌────────────┐  │         │        │
│  │ micro_ros      │  │  │ Subscriber │  │         │        │
│  │ WiFi Agent     ├──┼─►│ listener   ├──┼─► Console Log   │
│  │ UDP:8888       │  │  │ (Analysis) │  │                 │
│  └────────────────┘  │  └────────────┘  │         │        │
└──────────────────────┴──────────────────┴─────────────────┘
         │                   │                      │
         └───────────────────┴──────────────────────┘
              ROS2 Topics: /scan, /data
```

## ⚡ 5 Minutes Quick Start

### Étape 1: Compiler

```bash
cd ~/ros_test/ros2_ece_ws
colcon build --packages-select cpp_pubsub
source install/setup.bash
```

### Étape 2: Lancer les nodes

**Terminal 1 - Publisher:**
```bash
ros2 run cpp_pubsub talker
```

**Terminal 2 - Subscriber:**
```bash
ros2 run cpp_pubsub listener
```

### Étape 3: Voir les données

```bash
# Terminal 3 - Console output du Subscriber
ros2 topic echo /scan --max-count=3
```

## 📊 Architecture en Détail

### Components

#### 1. **talker** (Publisher)
- **Executable:** `cpp_pubsub/talker`
- **Topics publiés:** `/scan` (LaserScan), `/data` (Int32)
- **Fréquence:** 10 Hz
- **Données générées:** Simulations LIDAR pour test

**Fichier:** `src/publisher_member_function.cpp`

```cpp
// Créer Publishers
lidar_publisher_ = create_publisher<LaserScan>("/scan", 10);
counter_publisher_ = create_publisher<Int32>("/data", 10);

// Timer callback toutes les 100ms
timer_ = create_wall_timer(100ms, timer_callback);
```

#### 2. **listener** (Subscriber)
- **Executable:** `cpp_pubsub/listener`
- **Topics abonnés:** `/scan`, `/data`
- **Traitement:** Analyse + Affichage
- **Sortie:** Logs console détaillés

**Fichier:** `src/subscriber_member_function.cpp`

**Analyse effectuée:**
- ✓ Détection obstacle le plus proche
- ✓ Alerte obstacle devant (±15°, <1m)
- ✓ Statistiques distances
- ✓ Debug des premiers points

#### 3. **Configuration ROS2**
- **Package:** `cpp_pubsub`
- **Dependencies:** rclcpp, sensor_msgs, std_msgs
- **Build system:** ament_cmake

## 🔌 Topics ROS2

### `/scan` - sensor_msgs/LaserScan

**Format:**
```yaml
header:
  stamp:
    sec: 1705715400
    nanosec: 500000000
  frame_id: "lidar_link"

angle_min: 0.0           # radians
angle_max: 6.28318       # 2π radians (360°)
angle_increment: 0.01745 # ~1 degré

time_increment: 0.0
scan_time: 0.1           # 100ms
range_min: 0.06          # 60mm
range_max: 12.0          # 12m

ranges: [5.23, 5.15, 4.89, ...]  # Distances en mètres
intensities: [100, 120, 95, ...]  # Force du signal
```

### `/data` - std_msgs/Int32

**Format:**
```yaml
data: 42  # Compteur simple
```

## 🚀 Options de Lancement

### 1. Simple (Fichiers individuels)

```bash
# Terminal 1
ros2 run cpp_pubsub talker

# Terminal 2
ros2 run cpp_pubsub listener
```

### 2. Launch File

```bash
ros2 launch cpp_pubsub lidar_launch.py
```

**Fichier:** `launch/lidar_launch.py`
- Lance talker + listener
- Même terminal

### 3. Complet (avec RViz)

```bash
ros2 launch cpp_pubsub lidar_complete_launch.py
```

**Fichier:** `launch/lidar_complete_launch.py`
- Lance talker + listener + rviz2
- Chaque node dans son terminal

### 4. Avec Agent micro_ros (ESP32)

```bash
# Terminal 1 - Agent
docker run -it --rm \
  -v /dev:/dev --privileged \
  microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --ip 0.0.0.0 --port 8888

# Terminal 2 - Nodes ROS2
ros2 launch cpp_pubsub agent_launch.py

# Terminal 3 - Monitor
ros2 topic echo /scan
```

## 📈 Analyse des Données (Subscriber)

Le subscriber affiche automatiquement:

```
════════════════════════════════════════════
🔍 SCAN LIDAR #1 reçu
Frame ID: lidar_link
Nombre de points: 360
Angle Min/Max: 0.00 / 6.28 rad (0.0° / 360.0°)
Angle Increment: 0.0175 rad (1.00°)
Range Min/Max: 0.06 / 12.00 m
════════════════════════════════════════════

⚠️  Obstacle le plus proche: 0.52 m @ indice 45 (angle: 45.0°)
📈 Statistiques - Points valides: 360 / 360, Distance moyenne: 5.32 m
```

**Informations clés:**

1. **Obstale global:** Point le plus proche
2. **Danger avant:** Si <1m dans cône ±15°
3. **Statistiques:** Points valides + moyenne
4. **Debug:** Premiers 5 points (si ROS_LOG_LEVEL=DEBUG)

## 🎨 Visualisation RViz

### Lancer RViz

```bash
# Simple
rviz2

# Avec config fournie
rviz2 -d ~/ros_test/ros2_ece_ws/src/cpp_pubsub/launch/lidar.rviz
```

### Configuration

1. **Fixed Frame:** `lidar_link`
2. **Display:** LaserScan
3. **Topic:** `/scan`
4. **Color:** Gradient (intensité)

## 🔗 Intégration avec ESP32

### Architecture Complète

```
ESP32 (micro_ros client)
  │
  ├─ UART (GPIO16) ← LIDAR LD06
  │
  └─ WiFi
      │
      └─ UDP8888 (agent micro_ros)
          │
          └─ ROS2 Network
              │
              ├─ /scan ──► talker (Publisher)
              │             │
              │             └─► listener (Subscriber)
              │                   │
              │                   └─► Console + RViz
              │
              └─ /data ──► talker (Int32)
```

### Étapes de Déploiement

#### 1. Configurer ESP32
```cpp
// Dans microROS_ece_pl/src/main.cpp

#define LIDAR_ENABLED 1
#define LIDAR_RX 16
#define LIDAR_BAUD 230400

// Publier sur /scan (LaserScan)
rcl_publisher_t pub_lidar;
// Publier sur /data (Int32)
rcl_publisher_t pub_int;
```

#### 2. Upload vers ESP32
```bash
cd ~/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
pio run -e esp32dev -t upload
```

#### 3. Lancer agent
```bash
docker run -it --rm \
  -v /dev:/dev --privileged \
  microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --ip 0.0.0.0 --port 8888
```

#### 4. Vérifier connexion
```bash
# Voir tous les topics
ros2 topic list

# Doit afficher:
# /scan
# /data
```

#### 5. Lancer les nodes
```bash
ros2 launch cpp_pubsub agent_launch.py
```

## 🛠️ Utilité Script Helper

```bash
# Lancer le script
~/ros_test/ros2_ece_ws/run_lidar.sh [OPTION]

# Options
publisher       # Lancer talker uniquement
subscriber      # Lancer listener uniquement
launch          # Lancer avec launch file
launch-complete # Lancer complet (RViz)
agent           # Lancer pour recevoir ESP32
monitor         # Voir les topics
build           # Recompiler
clean           # Nettoyer la compilation
```

## 📊 Monitoring et Debug

### Topics en Live

```bash
# Lister tous les topics
ros2 topic list

# Info détaillée
ros2 topic info /scan

# Fréquence
ros2 topic hz /scan

# BW utilisée
ros2 topic bw /scan
```

### Logs Détaillés

```bash
# Niveau DEBUG
ROS_LOG_LEVEL=DEBUG ros2 run cpp_pubsub listener

# Voir les niveaux de log
# DEBUG, INFO, WARN, ERROR, FATAL
```

### Enregistrer les données

```bash
# Enregistrer en rosbag
ros2 bag record /scan /data

# Rejouer
ros2 bag play rosbag2_*
```

## 📁 Structure Complète

```
ros_test/ros2_ece_ws/
├── src/
│   └── cpp_pubsub/
│       ├── src/
│       │   ├── publisher_member_function.cpp     ← talker
│       │   ├── subscriber_member_function.cpp    ← listener
│       │   └── ultrasonic_processor.cpp
│       │
│       ├── launch/
│       │   ├── lidar_launch.py                   ← Simple
│       │   ├── lidar_complete_launch.py          ← Complet
│       │   ├── agent_launch.py                   ← Avec agent
│       │   └── lidar.rviz                        ← Config RViz
│       │
│       ├── CMakeLists.txt                        ← Build config
│       ├── package.xml                           ← Dependencies
│       └── README.md                             ← Documentation
│
├── run_lidar.sh                                   ← Script helper
└── install/
    └── setup.bash                                 ← Source ceci!
```

## 🎓 Concepts Utilisés

1. **ROS2 Nodes:** talker (pub) + listener (sub)
2. **Publishers/Subscribers:** Communication asynchrone
3. **Topics:** /scan (LaserScan), /data (Int32)
4. **Messages:** sensor_msgs/LaserScan format standard
5. **Launch Files:** Orchestration automatique
6. **RViz:** Visualisation 3D
7. **micro_ros:** Communication ESP32 ↔ ROS2

## 🔄 Flux de Données Complet

```
1. ESP32 (ou talker simul)
   │
   ├─ Génère données LIDAR
   └─ Publie sur /scan + /data
   
2. ROS2 Network
   │
   ├─ Reçoit messages
   └─ Route vers subscribers
   
3. Subscriber (listener)
   │
   ├─ Reçoit /scan
   ├─ Reçoit /data
   ├─ Analyse données
   └─ Affiche résultats
   
4. Visualisation (RViz)
   │
   ├─ Affiche points LIDAR
   ├─ Grid + obstacles
   └─ Temps réel
```

## 🚀 Prochaines Étapes

1. **Enregistrement:** `ros2 bag record /scan`
2. **Traitement:** Créer node processing personnalisé
3. **Évitement:** Implémenter logique obstacle
4. **Navigation:** Intégrer avec nav2_core
5. **SLAM:** Ajouter cartographie autonome

## 📞 Troubleshooting Rapide

| Problème | Solution |
|----------|----------|
| Topics non visibles | `source install/setup.bash` |
| RViz vide | Fixed Frame = "lidar_link" |
| Compilation échoue | `rosdep install --from-paths src` |
| Pas de connexion ESP32 | Vérifier agent + WiFi |
| Données manquantes | Vérifier /scan + /data topics |

---

**📌 Fichier Important:** [README.md](./cpp_pubsub/README.md)  
**📌 Script Helper:** `/home/matheo/ros_test/ros2_ece_ws/run_lidar.sh`  
**📌 Configuration RViz:** `launch/lidar.rviz`

**Auteur:** GitHub Copilot  
**Date:** 2026-01-19  
**Version:** 1.0.0
