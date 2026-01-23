# 📊 ROS2 LIDAR Publisher/Subscriber

## 🎯 Vue d'ensemble

Ce package ROS2 implémente une architecture complète pour traiter les données LIDAR:
- **Publisher** : Publie les données LIDAR sur `/scan` (sensor_msgs/LaserScan)
- **Subscriber** : Reçoit et affiche les données avec analyse
- **Topics** : `/scan` (LaserScan) + `/data` (Int32 counter)

## 📦 Structure du Package

```
cpp_pubsub/
├── src/
│   ├── publisher_member_function.cpp   # LIDAR Publisher
│   ├── subscriber_member_function.cpp  # LIDAR Subscriber
│   └── ultrasonic_processor.cpp        # Processor additionnel
├── launch/
│   ├── lidar_launch.py                 # Launch simple
│   ├── lidar_complete_launch.py        # Launch complet (RViz)
│   ├── agent_launch.py                 # Launch avec agent
│   └── lidar.rviz                      # Config RViz
├── CMakeLists.txt                      # Build config
└── package.xml                         # Dependencies
```

## 🚀 Installation et Build

### 1. Compiler le package

```bash
cd ~/ros_test/ros2_ece_ws
colcon build --packages-select cpp_pubsub

# Ou avec logs verbeux
colcon build --packages-select cpp_pubsub --symlink-install
```

### 2. Sourcer l'environnement

```bash
source install/setup.bash
```

## 📡 Exécution

### Option 1 : Lancer les nodes individuellement

**Terminal 1 - Publisher (LIDAR):**
```bash
ros2 run cpp_pubsub talker
```

**Terminal 2 - Subscriber (Console):**
```bash
ros2 run cpp_pubsub listener
```

**Terminal 3 - Vérifier les topics:**
```bash
ros2 topic list          # Lister tous les topics
ros2 topic info /scan    # Info sur /scan
ros2 topic hz /scan      # Fréquence de publication
```

### Option 2 : Lancer avec le launch file

```bash
# Simple (Publisher + Subscriber)
ros2 launch cpp_pubsub lidar_launch.py

# Complet (Publisher + Subscriber + RViz)
ros2 launch cpp_pubsub lidar_complete_launch.py

# Avec agent micro_ros
ros2 launch cpp_pubsub agent_launch.py
```

## 🔌 Topics ROS2

### `/scan` - sensor_msgs/LaserScan
**Publié par:** `talker` (Publisher)

Données LIDAR avec:
- Header (frame_id, timestamp)
- Angles (min, max, increment)
- Ranges (distances en mètres)
- Intensities (forces du signal)

**Exemple:**
```bash
ros2 topic echo /scan --max-count=3
```

### `/data` - std_msgs/Int32
**Publié par:** `talker` (Publisher)

Compteur simple pour test synchronisation

**Exemple:**
```bash
ros2 topic echo /data
```

## 📊 Données Affichées (Subscriber)

Le subscriber affiche automatiquement:

1. **Info générale du scan:**
   - ID du frame
   - Nombre de points
   - Plage d'angles
   - Plage de distances

2. **Obstacle le plus proche:**
   - Distance minimum
   - Index et angle

3. **Danger devant:**
   - Si obstacle < 1m dans cône frontal ±15°

4. **Statistiques:**
   - Nombre de points valides
   - Distance moyenne

5. **Debug info:**
   - Premiers points (si niveau DEBUG)

## 🎨 Visualisation RViz

### Lancer RViz seul

```bash
rviz2
```

### Configurer RViz

1. **Fixed Frame:** Changer en `lidar_link`
2. **Add Display:** Clic "Add" en bas à gauche
3. **Select LaserScan:** Chercher dans la liste
4. **Topic:** Sélectionner `/scan`

### Utiliser la config fournie

```bash
rviz2 -d ~/ros_test/ros2_ece_ws/src/cpp_pubsub/launch/lidar.rviz
```

## 🔗 Intégration ESP32 + micro_ros

### Terminal 1 - Démarrer l'agent

```bash
docker run -it --rm \
  -v /dev:/dev --privileged \
  microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --ip 0.0.0.0 --port 8888
```

### Terminal 2 - Lancer les nodes ROS2

```bash
ros2 launch cpp_pubsub agent_launch.py
```

### Terminal 3 - Vérifier la connexion

```bash
# Lister les clients connectés
ros2 topic list

# Écouter les données ESP32
ros2 topic echo /scan
ros2 topic echo /data
```

## 📈 Topics Disponibles (avec ESP32)

### De l'ESP32 (Agent micro_ros):
- `/scan` - Données LIDAR du capteur LD06
- `/data` - Compteur de test

### De ROS2 (Nodes locaux):
- `/scan` - Données LIDAR publiées par `talker`
- `/data` - Compteur publié par `talker`

## 🛠️ Debug et Troubleshooting

### Vérifier les dépendances

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Compiler en debug

```bash
colcon build --packages-select cpp_pubsub --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Augmenter le niveau de log

```bash
# Terminal 1
ROS_LOG_LEVEL=DEBUG ros2 run cpp_pubsub talker

# Terminal 2
ROS_LOG_LEVEL=DEBUG ros2 run cpp_pubsub listener
```

### Vérifier les erreurs de compilation

```bash
colcon build --packages-select cpp_pubsub --event-handlers console_cohesion+
```

## 📚 Format des Messages

### LaserScan
```cpp
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id

float32 angle_min          # Angle min (rad)
float32 angle_max          # Angle max (rad)
float32 angle_increment    # Incrément angle (rad)
float32 time_increment     # Temps entre points
float32 scan_time          # Durée du scan
float32 range_min          # Distance min valide
float32 range_max          # Distance max valide
float32[] ranges           # Distances (m)
float32[] intensities      # Forces du signal
```

### Int32
```cpp
int32 data  # Valeur du compteur
```

## 🔄 Pipeline de Données

```
ESP32 (LIDAR LD06)
  │
  └─ micro_ros Agent
      │
      └─ ROS2 Network
          │
          ├─ /scan (LaserScan)
          │   └─ Subscriber displays data
          │   └─ RViz visualization
          │
          └─ /data (Int32 counter)
              └─ Subscriber displays counter
```

## 🎯 Cas d'Utilisation

1. **Visualisation Simple:**
   ```bash
   ros2 run cpp_pubsub talker &
   ros2 run cpp_pubsub listener
   ```

2. **Visualisation RViz:**
   ```bash
   ros2 run cpp_pubsub talker &
   rviz2
   ```

3. **Avec ESP32:**
   ```bash
   # Terminal 1: Agent
   docker run ... micro_ros_agent ...
   
   # Terminal 2: Nodes ROS2
   ros2 launch cpp_pubsub agent_launch.py
   
   # Terminal 3: RViz
   rviz2 -d ... lidar.rviz
   ```

## 📝 Prochaines Étapes

- [ ] Enregistrer les données LIDAR (rosbag)
- [ ] Implémenter filtre Kalman
- [ ] Ajouter node d'évitement obstacle
- [ ] Créer service ROS pour configuration
- [ ] Intégrer navigation autonome

## 🐛 Problèmes Courants

| Problème | Solution |
|----------|----------|
| Topics non affichés | `colcon build` et `source install/setup.bash` |
| RViz ne montre rien | Vérifier frame_id = "lidar_link" |
| Pas de données ESP32 | Vérifier connexion WiFi + agent UDP |
| Compilation échoue | `rosdep install --from-paths src` |

---

**Auteur:** GitHub Copilot  
**Date:** 2026-01-19  
**ROS Version:** Humble  
**Python:** 3.10+
