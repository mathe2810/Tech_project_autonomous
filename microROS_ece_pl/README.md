# MicroROS ECE - Navigation Autonome avec SLAM

Stack de localisation et mapping autonome basé sur **ROS2 Humble** + **SLAM Toolbox**.

Deux modes de fonctionnement :
- **Simulation 2D** : Environnement contrôlé pour tester l'autonomie
- **Robot Réel** : Mapping pur via scan matching sur hardware ESP32

---

## 🎯 Modes de Fonctionnement

### 1️⃣ Simulation (start_auto_slam_only.sh)

Mode simulation complet avec robot en 2D et autonomie programmable.

**Démarrage :**
```bash
./start_auto_slam_only.sh
```

**Composants activés :**
- 🤖 Simulateur LIDAR 2D (`simu_lidar_auto.py`)
- 🌉 Bridge ROS2 : scan LIDAR + odométrie ground truth (`simu_bridge.py`)
- 🗺️ SLAM Toolbox en mode `async_slam_toolbox_node`
- 📍 Setter de pose initiale depuis ground truth
- 💾 Sauvegarde des cartes en PNG
- 🧭 Transformer odométrie → TF2

**Contrôles :**
- `A` dans la fenêtre pygame : bascule mode autonome ON/OFF
- `Flèches ↑↓←→` : commandes manuelles (même en mode autonome)

**Configuration utilisée :**
```
config/slam_corridor_slam_only.yaml
```

---

### 2️⃣ Robot Réel (start_robot_mapping_auto.sh)

Stack minimal pour robot physique avec ESP32 + LIDAR.
**Mode pur scan matching** : SLAM ne dépend que des données LIDAR, pas de l'odométrie moteur.

**Démarrage :**
```bash
./start_robot_mapping_auto.sh
```

**Pré-requis :**
- ESP32 programmé (`platformio.ini`) publiant `/scan_raw`
- LIDAR connecté et fonctionnel

**Composants activés :**
1. TF statique `base_link` → `laser_link` (position LIDAR sur robot)
2. Restampage LIDAR `/scan_raw` → `/scan`
3. Attente topics (vérification présence `/scan_raw`)
4. Odométrie moteur minimale (covariance très élevée = priorité SLAM)
5. SLAM Toolbox en mode scan matching pur
6. Option : `wall_centering_node.py` pour autonomie

**Configuration utilisée :**
```
config/slam_toolbox_minimal.yaml
```

---

## 📁 Structure du Projet

```
├── start_auto_slam_only.sh              ← 🎮 Simulation
├── start_robot_mapping_auto.sh          ← 🤖 Robot réel
│
├── config/
│   ├── slam_corridor_slam_only.yaml     # SLAM simulation
│   └── slam_toolbox_minimal.yaml        # SLAM robot réel
│
├── Simulateur
│   ├── simu_lidar_auto.py               # Simulateur 2D + autonomie
│   └── simu_bridge.py                   # Bridge ROS2 pour simu
│
├── SLAM Utils
│   ├── slam_initial_pose_setter.py      # Init pose SLAM
│   ├── slam_map_saver.py                # Export cartes PNG
│   └── slam_odom_from_tf.py             # Odométrie depuis TF
│
├── Robot Réel
│   ├── scan_restamper_simple.py         # Restampage LIDAR
│   ├── motor_odom_simple.py             # Odométrie moteur
│   └── wall_centering_node.py           # Contrôle autonome (optionnel)
│
├── Utilitaires
│   ├── odom_to_tf.py                    # Odométrie → TF2
│   └── wait_for_topic.py                # Attente de topics
│
├── Infrastructure
│   ├── src/main.cpp                     # Firmware ESP32
│   ├── include/                         # Headers C++
│   ├── platformio.ini                   # Config PlatformIO
│   └── .venv/                           # Environnement Python
│
└── Docs
    ├── README.md                        # Ce fichier
    └── QUICKSTART.md                    # Démarrage rapide
```

---

## 🛠️ Dépendances

### ROS2 Packages (Humble)
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-tf2-ros
```

### Python 3.10+ (venv inclus)
```bash
source .venv/bin/activate
# Packages: rclpy, PyYAML, numpy, opencv-python, pygame (simu uniquement)
```

### Firmware ESP32 (optionnel)
- PlatformIO CLI
- Board : `esp32dev`
- Transport : UART ou WiFi

---

## ⚙️ Configuration

### Simulation (`slam_corridor_slam_only.yaml`)
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom_ground_truth        # Utilise ground truth
    base_frame: base_link
    use_scan_matching: true             # Scan matching actif
    do_loop_closure: false              # Pas de loop closure en simulation
```

### Robot Réel (`slam_toolbox_minimal.yaml`)
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom                    # Odométrie moteur (covariance haute)
    base_frame: base_link
    use_scan_matching: true             # 100% scan matching
    do_loop_closure: false              # Optionnel sur petit robot
```

---

## 🧹 Contrôle des Processus

### Arrêter tous les nœuds
```bash
# Simulation
pkill -9 -f 'simu_|slam|restamper|transform'

# Ou spécifique robot
pkill -f slam_toolbox
pkill -f scan_restamper_simple
pkill -f motor_odom_simple
```

### Logs en temps réel
```bash
# Voir les topics publiés
ros2 topic list

# Écouter un topic
ros2 topic echo /map
ros2 topic echo /scan
```

---

## 📊 Topics ROS2

| Topic | Type | Source | Utilisation |
|-------|------|--------|-------------|
| `/scan` | LaserScan | LIDAR (simu ou réel) | Entrée SLAM |
| `/odom` | Odometry | Motor/Simu (réel/simu) | Prior SLAM |
| `/map` | OccupancyGrid | SLAM Toolbox | Carte mappée |
| `/tf` | TF2 | SLAM/Odom | Transforms |
| `/pose` | PoseWithCovarianceStamped | SLAM | Position estimée |

---

## 🚀 Exemples de Démarrage

### Simulation seule (sans robot réel)
```bash
cd /path/to/project
./start_auto_slam_only.sh &     # Terminal 1

# Dans un autre terminal - visualiser
rviz2 &                         # Terminal 2
```

### Robot réel
```bash
# Assurez-vous que l'ESP32 publie /scan_raw en UART
./start_robot_mapping_auto.sh

# Monitorer les topics
ros2 topic echo /scan --max-count=1
ros2 topic echo /map
```

### Test synthétique sans hardware
```bash
# Simulation seulement (pas besoin de LIDAR physique)
source .venv/bin/activate
./start_auto_slam_only.sh
```

---

## 🐛 Dépannage

### **Erreur : "slam_toolbox package not found"**
```bash
source /opt/ros/humble/setup.bash
# Vérifier l'install
dpkg -l | grep slam-toolbox
```

### **Erreur : "/scan_raw timeout" (robot réel)**
```bash
# Vérifier que l'ESP32 publie
ros2 topic list | grep scan
# Si absent: vérifier connexion série, baud rate
```

### **Mode autonomie ne fonctionne pas (simulation)**
```bash
# Vérifier pygame window active
# Appuyer sur 'A' dans la fenêtre du simulateur
# Vérifier logs de démarrage
```

---

## 📖 Références

- **SLAM Toolbox** : https://github.com/SteveMacenski/slam_toolbox
- **ROS2 Humble** : https://docs.ros.org/en/humble/
- **micro-ROS** : https://micro.ros.org/ (pour ESP32 firmware)

---

**Dernière mise à jour :** Mars 2026  
**Auteur :** Équipe ECE - Tech Project Autonome
