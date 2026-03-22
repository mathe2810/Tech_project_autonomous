# 🚀 Quick Start

Démarrage rapide pour **simulation** et **robot réel**.

---

## ⚡ Option 1 : Simulation (2 minutes)

### Setup
```bash
# 1. Activer l'environnement Python
source .venv/bin/activate

# 2. Sourcer ROS2
source /opt/ros/humble/setup.bash

# 3. Lancer le stack simulation
./start_auto_slam_only.sh
```

### Contrôles
- **Pygame window** (le simulateur)
  - `A` : Active/désactive l'autonomie
  - `↑↓←→` : Commandes manuelles
  - `Q` : Quitter

### Visualisation (optionnel)
```bash
# Terminal 2
rviz2
```
- Fixed Frame : `map`
- Ajouter layers : LaserScan (`/scan`), OccupancyGrid (`/map`)

---

## ⚡ Option 2 : Robot Réel

### Pré-requis
✓ ESP32 programmé avec firmware (`platformio.ini`)
✓ LIDAR connecté (GPIO16 RX, GPIO25 PWM)
✓ connexion USB ou WiFi fonctionnelle

### Démarrage
```bash
# 1. Environnement
source .venv/bin/activate
source /opt/ros/humble/setup.bash

# 2. Lancer le stack
./start_robot_mapping_auto.sh

# Le script attendra /scan_raw pendant 15s
# L'ESP32 doit publier rapidement
```

### Vérification
```bash
# Terminal 2 - Vérifier les topics
ros2 topic list | grep -E 'scan|odom|map'

# Écouter les scans
ros2 topic echo /scan --max-count=1

# Monitorer la carte
ros2 topic echo /map --max-count=1
```

---

## 📋 Checklist

### Simulation
- [ ] Python venv activé
- [ ] ROS2 sourced
- [ ] Pygame window apparaît
- [ ] Topics `/scan`, `/odom` publient (vérifier avec `ros2 topic list`)

### Robot Réel
- [ ] ESP32 programmé et connecté
- [ ] LIDAR allumé et connecté
- [ ] `/scan_raw` publié par l'ESP32 (vérifier : `ros2 topic list`)
- [ ] Script attend puis démarre SLAM

---

## 🔧 Arrêter le Stack

```bash
# Option 1 : Ctrl+C dans le terminal principal

# Option 2 : Tuer tous les processus
pkill -9 -f 'simu_|slam|restamper|transform'

# Option 3 : Spécifique
pkill -f slam_toolbox
pkill -f scan_restamper
pkill -f motor_odom_simple
```

---

## 🐛 Problèmes Courants

### "QUICKSTART: Command not found"
```bash
# Déterminer le chemin correct
pwd
ls start_auto_slam_only.sh

# Ou utiliser le chemin absolu
/home/matheo/microros_ece_ws/src/.../start_auto_slam_only.sh
```

### Simulation : "No module named pygame"
```bash
source .venv/bin/activate
pip install pygame
```

### Robot : "/scan_raw timeout"
```bash
# Vérifier ESP32
ros2 topic list | grep scan

# Si absent:
# 1. Vérifier connexion USB
# 2. Vérifier baud rate (généralement 115200)
# 3. Vérifier que l'ESP32 exécute le firmware
```

### SLAM ne démarre pas
```bash
# Vérifier installation slam_toolbox
ros2 pkg list | grep slam

# Réinstaller si absent
sudo apt install ros-humble-slam-toolbox
```

---

## 📚 Aller Plus Loin

- **Simulation avancée** : Éditer `simu_lidar_auto.py` pour modifier le circuit
- **Robot autonome** : Décommenter `wall_centering_node.py` dans `start_robot_mapping_auto.sh`
- **Configuration SLAM** : Modifier `config/slam_corridor_slam_only.yaml` ou `config/slam_toolbox_minimal.yaml`

---

**Voir `README.md` pour la documentation complète.**
