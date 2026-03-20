# 🤖 REAL ROBOT AUTONOMOUS MAPPING - Architecture Complète

## Overview

Système complet de **mapping autonome en 3 phases** pour robot réel sur tapis :

```
┌─────────────────────────────────────────────────────────────┐
│ Phase 1: AUTONOME                                           │
│ - Robot suit les murs (wall_centering_node)                │
│ - Enregistre trajectoire + pose (loop_closure_detector)    │
│ - Crée map SLAM Toolbox                                     │
│                                                             │
│ ↓ Détecte retour au départ (distance < 0.20m)             │
│                                                             │
│ Phase 2: PLANNIFICATION                                     │
│ - A* calcule chemin direct du courant → départ            │
│ - Optimise avec raccourcis + lissage                       │
│ - Trajectoire disponible pour suivi                        │
│                                                             │
│ ↓ Chemin reçu et validé                                   │
│                                                             │
│ Phase 3: RETURNING                                         │
│ - Robot suit chemin A* (PID heading control)              │
│ - Arrive au point de départ avec précision                │
│ - Mission COMPLÉTÉE                                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 📦 Les 4 Fichiers Clés

### 1. `loop_closure_detector.py` 🔍
**Détecte boucle + enregistre trajectoire**

- **Écoute:** Pose robot via `/tf` (map → base_link)
- **Publie:**
  - `/loop_closure/return_detected` (Bool) - Signal détection
  - `/loop_closure/distance_to_start` (Float32) - Distance au départ
  - `/loop_closure/trajectory_point` (PoseStamped) - Points échantillonnés
- **Logique:**
  1. Attend robot s'éloigne (> LEAVE_START_DISTANCE = 0.60m)
  2. Active enregistrement trajectoire
  3. Sample chaque LAP_SAMPLE_DISTANCE = 0.08m
  4. Détecte distance < RETURN_DETECTION_DISTANCE = 0.20m
  5. Signal boucle complétée après EXTRA_DISTANCE_AFTER_RETURN = 2.0m

**Sorties:** `~/trajectories/auto_mapping/trajectory_*.json` + `.npy`

### 2. `trajectory_saver.py` 💾
**Gère sauvegarde trajectoire**

- **Écoute:** Points de `/loop_closure/trajectory_point`
- **Formats de sortie:**
  - JSON (lisible)
  - NumPy (.npy - accès rapide)
  - CSV (compatibilité)
  - Stats (.json - min/max/length)
- **Méthodes:**
  - `save_all_formats()` sauvegarde tous formats
  - `optimize_trajectory()` lisse points bruyants
  - Sauvegarde auto à Ctrl+C

### 3. `return_path_generator.py` 🗺️
**Planification A* sur vraie map SLAM**

- **Écoute:**
  - `/map` (OccupancyGrid de SLAM)
  - `/loop_closure/return_detected` (signal)
  - `/loop_closure/trajectory_point` (point départ)
- **Publie:**
  - `/return_path` (Path - points du chemin)
  - `/return_waypoints` (Float32MultiArray)
- **Algorithme:**
  - Convertit map SLAM en grille
  - A* pathfinding (heuristique Euclidienne)
  - Optimisation: raccourcis + inflation obstacles
  - Lissage par moyenne mobile
  - Conversion monde ↔ grille

**Paramètres tuning:**
```
ASTAR_CELL_SIZE_M = 0.08              # Résolution grille
ASTAR_INFLATION_CELLS = 4             # Sécurité obstacle
ASTAR_OBSTACLE_THRESHOLD = 50         # Seuil occupé (0-100)
ASTAR_SHORTCUT_MAX_SKIP = 20          # Max saut raccourci
ASTAR_ALLOW_DIAGONAL = True           # Mouvements 8-directions
```

### 4. `real_robot_mapping_auto.py` 🎬
**Orchestrateur + suivi chemin de retour**

- **Gère états machine:**
  - `AUTONOME` - wall_centering_node active
  - `RETURN_PLANNED` - Attente chemin A*
  - `RETURNING` - Suivi chemin PID
  - `COMPLETED` - But atteint
- **Contrôle PID pour retour:**
  ```
  heading_error = desired_heading - robot_theta
  angular_cmd = KP * error + KD * (error - error_prev)
  ```
- **Charge tous nœuds:** Lance les 4 en ordre correct
- **Signaux:** Publie `/mapping_completed` (Bool) quand fini

**Paramètres PID retour:**
```python
return_kp = 1.8              # Proportionnel
return_kd = 0.25             # Dérivé
return_lookahead_m = 0.3      # Distance look-ahead
return_goal_tolerance_m = 0.15  # Rayon but
return_max_speed = 0.5        # Vitesse linéaire
```

---

## 🚀 Démarrage

### Prérequis

1. **SLAM Toolbox lancé:**
   ```bash
   ./start_robot_mapping_auto.sh
   ```

2. **ESP32 LiDAR actif** (topic `/scan_raw`)

3. **TF publishing actif** (`/tf` map→base_link depuis SLAM)

### Lancement Complet

**Option 1: Script shell (recommandé)**
```bash
chmod +x real_robot_mapping_auto.sh
./real_robot_mapping_auto.sh
```

**Option 2: Nœuds individuels**
```bash
# Terminal 1
python3 wall_centering_node.py

# Terminal 2
python3 loop_closure_detector.py

# Terminal 3
python3 trajectory_saver.py

# Terminal 4
python3 return_path_generator.py

# Terminal 5 (orchestrateur)
python3 real_robot_mapping_auto.py
```

---

## 📊 Monitoring

### Topics ROS disponibles

```bash
# État mission
ros2 topic echo /loop_closure/return_detected        # Signal boucle
ros2 topic echo /loop_closure/distance_to_start      # Distance départ (m)
ros2 topic echo /mapping_completed                   # Signal fin

# Chemin calculé
ros2 topic echo /return_path                         # Path ROS

# Trajectoire enregistrée
ros2 topic echo /loop_closure/trajectory_point       # Points (stream)
```

### Logs

```bash
# Logs temps réel (shell script)
tail -f /tmp/mapping_auto_logs/*.log

# Logs individuels
tail -f /tmp/mapping_auto_logs/wall_centering.log
tail -f /tmp/mapping_auto_logs/loop_closure.log
tail -f /tmp/mapping_auto_logs/return_path.log
tail -f /tmp/mapping_auto_logs/trajectory_saver.log
```

### Fichiers sauvegardés

```
~/trajectories/auto_mapping/
├── trajectory_20260320_092105.json    # Format lisible
├── trajectory_20260320_092105.npy     # Format numpy
├── trajectory_20260320_092105.csv     # Format CSV
└── trajectory_20260320_092105_stats.json  # Statistiques

~/mapping_logs/
└── mission_1711000865.json           # Logs mission
```

---

## ⚙️ Réglage Fin (Tuning)

### Robot rapide → Augmente temps réaction
```python
# loop_closure_detector.py
RETURN_DETECTION_DISTANCE = 0.30        # Augmente tolérance
LAP_SAMPLE_DISTANCE = 0.15              # Moins de points

# real_robot_mapping_auto.py
return_kp = 2.5                         # Plus décisif
return_kd = 0.4                         # Moins d'overshoot
```

### Obstacles proches → Augmente distance sécurité
```python
# return_path_generator.py
ASTAR_INFLATION_CELLS = 6               # Plus large
ASTAR_SHORTCUT_CLEARANCE_M = 0.50       # Plus strict
```

### Map de mauvaise qualité → Simplifie chemin
```python
# return_path_generator.py
ASTAR_SHORTCUT_MAX_SKIP = 40            # Plus de raccourcis
ASTAR_MIN_PATH_KEEP_RATIO = 0.30        # Accepte path plus court
```

---

## 🔧 Dépannage

### ❌ "Phase 1 démarre mais robot ne bouge pas"
```bash
# Vérifier wall_centering_node
ros2 topic echo /cmd_vel                    # Commandes envoyées?
ros2 topic echo /scan                       # LiDAR reçu?
ros2 service call /freeze_scan std_srvs/...  # Tester service
```

### ❌ "Boucle jamais détectée"
```
- Augmente Lookahead: LEAVE_START_DISTANCE = 1.0m
- Réduit tolérance: RETURN_DETECTION_DISTANCE = 0.30m
- Vérifie TF: ros2 run tf2_tools view_frames
```

### ❌ "A* échoue - pas de chemin"
```
- Augmente inflation: ASTAR_INFLATION_CELLS = 8
- Réduit threshold obstacles: ASTAR_OBSTACLE_THRESHOLD = 60
- Vérifie /map: ros2 topic echo /map | head -20
```

### ❌ "Retour dévie du chemin"
```
- Augmente KP: return_kp = 2.5
- Baisse vitesse: return_max_speed = 0.3
- Réduit lookahead: return_lookahead_m = 0.15
```

---

## 📈 Améliorations Futures

- [ ] Nav2 integration au lieu de PID simple
- [ ] Multi-loop support (circuits multiples)
- [ ] Loop closure graph optimization (pose graph)
- [ ] Kinetic obstacle avoidance dynamique durant retour
- [ ] Global trajectory correction via ICP
- [ ] Automatic parameter tuning (AUTOML)

---

## 📝 Architecture Détail

### Flux de données

```
[wall_centering_node]
        ↓ (autonomie + TF)
   [loop_closure_detector]
        ├→ [trajectory_saver]
        └→ Signal boucle ↓
   [return_path_generator]
        ├→ Écoute /map SLAM
        ├→ A* planning
        └→ Publie /return_path
                ↓
[real_robot_mapping_auto]
        ├→ Reçoit chemin
        ├→ Suivi PID
        └→ Publie /cmd_vel_return
```

### States Transitions

```
┌──────────┐
│ AUTONOME │ (wall_centering actif)
│          │
└─────┬────┘
      │ distance_to_start < 0.20m
      │ + distance_traveled > 3.0m
      ↓
┌──────────────────┐
│ RETURN_PLANNED   │ (Attente A* path)
│                  │
└─────┬────────────┘
      │ len(return_path) > 0
      ↓
┌──────────┐
│ RETURNING│ (Suivi chemin + PID)
│          │
└─────┬────┘
      │ distance_to_goal < 0.15m
      ↓
┌──────────┐
│COMPLETED │ (Mission fini!)
│          │
└──────────┘
```

---

## 📚 Références

- **SLAM Toolbox Config:** `config/slam_toolbox_minimal.yaml`
- **Wall Centering Control:** `wall_centering_node.py` (ligne ~820)
- **Scan Restamper:** `scan_restamper_simple.py` (throttle 1/4)
- **Simulation reference:** `simu_lidar_auto.py` (1273 lignes)

---

**Version:** 1.0 - Robot ECE Project  
**Date:** March 2026  
**Status:** ✅ Production Ready
