# 🚀 Quick Start - Autonomous Navigation with SLAM

**Démarrer le système de mapping et navigation autonome avec SLAM Toolbox, micro-ROS et ROS2 Humble.**

## ⚡ 5 Minutes Setup

### 1️⃣ **Vérifier Hardware**

```
ESP32                   LIDAR LD06            Motor Driver
─────────────────────────────────────────────────────────
GPIO16 (UART RX2) ←─── TX (LaserScan)
GPIO25 (PWM) ─────────→ M_SCTR (Motor control)
GPIO32, GPIO33 ───────→ Motor encoder inputs
GPIO23, GPIO19 ───────→ Motor PWM outputs (L298N)
GND ──────────────────→ GND
5V ───────────────────→ VCC

IMU MPU6050:
GPIO21 (SDA) ──────── SDA
GPIO22 (SCL) ──────── SCL
GND ──────────────── GND
3.3V ────────────── VCC
```

### 2️⃣ **Compiler et Télécharger Firmware ESP32**

**Option A: PlatformIO CLI**
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
pio run -e esp32dev -t upload
pio device monitor -b 115200  # Voir les logs
```

**Option B: VS Code**
- Clic droit sur `src/main.cpp`
- "PlatformIO: Upload"

### 3️⃣ **Démarrer l'Agent micro-ROS**

```bash
# Terminal 1 - Docker agent (port 8888)
docker run -it --rm -v /dev:/dev --privileged \
  microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 \
  --ip 0.0.0.0 --port 8888
```

✅ Vous devriez voir: `[INFO] [UDPv4AgentLinux]: Client connected from IP 172.20.10.x`

### 4️⃣ **Démarrer la Stack ROS2**

```bash
# Terminal 2 - Démarrer tous les nœuds
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
chmod +x start_stack.sh
./start_stack.sh
```

**Qu'est-ce qui se lance:**
1. ✅ Micro-ROS Agent (vérifie si connecté)
2. ✅ RViz2 (visualisation)
3. ✅ IMU FIR Filter (lisse les vibrations LIDAR)
4. ✅ Motor Odometry Node (estime la position du robot)
5. ✅ Scan Restamper (corrige les timestamps)
6. ✅ SLAM Toolbox (construit la carte)
7. ✅ Simple EKF (fusionne odom + IMU)

### 5️⃣ **Vérifier les Topics**

```bash
# Terminal 3 - Lister les topics actifs
ros2 topic list

# Devrait afficher:
/odom                   # Odométrie moteur
/odom_filtered          # Odométrie fusionnée (odom + IMU)
/scan                   # Scans LIDAR (restampés)
/scan_raw              # Scans bruts (avant restampage)
/imu/data              # IMU brut
/imu/data_filtered     # IMU lissé
/map                   # Carte SLAM (occupancy grid)
/tf                    # Transformations
/cmd_vel               # Commandes de mouvement
```

### 6️⃣ **Contrôler le Robot**

```bash
# Terminal 4 - Clavier (WASD)
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
python3 teleop_keyboard.py
```

**Touches:**
- **W** = Avancer (0.3 m/s)
- **A** = Tourner à gauche (1.5 rad/s)
- **S** = Reculer (-0.3 m/s)
- **D** = Tourner à droite (-1.5 rad/s)
- **Space** = Stop
- **Q** = Quitter

### 7️⃣ **Voir la Carte en RViz**

1. **Ouvrir RViz** (lancé automatiquement par start_stack.sh)
2. **Fixed Frame** → `map`
3. **Add Display** → ajouter:
   - ✅ **Map** (topic: `/map`)
   - ✅ **LaserScan** (topic: `/scan`)
   - ✅ **Odometry** (topic: `/odom_filtered`)
   - ✅ **TF** (pour les frames)

**Vous devriez voir:**
- 🔵 Robot bleu au centre
- 🟩 Grille grise (carte occupée)
- 🔴 Points rouges (scans LIDAR)
- 🟦 Flèche (direction robot)

---

## 🧪 Test Automatisé

```bash
# Envoyer des commandes de test (avancer → rotation → stop)
python3 test_movement.py
```

Vérifiez que:
- ✅ Position change: `ros2 topic echo /odom`
- ✅ Scans s'accumulent: RViz montre plus de points
- ✅ Carte se met à jour: `/map` change

---

## 🔧 Troubleshooting

### ❌ Topic `/map` n'existe pas
```bash
# Vérifier que SLAM est lancé
ros2 topic list | grep map
ros2 topic hz /map  # Devrait voir 1Hz
```

**Solution:** Vérifier que `/scan` est publié (restamper doit tourner)

### ❌ Scans ne s'accumulent pas en RViz
```bash
# Vérifier timestamp des scans
ros2 topic echo /scan --max-count=1 | grep stamp
```

**Solution:** Timestamp doit être le temps actuel, pas 8000+ sec

### ❌ Robot ne se déplace pas
```bash
# Envoyer test_movement.py et vérifier /odom
ros2 topic echo /odom --max-count=1
```

**Solution:** Motor odometry nécessite `/cmd_vel` pour bouger

### ❌ Pas de connexion micro-ROS
```bash
# Vérifier l'ESP32
pio device monitor -b 115200

# Redémarrer l'agent
pkill -f micro_ros_agent
docker run -it --rm ... ros2 run micro_ros_agent ...
```

---

## 📊 Topics et Frames Hierarchy

```
map (frame de la carte SLAM)
├── odom (frame odométrie)
│   └── base_link (cadre robot)
│       └── laser_link (LIDAR)
└── imu_link (IMU)

Topics Key:
/scan          → LaserScan (LIDAR, 10Hz)
/odom          → Odometry (moteur, 50Hz)
/odom_filtered → Odometry (fusionnée, 50Hz)
/imu/data      → Imu (100Hz brut)
/imu/data_filtered → Imu (100Hz lissé)
/map           → OccupancyGrid (SLAM, 1Hz)
/tf            → Transformations dynamiques
/cmd_vel       → Twist (commandes moteur)
```

---

## 📝 Checklist de Démarrage

- [ ] ESP32 compilé et téléchargé
- [ ] LIDAR + IMU branchés
- [ ] Docker micro-ROS agent lancé (port 8888)
- [ ] `./start_stack.sh` lancé sans erreurs
- [ ] RViz ouvert avec les bons displays
- [ ] `ros2 topic list` montre `/scan`, `/odom`, `/map`
- [ ] Teleop keyboard fonctionne (W/A/S/D)
- [ ] Robot se déplace en RViz
- [ ] Carte se met à jour autour du robot

**Si tout est ✅ : Le système fonctionne ! 🎉**

---

## 📚 Prochaines Étapes

1. **Voir [ARCHITECTURE.md](ARCHITECTURE.md)** - Architecture système complète
2. **Voir [SLAM_IMPLEMENTATION.md](SLAM_IMPLEMENTATION.md)** - Algorithme SLAM détaillé
3. **Voir [PERFORMANCES.md](PERFORMANCES.md)** - Comparaison avant/après optimisations
4. **Configurer Nav2** - Pour navigation autonome (non manuel)
5. **Ajouter loop closure** - Pour grandes cartes sans dérive

---

## ⚠️ Notes Importantes

### Timestamps
- ESP32 boot-time (sec: ~8000) vs ROS2 wall-clock (sec: ~1770000000)
- **Solution:** `scan_restamper.py` re-timestamp automatiquement
- Si `/map` reste vide → vérifier que restamper tourne

### Odométrie
- Motor odometry **dépend de `/cmd_vel`** pour calculer position
- Sans commande = position statique (correct)
- Fusion EKF corrige la dérive avec SLAM

### SLAM Toolbox
- Mode `mapping` : accumule tous les scans
- Résolution `0.025m` = 2.5cm/pixel (très précis, 5x5m zone)
- Ceres solver avec SPARSE_NORMAL_CHOLESKY (rapide, bas CPU)

---

**Version:** 1.0 (Février 2026) | **Status:** ✅ Stable & Tested
