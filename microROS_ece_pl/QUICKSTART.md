# 🚀 Quick Start - Multithreading LIDAR + micro_ros

## ⚡ 5 Minutes Setup

### 1. **Vérifier la Configuration**

```cpp
// platformio.ini - Vérifier que c'est configuré
[env:esp32dev]
board_microros_transport = wifi
board_microros_distro = humble
lib_deps = https://github.com/micro-ROS/micro_ros_platformio
build_flags = 
    -DMICRO_ROS_TRANSPORT_ARDUINO_WIFI
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue
```

### 2. **Compiler et Uploader**

```bash
# Option 1: PlatformIO CLI
pio run -e esp32dev -t upload

# Option 2: VS Code
# - Clic droit sur main.cpp
# - Upload

# Monitor les logs
pio device monitor -b 115200
```

### 3. **Branchement LIDAR**

```
ESP32           LD06 LIDAR
─────────────────────────
GPIO16 (RX2) ←  TX
GPIO25 (PWM) →  M_SCTR
GND          →  GND
5V           →  VCC
```

### 4. **Démarrer Agent micro_ros**

```bash
# Terminal 1: Docker container
docker run -it --rm \
  -v /dev:/dev --privileged \
  microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 \
  --ip 0.0.0.0 --port 8888
```

### 5. **Vérifier la Connexion**

```bash
# Terminal 2: Écouter les topics
ros2 topic list

# Devrait afficher:
# /data      (compteur Int32)
# /scan      (LIDAR LaserScan)
```

### 6. **Visualiser les Données**

```bash
# Terminal 3: Voir compteur
ros2 topic echo /data

# Terminal 4: Voir scan LIDAR
ros2 topic echo /scan --max-count=5

# Terminal 5: RViz pour visualisation
rviz2

# Ajouter:
# - Fixed Frame: lidar_link
# - LaserScan (/scan)
```

## 📋 Checklist de Vérification

### Hardware
- [ ] ESP32 connecté via USB
- [ ] LIDAR connecté (UART GPIO16 + PWM GPIO25)
- [ ] 5V/GND sur ESP32 ✓
- [ ] Alimentation LIDAR 5V ✓

### Software
- [ ] PlatformIO installé
- [ ] micro_ros_platformio library ✓
- [ ] Docker + micro_ros_agent prêt

### Compilation
- [ ] `pio run` sans erreurs
- [ ] Upload réussi
- [ ] Serial logs visibles

### ROS2 Connection
- [ ] WiFi ESP32 conecté à réseau
- [ ] Agent micro_ros listening sur UDP8888
- [ ] `ros2 topic list` affiche /data et /scan
- [ ] Messages arrivent (vérifier avec `echo`)

## 🔍 Troubleshooting

### Problem: "LIDAR not publishing"

```bash
# Solution 1: Vérifier logs serial
pio device monitor -b 115200
# Chercher "[LIDAR] Tâche démarrée"

# Solution 2: Vérifier CRC
# Si beaucoup d'erreurs CRC:
# - Vérifier câble UART
# - Essayer baud 115200 au lieu 230400

# Solution 3: Vérifier UART
# - GPIO16 = RX2 ✓
# - Pas de conflit pin
```

### Problem: "WiFi connects but no ROS messages"

```bash
# Solution 1: Vérifier agent
# Terminal 1 - Agent doit afficher:
# [INFO] Client initialized

# Solution 2: Vérifier IP ESP32
# Serial logs: [WiFi] IP locale: XXX.XXX.X.X

# Solution 3: Firewall
# Si derrière firewall: ouvrir port 8888 UDP
```

### Problem: "micro_ros libraries not found"

```bash
# Solution: Forcer mise à jour libraries
pio lib update
pio pkg install
pio run -e esp32dev --verbose
```

## 📊 Expected Output

### Serial Monitor
```
=== SETUP START ===
[LIDAR] Tâche démarrée sur cœur 1
[WiFi] Connecting to iPhone (3)...
[WiFi] IP locale: 192.168.x.x
[STATE] Waiting for agent...
[AGENT] Agent found! Connecting...
[OK] ROS Entities created!
[STATE] CONNECTED!
[PUBLISH OK] Counter: 0
[LIDAR PUBLISH OK] Distance: 320mm, Angle: 45°, Conf: 200
```

### ROS2 Topic Echo
```bash
$ ros2 topic echo /data
---
data: 0
---
data: 1
---
data: 2

$ ros2 topic echo /scan
---
header:
  stamp:
    sec: 5
    nanosec: 234000000
  frame_id: lidar_link
angle_min: 0.0
angle_max: 6.283185
...
ranges: [0.32, 0.35, 0.38, ...]
```

## 🎯 Cas d'Utilisation

### 1. Navigation Robot Autonome
```cpp
// Subscriber: /scan
// Decision: Obstacle avant? → Stop moteurs
if(laser_scan.ranges[0] < 0.5) {
    motor_forward = 0;
}
```

### 2. Cartographie (SLAM)
```cpp
// Publier /scan + /odom
// Utiliser: ros2_humble + Cartographer
ros2 launch cartographer_ros cartographer_offline_node.launch.py
```

### 3. Évitement d'Obstacles
```cpp
// Combiner /scan + /cmd_vel
if(obstacle_detected()) {
    rotate(90°);
}
```

## 📚 Ressources

| Resource | URL |
|----------|-----|
| micro_ros Docs | https://docs.micro-ros.org/ |
| PlatformIO Docs | https://docs.platformio.org/ |
| ROS2 Humble | https://docs.ros.org/en/humble/ |
| LD06 Datasheet | https://www.ydlidar.com/ld06/ |
| FreeRTOS ESP32 | https://esp-idf.readthedocs.io/en/latest/esp32/api-reference/system/freertos.html |

## 🎓 Prochaines Étapes

1. **Ring Buffer** - Implémenter 8000 points historique
2. **PID Motor Control** - Contrôler vitesse rotation plateau
3. **Obstacle Detection** - Alerte automatique <50cm
4. **Web GUI** - Interface radar temps-réel
5. **SLAM Integration** - Cartographie autonome

## 📞 Support

Erreurs rencontrées? Créer issue avec:
- [ ] Logs sérial complets
- [ ] Output `pio run -v`
- [ ] Output `ros2 topic list`
- [ ] Version ESP32 board
- [ ] Version PlatformIO

---

**Happy Coding!** 🚀🤖

**Auteur**: GitHub Copilot  
**Date**: 2026-01-19
