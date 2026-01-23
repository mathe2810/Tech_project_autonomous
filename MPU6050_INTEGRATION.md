# MPU6050 Integration - LIDAR + IMU Fusion

## 📋 Vue d'ensemble

Ce projet combine:
- **LIDAR 360°** (publication `/scan`)
- **MPU6050 IMU** (publication `/imu/data`)
- **Fusion de capteurs** dans ROS2 + Foxglove

## 🔌 Brochage

### ESP32 ↔ MPU6050
| Signal | ESP32 Pin | MPU6050 Pin |
|--------|-----------|-------------|
| SDA    | GPIO 21   | SDA         |
| SCL    | GPIO 22   | SCL         |
| GND    | GND       | GND         |
| VCC    | 3.3V      | VCC         |
| AD0    | GND       | (Adresse I2C: 0x68) |

### ESP32 ↔ LIDAR
| Signal | ESP32 Pin | LIDAR Pin |
|--------|-----------|-----------|
| TX     | GPIO 17   | RX        |
| RX     | GPIO 16   | TX        |
| GND    | GND       | GND       |
| VCC    | 5V        | VCC       |

## 📦 Architecture

```
ESP32
├── LIDAR Serial2 (115200 baud)
│   └── Publie: /scan (720 points @ 10Hz)
├── MPU6050 I2C (400kHz)
│   ├── Accélération (±2g)
│   ├── Gyroscope (±250°/s)
│   └── Publie: /imu/data (20Hz)
└── microROS Agent
    └── UDP Port 8888

ROS2 Humble
├── /imu/data (sensor_msgs/Imu)
├── /scan (sensor_msgs/LaserScan)
├── /tf (transformations)
└── Foxglove Bridge (port 8765)
```

## 🚀 Démarrage

### 1. Compiler le Firmware
```bash
cd ~/ros_test/microROS_ece_pl
platformio run --target upload
```

**Attendre les logs:**
```
[SETUP] Initialisation ESP32 + LIDAR + MPU6050 + microROS
[I2C] Initialisation...
[MPU6050] Détecté et initialisé!
[LIDAR] Serial2 initialisé
[SETUP] ✓ Tous les composants initialisés!
```

### 2. Lancer ROS2 (4 Terminaux)

**T1 - Agent microROS:**
```bash
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**T2 - Foxglove Bridge:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

**T3 - Sensor Fusion Listener:**
```bash
cd ~/ros_test/ros2_ece_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run cpp_pubsub sensor_fusion_listener
```

**T4 - Foxglove UI:**
```bash
foxglove-studio
# Connexion: ws://localhost:8765
```

## 📊 Topics

### /imu/data (20 Hz)
```bash
geometry_msgs/msg/Vector3 linear_acceleration
  float64 x  # m/s²
  float64 y
  float64 z

geometry_msgs/msg/Vector3 angular_velocity
  float64 x  # rad/s
  float64 y
  float64 z

float64[9] linear_acceleration_covariance
float64[9] angular_velocity_covariance
```

Visualiser:
```bash
ros2 topic echo /imu/data
ros2 topic hz /imu/data
```

### /scan (10 Hz)
```bash
float32[] ranges      # Distance (m) pour chaque angle
float32[] intensities # Intensité (0-1)
float32 angle_min     # 0 rad
float32 angle_max     # 2π rad
float32 angle_increment # 2π/720 ≈ 0.00873 rad
```

Visualiser:
```bash
ros2 topic echo /scan --max-count=1
```

## 🔧 Configuration Foxglove

1. **Ouvrir Foxglove**
   ```
   ws://localhost:8765
   ```

2. **Ajouter un panel 3D**
   - Plus (+) → 3D

3. **Topics à afficher:**
   - ✓ `/scan` → LaserScan (nuage points)
   - ✓ `/imu/data` → (optionnel, affiche accélération)
   - ✓ `/tf` → Transformations

4. **Configurer la vue:**
   - Frame: "map"
   - Target: "lidar"
   - Point size: 5-8px

## 📈 Données MPU6050

### Calibrage (optionnel)
```bash
# Accélération au repos ≈ [0, 0, 9.81] m/s²
# Gyroscope au repos ≈ [0, 0, 0] rad/s
# Température: voir logs ESP32
```

### Conversion LSB → SI
**Accélération (±2g):**
- 1g = 9.81 m/s²
- 1 LSB = 9.81 / 16384 m/s² ≈ 0.000599 m/s²

**Gyroscope (±250°/s):**
- 1°/s = π/180 rad/s ≈ 0.01745 rad/s
- 1 LSB = π/(180 × 131) rad/s ≈ 0.000133 rad/s

**Température:**
```
T(°C) = (raw_temp + 12412) / 340
```

## 🐛 Debugging

### Vérifier les topics
```bash
ros2 topic list
# Doit afficher: /imu/data, /scan, /tf, etc.
```

### Vérifier les nœuds
```bash
ros2 node list
# Doit afficher: /esp32_sensor_fusion, /micro_ros_agent, etc.
```

### Vérifier les données
```bash
ros2 topic echo /imu/data
ros2 topic echo /scan --max-count=1
```

### Pas de données?
1. Vérifier que l'agent microROS est lancé
2. Vérifier le brochage I2C (SDA=21, SCL=22)
3. Vérifier que le MPU6050 est détecté:
   ```bash
   # Sur ESP32, regarder les logs Serial:
   # [MPU6050] Détecté et initialisé!
   ```
4. Vérifier la fréquence `ros2 topic hz /imu/data`

## 📝 Code Source

### Firmware ESP32
- `microROS_ece_pl/src/main.cpp` - Publication IMU + LIDAR

### Nœud ROS2
- `ros2_ece_ws/src/cpp_pubsub/src/sensor_fusion_listener.cpp` - Réception + affichage

## 🔗 Ressources

- [MPU6050 Datasheet](https://www.uctronics.com/download/Arduino/MPU6050.pdf)
- [microROS with MPU6050](https://github.com/micro-ROS/micro_ros_arduino)
- [ROS2 sensor_msgs/Imu](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)
- [Foxglove IMU Plugin](https://foxglove.dev/)

## 📊 Performance

| Capteur | Fréquence | Bande passante |
|---------|-----------|----------------|
| IMU     | 20 Hz     | ~2 KB/s        |
| LIDAR   | 10 Hz     | ~100 KB/s      |
| Total   | -         | ~102 KB/s      |

---

**Version:** 1.0  
**Date:** Janvier 2026  
**Statut:** Production Ready
