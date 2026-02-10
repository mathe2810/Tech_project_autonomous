# MicroROS ECE Navigation Stack

Localisation et navigation autonome avec Kalman Filter fusion + SLAM + Nav2.

## Architecture

```
ESP32 (Capteurs)
├── /cmd_vel (commandes moteur PWM)
├── /imu/data (IMU brut)
└── /scan (LIDAR)
    ↓
[Nœuds de localisation]
├── motor_odom_node.py       → /odom (odométrie moteur)
├── imu_kalman_filter.py     → /imu/data_filtered (lissage IMU)
├── simple_slam.py           → /map + /slam/pose (mapping)
└── kalman_filter_fusion.py  → /odom_filtered + /tf (fusion Kalman)
    ↓
Nav2 Stack → Navigation autonome
```

## Nœuds

### 1. **motor_odom_node.py**
Odométrie basée sur les commandes PWM + gyroscope IMU.
- **Entrées**: `/cmd_vel`, `/imu/data_filtered`
- **Sorties**: `/odom`, `/tf` (odom → base_link)

### 2. **kalman_filter_fusion.py**
Extended Kalman Filter fusionne odométrie + SLAM pour éliminer la dérive.
- **Entrées**: `/odom`, `/slam/pose`, `/imu/data_filtered`
- **Sorties**: `/odom_filtered`, `/tf` (corrigé)

### 3. **simple_slam.py**
SLAM simple (ICP scan matching + occupancy grid).
- **Entrées**: `/scan`
- **Sorties**: `/map`, `/slam/pose`, `/tf` (map → base_link)

### 4. **imu_kalman_filter.py**
Filtre EMA pour lissage IMU (court terme).
- **Entrées**: `/imu/data`
- **Sorties**: `/imu/data_filtered`

## Démarrage

```bash
# Démarrer tout le stack
./start_stack.sh

# Ou lancer les nœuds individuellement
python3 motor_odom_node.py
python3 imu_kalman_filter.py
python3 simple_slam.py
python3 kalman_filter_fusion.py
ros2 launch nav2_bringup navigation_launch.py
```

## Configuration

- **Nav2**: [config/nav2_params.yaml](config/nav2_params.yaml)
- **SLAM Toolbox**: [config/slam_toolbox_params.yaml](config/slam_toolbox_params.yaml)

## Topics clés

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/odom` | Odometry | OUT | Odométrie moteur (dérive) |
| `/odom_filtered` | Odometry | OUT | Odométrie fusionnée (stable) |
| `/map` | OccupancyGrid | OUT | Carte du SLAM |
| `/scan` | LaserScan | IN | Données LIDAR |
| `/imu/data` | Imu | IN | Données IMU brutes |
| `/imu/data_filtered` | Imu | OUT | Données IMU lissées |
| `/tf` | TF2 | OUT | Transforms (map/odom/base_link) |

## Fichiers de configuration
- `src/main.cpp` - Firmware ESP32 (sensors + motor control)
- `platformio.ini` - Configuration PlatformIO
