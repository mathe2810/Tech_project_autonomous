# microROS ESP32 Firmware - LIDAR 360°

Firmware ESP32 pour la publication de données LIDAR 360° via microROS et ROS2.

## 📋 Vue d'ensemble

Ce firmware implémente:
- **Buffer circulaire** - 720 points LIDAR (360° / 0.5°)
- **Publication LaserScan** - Format standard ROS2
- **Communication microROS** - Via UDP (port 8888)
- **Interface Arduino** - Utilise le SDK Arduino ESP32

## ⚙️ Configuration

### Fichier: `platformio.ini`

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
board_microros_transport = serial
board_microros_distro = humble
lib_deps = https://github.com/micro-ROS/micro_ros_platformio
```

**Paramètres clés:**
- **Platform**: Espressif32 (ESP32)
- **Board**: ESP32-DevKit-C
- **Framework**: Arduino
- **microROS Transport**: Serial (USB)
- **ROS Distro**: Humble (compatible ROS2)

## 📦 Dépendances

Installées via PlatformIO:
```
micro_ros_platformio        # microROS for PlatformIO
```

## 🔌 Brochages Recommandés

| Capteur LIDAR | ESP32 | Notes |
|---------------|-------|-------|
| TX | RX2 (GPIO16) | Serial2 |
| RX | TX2 (GPIO17) | Serial2 |
| GND | GND | |
| VCC | 5V | |

Voir `src/main.cpp` pour la configuration Serial2.

## 🚀 Installation & Compilation

### Option 1: VS Code + PlatformIO Extension (Recommandé)

1. Installer l'extension **PlatformIO IDE** dans VS Code
2. Ouvrir ce dossier dans VS Code
3. Cliquer sur "Build" dans la barre de statut
4. Connecter l'ESP32 via USB
5. Cliquer sur "Upload"

### Option 2: CLI PlatformIO

```bash
# Installation
pip install platformio

# Compilation
platformio run

# Compilation + Upload
platformio run --target upload

# Monitorer la sortie série
platformio device monitor --port /dev/ttyUSB0 --baud 115200
```

### Option 3: Docker

```bash
docker run -it --rm \
  -v $(pwd):/workspace \
  -v /dev:/dev \
  --privileged \
  platformio/platformio-core \
  pio run --target upload
```

## 📝 Code Principal

Fichier: `src/main.cpp`

### Structure

```cpp
// Initialisation
setup() {
  // Configuration Serial2
  // Initialisation microROS
  // Création du nœud ROS2
}

// Boucle principale
loop() {
  // Lecture LIDAR
  // Publication LaserScan
  // Gestion microROS
}
```

### Points clés du code

1. **Buffer circulaire** - Stockage efficace des 720 points
2. **Temps de publication** - Gestion des timestamps
3. **Frame référence** - "lidar" pour RViz
4. **Gestion erreurs** - Reconnexion automatique

## 🔍 Debugging

### Monitorer la sortie série

```bash
platformio device monitor --port /dev/ttyUSB0 --baud 115200
```

### Messages de debug

Le firmware affiche:
```
[microROS] Connecting to agent...
[microROS] Connected!
[LIDAR] Publishing: 720 points @ 10Hz
```

## 📊 Performance

| Métrique | Valeur |
|----------|--------|
| Points LIDAR | 720 |
| Résolution angulaire | 0.5° |
| Fréquence publication | ~10 Hz |
| Latence | <100ms |
| Bande passante | ~200 KB/s |
| RAM utilisée | ~150 KB |
| Flash utilisée | ~400 KB |

## 🔧 Modification du Code

### Changer la résolution

```cpp
// Dans src/main.cpp
#define LIDAR_POINTS 720  // Modifier cette valeur
```

### Changer la fréquence de publication

```cpp
// Dans src/main.cpp
#define PUBLISH_FREQ_HZ 10  // Changer la fréquence
```

### Ajouter des capteurs supplémentaires

1. Modifier le code dans `src/main.cpp`
2. Compiler: `platformio run`
3. Upload: `platformio run --target upload`

## ⚡ Pins GPIO Disponibles

| Port | Utilisation |
|------|------------|
| GPIO16 (RX2) | Serial2 RX |
| GPIO17 (TX2) | Serial2 TX |
| GPIO4 | GPIO libre |
| GPIO12 | GPIO libre |
| GPIO13 | GPIO libre |
| GPIO14 | GPIO libre |

## 🐛 Troubleshooting

### Port série non trouvé
```bash
# Lister les ports
ls /dev/tty*

# Vérifier les permissions
sudo usermod -a -G dialout $USER
newgrp dialout
```

### Upload échoue
```bash
# Vérifier les drivers FTDI/CH340
# Installer les drivers si nécessaire pour l'ESP32
```

### Pas de connexion microROS
1. Vérifier que l'agent microROS est lancé
2. Vérifier le port UDP (8888)
3. Vérifier la connexion réseau

## 📚 Ressources

- [PlatformIO Documentation](https://docs.platformio.org/)
- [microROS for PlatformIO](https://github.com/micro-ROS/micro_ros_platformio)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [ROS2 LaserScan Message](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/LaserScan.html)

## 🔄 Mise à jour Firmware

```bash
# Pull latest changes
git pull

# Rebuild
platformio run

# Upload new version
platformio run --target upload
```

## 📄 Fichiers Importants

| Fichier | Description |
|---------|-------------|
| `src/main.cpp` | Code principal du firmware |
| `platformio.ini` | Configuration du projet |
| `include/` | En-têtes (librairies externes) |
| `lib/` | Dépendances locales |

---

**Dernière mise à jour:** Janvier 2026  
**Version firmware:** 1.0.0  
**Compatible:** ROS2 Humble, microROS
