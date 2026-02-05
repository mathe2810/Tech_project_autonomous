# 🔧 Optimisations WiFi pour Connexion Partagée via Téléphone

## 🚨 Problèmes avec les connexions partagées
- **Bande passante limitée** (partage de données mobiles)
- **Latence variable** (réseau mobile instable)
- **Pertes de paquets** (WiFi avec signal faible)

---

## ✅ Solutions pour ESP32

### 1. **Réduire la taille des messages (Mini-scan au lieu de 360 points)**

Dans `main.cpp`, au lieu de publier 360 points :

```cpp
// Publier seulement les points pertinents (ex: 90 points au lieu de 360)
#define PTS_PUBLISH 90  // Au lieu de 360

// Dans publishTask:
for(int i = 0; i < 360; i += 4) {  // Prendre 1 point sur 4
  lidar_ranges[j] = lidar_buffer[i].distance;
  j++;
}
msg_lidar.ranges.size = j;  // Dynamique
```

**Gain**: -75% bande passante, 4x plus rapide ✅

---

### 2. **Configurer WiFi pour débit optimal**

**À ajouter dans `setup()` du ESP32** :

```cpp
// Réduire la bande passante WiFi (économise batterie + réduit bruit)
WiFi.setTxPower(WIFI_POWER_8dBm);  // Min puissance

// Mode WiFi 802.11 B (lent mais stable sur mauvais signal)
esp_wifi_set_phy_mode(WIFI_PHY_MODE_11B);

// Buffer WiFi réduit
WiFi.setAutoReconnect(true);
WiFi.setSleep(false);  // Garder radio active
```

---

### 3. **Compression des données**

Au lieu de 360 floats, utiliser des **uint8_t** (distance / 10) :

```cpp
// Avant: 360 * 4 bytes = 1440 bytes
// Après: 360 * 1 byte = 360 bytes = 4x plus petit!

uint8_t compressed_ranges[360];
for(int i = 0; i < 360; i++) {
  compressed_ranges[i] = (uint8_t)(lidar_buffer[i].distance * 10);  // Précision: 10cm
}
```

---

### 4. **QoS Optimisé** (dans le bridge)

**Dans le terminal ROS2** :

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 \
  ros_qos_profile:=sensor_data  # Au lieu de default
```

---

### 5. **Monitorer la qualité WiFi**

**Sur l'ESP32, ajouter dans le loop** :

```cpp
if(WiFi.status() == WL_CONNECTED) {
  int rssi = WiFi.RSSI();  // Signal strength (-30 à -100 dBm)
  Serial.printf("[WiFi] Signal: %d dBm | Quality: %d%%\n", 
                rssi, (rssi + 100) * 2);
}
```

**Interprétation** :
- `-30 à -50 dBm`: Excellent ✅
- `-50 à -70 dBm`: Bon
- `-70 à -80 dBm`: Acceptable
- `-80 à -100 dBm`: Faible 🚨

---

### 6. **Fragmentation UDP**

**Si la connexion coupe souvent** : Réduire la MTU

```cpp
// MTU par défaut: 1500 bytes (trop gros pour WiFi faible)
// Réduire à 512 pour pas de fragmentation
#define UDP_PACKET_MAX 512
```

---

## 🎯 Priorité des optimisations

| Solution | Impact | Effort |
|---|---|---|
| 1. Réduire points LIDAR (90 au lieu de 360) | **~80% bande passante** | ⭐ Facile |
| 2. Utiliser uint8_t compressé | **4x plus petit** | ⭐⭐ Moyen |
| 3. Réduire puissance WiFi | **Meilleure stabilité** | ⭐ Facile |
| 4. QoS optimisé ROS | **Moins de retransmissions** | ⭐ Facile |
| 5. Monitorer signal | **Debugger les problèmes** | ⭐ Facile |

---

## 🚀 Test rapide

**Teste d'abord la solution #1** (réduire les points) :

```cpp
// Dans publishTask, remplacer :
for(int i = 0; i < 360; i++) {
  lidar_ranges[i] = lidar_buffer[i].distance;
}
msg_lidar.ranges.size = 360;

// Par :
uint16_t count = 0;
for(int i = 0; i < 360; i += 4) {  // 1 point sur 4
  lidar_ranges[count++] = lidar_buffer[i].distance;
}
msg_lidar.ranges.size = count;  // ~90 points
```

**Résultat** : Bande passante divisée par 4, latence réduite ! 🚀
