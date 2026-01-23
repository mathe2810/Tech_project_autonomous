# 🎯 Guide Complet - Visualisation LIDAR 360° avec RViz

## ✅ Changements Effectués

### 1. **ESP32 (main.cpp) - Buffer Circulaire**
- ✨ **Buffer circulaire** de 720 points (60 frames × 12 points)
- 📡 **Stockage de TOUS les points bruts** du LIDAR (même ceux avec faible confiance)
- 🔄 Remplissage continu et wrap-around automatique
- 📦 Publié directement en tant que LaserScan 360°

**Caractéristiques:**
```cpp
#define LIDAR_BUFFER_SIZE 720  // 60 frames de données

typedef struct {
    float angle[LIDAR_BUFFER_SIZE];        // Angle en degrés
    float distance[LIDAR_BUFFER_SIZE];     // Distance en mètres
    uint8_t confidence[LIDAR_BUFFER_SIZE]; // Confiance 0-255
    int head;                              // Index d'écriture
    int count;                             // Nombre de points stockés
} LidarCircularBuffer;
```

### 2. **ROS2 Listener (publisher_member_function.cpp)**
- 📊 **Callbacks pour /scan et /data**
- 📈 Affichage des statistiques: min, max, moyenne distance
- 🎯 Log des 10 premiers points avec angle et distance
- 📝 Messages clairs et informatifs

### 3. **Launch File (agent_launch.py)**
- 📋 **Guide d'utilisation intégré**
- 🎨 **Instructions RViz complètes**
- 📡 **Affichage des topics reçus**
- 💡 **Tips pour l'utilisation**

### 4. **Configuration RViz (lidar.rviz)**
- 🔴 **Points en rouge** (Flat Color)
- 📏 **Taille des points**: 8cm (visible)
- 🔍 **Décay Time**: 0.5s (traînée visuelle)
- 📐 **Grille XY** pour orientation
- 🎯 **Vue 3D optimisée** (distance 15m, angle 45°)

---

## 🚀 Procédure de Compilation et Lancement

### Étape 1: Compiler le code ROS2

```bash
cd /home/matheo/ros_test/ros2_ece_ws
source install/setup.bash
colcon build --packages-select cpp_pubsub
source install/setup.bash
```

### Étape 2: Téléverser le code sur ESP32

**Via PlatformIO:**
```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
pio run -t upload
```

Ou via l'interface PlatformIO VS Code.

### Étape 3: Lancer l'agent micro-ROS (Terminal 1)

```bash
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Étape 4: Lancer le listener + RViz (Terminal 2)

```bash
cd /home/matheo/ros_test/ros2_ece_ws
source install/setup.bash
ros2 launch cpp_pubsub agent_launch.py
```

---

## 🎨 Configuration RViz pour Visualisation Optimale

La configuration est **déjà optimisée** dans `lidar.rviz`, mais voici comment la modifier si nécessaire:

### Si vous voulez ajuster la taille des points:
1. Ouvrir RViz
2. Panel gauche → **Displays** → **LaserScan**
3. Modifier **Size (m)**: de 0.05 à 0.15 (par ex.)
4. Cliquer **Save Config**

### Si vous voulez changer la couleur:
1. **LaserScan** → **Color Transformer**: passer de "Flat Color" à "Intensity" ou "Range"
2. Ajuster **Color** si en mode "Flat Color"

### Pour voir l'historique des points (traînée):
1. **LaserScan** → **Decay Time**: passer de 0.5 à 2.0 secondes
2. Les anciens points disparaîtront progressivement

### Pour une meilleure vue 3D:
1. Utiliser **SCROLL** pour zoomer
2. **Click-droit + Drag** pour tourner la vue
3. **Click-gauche + Drag** pour translater

---

## 📡 Topics et Messages

### Topic `/scan` (LaserScan)
```
sensor_msgs/LaserScan
├─ header
│  ├─ frame_id: "lidar_link"
│  └─ stamp: timestamp
├─ angle_min: 0.0
├─ angle_max: 6.283 rad (360°)
├─ angle_increment: 0.01745 rad (~1°)
├─ range_min: 0.06m
├─ range_max: 12.0m
├─ ranges[360]: distance pour chaque degré
└─ intensities: vide
```

### Topic `/data` (Int32)
```
std_msgs/Int32
└─ data: compteur incrementé
```

---

## 🔍 Dépannage

### ❌ RViz affiche "Fixed Frame Error"
**Solution:** Assurez-vous que l'ESP32 publie (connecté à l'agent)

### ❌ Aucun point n'apparaît
1. Vérifier que l'agent est lancé: `docker ps`
2. Vérifier que l'ESP32 envoie des données:
   ```bash
   ros2 topic echo /scan --count 1
   ```
3. Vérifier la connexion WiFi de l'ESP32

### ❌ Les points apparaissent en blanc/gris
**Solution:** Dans RViz, changer **Color Transformer** de "Intensity" à "Flat Color"

### ❌ Les points ne s'affichent qu'en un angle
**Solution:** C'est normal! Le LIDAR 2D ne voit que sur 180° généralement. Vérifiez que:
- `angle_max - angle_min = π` (ou 360° si LIDAR 360)
- Les points sont bien dans le tableau `ranges[360]`

---

## 📊 Lecture des Logs du Listener

Quand le listener reçoit des données:
```
[INFO] LIDAR Data: 145/360 points valid | Min: 0.25m | Max: 5.67m | Avg: 3.45m | Angle range: 180°
```

Cela signifie:
- ✅ 145 points ont une distance valide sur 360
- 📏 Distance la plus proche: 0.25m, la plus lointaine: 5.67m
- 📐 Angle total: 180° (LIDAR 2D classique)

---

## 🛠️ Points d'Intégration Clés

### Buffer Circulaire (main.cpp)
```cpp
// Tous les points sont stockés ici:
static volatile LidarCircularBuffer lidar_buffer;

// À chaque point du LIDAR:
lidar_buffer.angle[lidar_buffer.head] = angle_deg;
lidar_buffer.distance[lidar_buffer.head] = dist_m;
lidar_buffer.confidence[lidar_buffer.head] = conf;
lidar_buffer.head = (lidar_buffer.head + 1) % LIDAR_BUFFER_SIZE;
```

### Publication (main.cpp)
```cpp
// Parcourir TOUS les points du buffer et les placer en 360°
for(int i = 0; i < lidar_buffer.count; i++) {
    int idx = (int)(lidar_buffer.angle[i]) % 360;
    msg_lidar.ranges.data[idx] = lidar_buffer.distance[i];
}
```

### Réception (publisher_member_function.cpp)
```cpp
// Compter les points reçus
for(size_t i = 0; i < msg->ranges.size(); i++) {
    if(msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
        points_detected++;
    }
}
```

---

## 🎯 Résumé des Améliorations

| Avant | Après |
|-------|-------|
| ❌ Seuls les 12 points du dernier frame | ✅ **720 points** en buffer circulaire |
| ❌ Impossible de voir tous les angles | ✅ **360 points** en LaserScan |
| ❌ Pas de validation des données | ✅ **Validation + logging complet** |
| ❌ Configuration RViz générique | ✅ **Configuration optimisée 3D** |
| ❌ Pas de feedback utilisateur | ✅ **Guide + Tips intégrés** |

---

## 📝 Prochaines Étapes Optionnelles

1. **Ajouter des intensités** pour colorer par confiance:
   ```cpp
   msg_lidar.intensities.data = (float*)malloc(360 * sizeof(float));
   msg_lidar.intensities.data[idx] = (float)confidence;
   ```

2. **Filtrer les points bas confiance** dans ROS2:
   ```cpp
   if(lidar_buffer.confidence[i] < 50) continue; // Ignorer
   ```

3. **Exporter les données** en fichier pour analyse:
   ```bash
   ros2 bag record /scan
   ```

4. **Intégrer un node de point cloud** pour 3D complet:
   ```bash
   ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node
   ```

---

**Configuration terminée! 🎉 Les 360 points du LIDAR sont maintenant visibles en temps réel dans RViz.**
