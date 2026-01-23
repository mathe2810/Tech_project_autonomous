# 🎨 Instructions RVIZ - Visualisation LIDAR 360°

## 📍 Vue d'ensemble

Quand vous lancez `ros2 launch cpp_pubsub agent_launch.py`, RViz s'ouvre **automatiquement** avec la configuration optimisée pour afficher les 360 points du LIDAR.

---

## ✅ Vérification Automatique

La configuration RViz est **pré-configurée** avec:
- ✓ **Grille XY** grise en fond (référence spatiale)
- ✓ **Points LIDAR** en **rouge** (très visibles)
- ✓ **360 points** affichés simultanément
- ✓ **Frame** fixe: `lidar_link`
- ✓ **Vue 3D** optimisée (distance 15m, angle 45°)

**Vous n'avez normalement rien à faire!** La config s'applique automatiquement.

---

## 🎯 Si Vous Devez Ajuster Manuellement

### Cas 1: Les points ne s'affichent pas

**Étape 1:** Vérifier que l'agent est lancé
```bash
docker ps | grep micro-ros-agent
```

Si rien n'apparaît → Lancez l'agent:
```bash
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Étape 2:** Vérifier les topics
```bash
ros2 topic list
```

Vous devez voir:
```
/scan
/data
```

**Étape 3:** Vérifier les données reçues
```bash
ros2 topic echo /scan --count 1
```

Vous devriez voir les champs du LaserScan.

---

### Cas 2: RViz affiche "No Transform available"

**Solution:** C'est normal les 5 premières secondes. Attendez.

Si le message persiste:
1. Dans RViz, allez à **Global Options** (panel gauche)
2. **Fixed Frame**: choisir **lidar_link**
3. Cliquer sur **OK**

---

### Cas 3: Je veux agrandir les points

1. Panel gauche → **Displays**
2. Cliquer sur **LaserScan (Tous les 360 points)**
3. Trouver **Size (m)**: modifier de **0.08** à **0.15** (ou plus)
4. Les points deviennent plus gros instantanément

---

### Cas 4: Je veux changer la couleur

**Option A: Gardez la couleur rouge fixe**
1. **LaserScan** → **Color Transformer**: rester à **Flat Color**
2. Cliquer sur **Color**: choisir la couleur désirée
3. Cliquer **OK**

**Option B: Colorer par distance (rouge = proche, bleu = loin)**
1. **Color Transformer**: choisir **Range**
2. Les points deviennent progressivement rouges/vert/bleu selon la distance

**Option C: Colorer par confiance**
1. D'abord, compiler avec intensités:
   ```cpp
   // Dans main.cpp ESP32, section publish:
   msg_lidar.intensities.size = 360;
   msg_lidar.intensities.data = (float*)malloc(360 * sizeof(float));
   for(int i = 0; i < 360; i++) {
       msg_lidar.intensities.data[i] = (float)lidar_buffer.confidence[i];
   }
   ```
2. Recompiler et téléverser
3. Dans RViz: **Color Transformer** → **Intensity**

---

### Cas 5: Je veux voir l'historique des points (traînée)

1. **LaserScan** → **Decay Time**: modifier de **0.5** à **2.0** secondes
2. Les anciens points disparaîtront lentement

Pour voir plus longtemps: mettez **5.0**

---

### Cas 6: La vue 3D est bizarrement orientée

**Réinitialiser la vue:**
1. Panel droit → **Views**
2. Sous **Current View**, cliquer sur **View Type**: choisir **XYOrbit**
3. Régler:
   - **Distance**: 15m
   - **Pitch**: 0.65 rad (≈37°)
   - **Yaw**: 0.785 rad (45°)

**Ou simplement:**
1. Pressez **Home** sur le clavier
2. RViz remet la vue par défaut

---

## 🎮 Contrôles RViz

| Action | Clavier/Souris |
|--------|-----------------|
| Zoomer | **Scroll** vers haut/bas |
| Tourner la vue | **Click droit** + Drag |
| Translater (panorama) | **Click droit + Shift** + Drag |
| Sélectionner un point | **Click gauche** |
| Mesurer distance | Outil **Measure** (toolbar) |

---

## 📊 Lecture des Affichages

### Dans RViz:

```
┌─────────────────────────────────┐
│ Global Options                  │
├─────────────────────────────────┤
│ ✓ Fixed Frame: lidar_link       │
│ ✓ Background: gris (48,48,48)   │
│ ✓ Frame Rate: 30Hz              │
└─────────────────────────────────┘

┌─────────────────────────────────┐
│ Displays                        │
├─────────────────────────────────┤
│ ✓ Grid (grise, alpha 0.3)       │
│ ✓ LaserScan (rouge, 360 points) │
│   - Size: 8cm                   │
│   - Decay: 0.5s                 │
│   - Color: Red                  │
└─────────────────────────────────┘

┌─────────────────────────────────┐
│ Terminal Output:                │
├─────────────────────────────────┤
│ [INFO] LIDAR Data: 145/360      │
│ points valid | Min: 0.25m       │
│ Max: 5.67m | Avg: 3.45m         │
└─────────────────────────────────┘
```

**Interprétation:**
- ✅ 145 points ont une distance valide (entre 0.06m et 12m)
- 📏 Point le plus proche: 25cm
- 📏 Point le plus lointain: 5.67m
- 📏 Distance moyenne: 3.45m

---

## 🔍 Dépannage Avancé

### Les points sautent partout

**Cause:** Buffer circulaire en remplissage
**Solution:** Attendez 60 frames (environ 2 secondes) le temps que le buffer se remplisse complètement

### Je vois des "trous" dans le scan

**Cause:** Normal! Le LIDAR 2D ne voit que 180° (pas 360°)
**Vérification:** 
```bash
ros2 topic echo /scan | grep "angle_max"
```
Vous verrez probablement: `angle_max: 3.14` (π radians = 180°)

### Les points sont très loin (>10m) ou très près (<5cm)

**Cause:** Vérifiez que le LIDAR communique correctement
**Debug:**
```bash
# Terminal ESP32:
Serial Monitor du LIDAR
Vous devez voir: [LIDAR] Task started on core 1
```

---

## 💾 Sauvegarder votre Configuration RViz

Après avoir ajusté les paramètres:

**Méthode 1: Automatiquement**
- RViz sauvegarde automatiquement quand vous fermez
- Si c'est votre première fois, cliquer **Panels** → **Save Config As**

**Méthode 2: Manuellement**
```bash
# Exporter la config actuelle
ros2 run rviz2 rviz2 -d lidar.rviz --save-display-config
```

---

## 📝 Points Clés à Retenir

✅ **Configuration automatique** → Pas besoin de setup manuel
✅ **360 points affichés** → Voir la zone complète autour du LIDAR
✅ **Points rouges** → Faciles à voir sur fond gris
✅ **Frame: lidar_link** → C'est le référentiel du LIDAR
✅ **Zoom avec scroll** → Contrôle facile
✅ **Traînée 0.5s** → Voir le mouvement des points

---

## 🎬 Workflow Rapide

1. **Terminal 1:**
   ```bash
   docker run -it --rm microros/micro-ros-docker:humble \
     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

2. **Terminal 2:**
   ```bash
   cd /home/matheo/ros_test/ros2_ece_ws
   source install/setup.bash
   ros2 launch cpp_pubsub agent_launch.py
   ```

3. **RViz s'ouvre automatiquement** avec les 360 points en rouge ✓

4. **Regarder le terminal 2** pour les statistiques LIDAR

5. **Ajuster si nécessaire** avec les touches/souris RViz

---

## ❓ Questions Fréquentes

**Q: Pourquoi les points apparaissent en rouge et pas d'autres couleurs?**
A: C'est intentionnel! Le rouge est très visible sur le fond gris. Vous pouvez changer en suivant le cas 4.

**Q: Combien de points devrais-je voir?**
A: Entre 100-360 selon la géométrie autour du LIDAR. 360 points si tous les angles reçoivent une détection.

**Q: Pourquoi 0.5s de décay et pas 0?**
A: Cela permet de voir le mouvement des obstacles. Si c'est trop, augmentez Decay Time.

**Q: Je ne vois rien à 0.06m du LIDAR - c'est normal?**
A: Oui! C'est la "zone morte" du capteur. Il ne peut pas détecter à moins de 6cm.

---

## 🎓 Prochaines Étapes

Maintenant que RViz fonctionne:

1. **Tester le filtrage par confiance** dans ROS2
2. **Exporter en point cloud** pour cartographie 3D
3. **Intégrer avec SLAM** pour la localisation
4. **Créer une carte occupancy** pour la navigation

Voir: **GUIDE_LIDAR_360_COMPLET.md** section "Prochaines étapes"

---

**Vous êtes prêt pour explorer vos données LIDAR en 360°! 🚀**
