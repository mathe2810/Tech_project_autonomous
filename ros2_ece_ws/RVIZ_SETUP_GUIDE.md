# AFFICHER LES POINTS LIDAR DANS RVIZ ✅

## Étape 1 : Lancer le système complet

Terminal 1 - Agent micro_ros :
```bash
cd ~/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Terminal 2 - Launch RViz + Listener :
```bash
cd ~/ros_test/ros2_ece_ws
source install/setup.bash
ros2 launch cpp_pubsub agent_launch.py
```

## Étape 2 : Configurer RViz MANUELLEMENT

### ⚠️ IMPORTANT : Ne pas charger la config directement !

RViz doit être configuré manuellement pour ce LIDAR car :
- Le frame `lidar_link` doit être défini correctement
- Les topics doivent être découverts en temps réel
- Les transformations doivent être valides

### 🎯 Procédure pour ajouter le LaserScan :

1. **RViz s'ouvre** → Tu vois la fenêtre principale vide

2. **En bas à gauche**, clique sur le bouton **"Add"** 

3. **Une boîte de dialogue s'ouvre** → Cherche **"LaserScan"** dans la liste
   - Clique sur **"LaserScan by Topic"**

4. **Nouvelle fenêtre "LaserScan"** → Dans le champ "Topic" :
   - Clique sur la zone blanche
   - Sélectionne **"/scan"** dans la dropdown
   - OU tape `/scan` directement

5. **Clique OK** → Les points devraient apparaître ! 🎨

### ✅ Configuration RViz recommandée :

| Paramètre | Valeur |
|-----------|--------|
| Style | **Spheres** |
| Size (Pixels) | **3-5** |
| Size (m) | **0.05** |
| Color Transformer | **Intensity** ou **Rainbow** |
| Decay Time | **0.2** |
| Use rainbow | **Coché** ✓ |
| Queue Size | **10** |

### 🔍 Vérifier les données qui arrivent :

Dans un terminal 3 :
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /scan --max-count=1
```

Tu devrais voir une sortie LaserScan avec :
- 360 points
- Angles 0-2π
- Distances en mètres

### ❌ Si ça ne marche toujours pas :

```bash
# 1. Vérifier que le topic existe
ros2 topic list | grep scan

# 2. Vérifier les données
ros2 topic echo /scan

# 3. Vérifier le frame
ros2 topic echo /scan | grep frame_id

# 4. Relancer le listener
pkill -f listener
ros2 run cpp_pubsub listener
```

## 📺 Résultat attendu

- **Points colorés** au centre de RViz (dégradé arc-en-ciel)
- **~10-12 points par scan** (sparse LIDAR)
- **Mise à jour toutes les ~1 seconde**
- **Distances 0.2-0.6m** généralement (selon ce qui est autour)

## 💾 Sauvegarder la configuration RViz

Une fois configuré manuellement :
1. **File** → **Save As**
2. Nomme le fichier : `lidar_setup.rviz`
3. Sauvegarde-le dans `~/ros_test/ros2_ece_ws/`
4. Prochaine fois, charge-le avec :
   ```bash
   rviz2 -d ~/ros_test/ros2_ece_ws/lidar_setup.rviz
   ```
