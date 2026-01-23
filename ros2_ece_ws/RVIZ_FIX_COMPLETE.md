# ✅ RVIZ - SOLUTION COMPLÈTE

## Problème Identifié
```
rviz2: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: 
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

### Cause Racine
VS Code (snap) pollue les variables d'environnement :
- `XDG_DATA_HOME` → `/home/matheo/snap/code/...`
- `GTK_PATH` → `/snap/code/219/usr/lib/x86_64-linux-gnu/gtk-3.0`
- Plusieurs autres variables "VSCODE_SNAP_ORIG"

Cela force RViz à chercher des dépendances dans `/snap/core20/` au lieu de `/usr/lib/`.

## Solution Implémentée

### 1. Wrapper Script (`run_rviz.sh`)
Crée un environnement propre en neutralisant les variables snap de VS Code :

```bash
#!/bin/bash
# Désinfecter toutes les variables snap de VS Code
unset XDG_DATA_HOME
unset GSETTINGS_SCHEMA_DIR
unset GTK_PATH
unset GTK_EXE_PREFIX
# ... etc (18 variables nettoyées)

# Remettre le PATH correct (sans snap)
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Source ROS
source /opt/ros/humble/setup.bash

# Lancer RViz
exec rviz2 "$@"
```

**Localisation :** `/home/matheo/ros_test/ros2_ece_ws/run_rviz.sh`

### 2. Intégration dans Launch File
**Fichier :** `src/cpp_pubsub/launch/agent_launch.py`

Changement :
```python
# AVANT : Direct Node (cassé par snap)
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
),

# APRÈS : ExecuteProcess avec wrapper script (propre)
ExecuteProcess(
    cmd=['/home/matheo/ros_test/ros2_ece_ws/run_rviz.sh'],
    output='screen',
),
```

## Utilisation

### Option 1 : Lancer RViz seul (test rapide)
```bash
cd ~/ros_test/ros2_ece_ws
./run_rviz.sh
```

### Option 2 : Lancer avec agent_launch.py (complet)
```bash
# Terminal 1 : Micro_ros agent
docker run -it --rm microros/micro-ros-docker:humble \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2 : Launch file + listener + RViz
cd ~/ros_test/ros2_ece_ws
source install/setup.bash
ros2 launch cpp_pubsub agent_launch.py
```

## Vérification
```bash
ps aux | grep rviz2          # Voir RViz en train de tourner
cat /tmp/rviz.log            # Logs de démarrage
```

## Avertissement Wayland
```
Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. 
Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
```

→ **Normal et ignorable.** RViz démarre correctement en mode X11 de fallback.

## Debugging Avancé (si ça rate encore)
```bash
# Voir exactement quelles dépendances RViz charge
ldd $(which rviz2) | grep snap    # Devrait être VIDE

# Vérifier l'environnement RViz
/home/matheo/ros_test/ros2_ece_ws/run_rviz.sh --verbose

# Lancer RViz avec strace pour voir les appels système
strace -e open,openat /home/matheo/ros_test/ros2_ece_ws/run_rviz.sh 2>&1 | grep pthread
```

## Configuration RViz pour LaserScan
1. Dans RViz, clique **"Add"** → **"By topic"** → Sélectionne `/scan` (LaserScan)
2. Ajuste les paramètres de visualisation :
   - **Color transformer** : Intensity
   - **Style** : Points ou Spheres
   - **Decay time** : 0.2s (pour voir l'historique)

## Status ✅
- ✅ RViz lance sans erreur snap
- ✅ À intégrer dans le launch file
- ✅ Prêt pour visualisation temps réel LIDAR
