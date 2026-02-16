# SLAM Stack avec RF2O - Guide de Démarrage

## 📋 Architecture

Stack complet avec:
- **RF2O**: Odométrie laser en temps réel (scan-matching LIDAR)
- **Motor Odometry**: Odométrie moteur (secours)
- **Fusion**: 80% RF2O + 20% Motor
- **SLAM Toolbox**: Mapping avec boucle de fermeture

## 🚀 Lancement

```bash
cd /home/matheo/microros_ece_ws/src/micro_ros_setup/microROS_ece_pl
./start_slam_simple.sh
```

## 📊 Topics Publiés

| Topic | Source | Description |
|-------|--------|-------------|
| `/scan` | Scan Restamper | LIDAR scans avec timestamps synchro |
| `/odom_rf2o` | RF2O | Odométrie basée LIDAR (scan-matching) |
| `/odom_motor` | Motor Odom | Odométrie moteur (dead-reckoning) |
| `/odom` | Fusion | **Odométrie fusionnée** (utilisée par SLAM) |
| `/map` | SLAM Toolbox | Carte d'occupation |
| `/map_metadata` | SLAM Toolbox | Métadonnées de la carte |

## 🔧 Configuration

- **RF2O**: 50Hz, scan-matching Ceres
- **Motor Odom**: 50Hz, modèle cinématique simple
- **Fusion**: RF2O=80%, Motor=20%
- **SLAM**: Async, résolution 2.5cm, loop closure 2.0m

## ✅ Statut

- ✓ RF2O compilé et fonctionnel
- ✓ Motor odometry réorienté vers `/odom_motor`
- ✓ Fusion node crée
- ✓ SLAM reconfigure pour utiliser `/odom`
- ✓ Stack démarre sans erreurs

## ⚙️ Prochaines étapes

1. Tester avec micro-ROS actif (LIDAR en live)
2. Vérifier RF2O reçoit les scans
3. Valider fusion des odométries
4. Tester mapping en mouvement
