# 🗺️ SLAM Toolbox Setup - Explication complète

**Date:** Février 2026  
**Basé sur:** PDF du cours du Professeur Naila Bouchemal

---

## 📚 Ce que dit le cours

Le PDF du professeur recommande **clairement**:
1. ❌ Ne PAS réinventer la roue avec un SLAM custom
2. ✅ Utiliser **SLAM Toolbox** - librairie ROS2 prête à l'emploi
3. ✅ Utiliser **Nav2** - framework complet pour navigation autonome

---

## 🎯 À quoi sert SLAM Toolbox?

### **SLAM = Simultaneous Localization And Mapping**

**En français simple:**
- **Mapping** = Construire une carte de l'environnement en temps réel
- **Localization** = Savoir où le robot se trouve dans la carte
- **Simultaneous** = Faire les deux en même temps

### **Concrètement, SLAM Toolbox fait:**

```
Scans LIDAR (capteur)
    ↓
SLAM Toolbox
    ├─→ Accumule les scans dans une carte STATIQUE (frame /map)
    ├─→ Estime la pose du robot (x, y, theta)
    ├─→ Détecte les boucles fermées (loop closure)
    └─→ Améliore continuellement la précision
    ↓
/map (STATIQUE)
/slam_toolbox/pose (où est le robot)
```

---

## 🛠️ Configuration pour ton robot

Fichier: `config/slam_toolbox_params.yaml`

### **Paramètres clés:**

| Paramètre | Valeur | Signification |
|-----------|--------|---------------|
| `scan_topic` | `/scan` | Topic LIDAR (d'où viennent les scans) |
| `resolution` | `0.05` | 5cm par cellule (très précis) |
| `minimum_travel_distance` | `0.1m` | Attend 10cm avant d'ajouter un scan |
| `minimum_travel_heading` | `0.0998rad` | Attend 5.7° avant d'ajouter un scan |
| `map_update_interval` | `0.2s` | Publie la map 5 fois par seconde |
| `loop_search_distance` | `5.0m` | Cherche les boucles à 5m |
| `do_loop_closing` | `true` | Détecte et ferme les boucles |

### **Pourquoi ces valeurs?**

**`minimum_travel_distance` = 0.1m**
- ✅ Évite d'ajouter des scans identiques à la map
- ✅ Économise la mémoire
- ✅ Réduit les calculs

**`minimum_travel_heading` = 0.0998rad (~5.7°)**
- ✅ Force le robot à tourner avant d'ajouter un scan
- ✅ Réduit la redondance angulaire
- ✅ Améliore la qualité de la map

**`loop_search_distance` = 5.0m**
- ✅ Détecte quand le robot revient à un endroit connu
- ✅ Corrige les petites erreurs d'odométrie
- ✅ Rend la map plus cohérente

---

## 🚀 Comment ça marche en pratique?

### **Startup (start_stack.sh):**

```bash
[1/7] Micro-ROS Agent (ESP32)
      └─→ Lit: /odom, /imu, /scan

[2/7] RViz2 (Visualisation)
      └─→ Affiche la map et la pose du robot

[3/7] IMU FIR Filter (Filtre)
      └─→ Nettoie le bruit du LIDAR à 20Hz

[4/7] Motor Odometry (Odométrie moteur)
      └─→ Calcule: déplacement du robot

[5/7] SLAM Toolbox (SLAM)
      └─→ Fusionne: LIDAR + Odométrie
      └─→ Publie: /map, /slam_toolbox/pose

[6/7] Kalman Filter Fusion (Fusion)
      └─→ Fusionne: Odom + IMU + SLAM
      └─→ Publie: /odom_filtered (meilleure pose)
```

### **Flux de données:**

```
ESP32 (Capteurs)
  ├─ /imu/data
  ├─ /scan (LIDAR brut)
  └─ Odométrie moteur
    ↓
[IMU FIR Filter] → /imu/data_filtered
    ↓
[Motor Odom Node] → /odom (odométrie brute)
    ↓
[SLAM Toolbox] 
  Entrées: /scan, /odom
  Sorties:
    ├─ /map (grid 5cm, STATIQUE)
    ├─ /slam_toolbox/pose (pose estimée)
    └─ /tf (map → odom → base_link)
    ↓
[Kalman Fusion]
  Entrées: /odom, /imu/data_filtered, /slam_toolbox/pose
  Sortie: /odom_filtered (meilleure estimation pose)
    ↓
Pour Nav2 / Navigation autonome
```

---

## 📊 Topics importants

Après lancement de `start_stack.sh`:

```bash
# Entrées (capteurs)
/scan                      # LIDAR brut (10 Hz)
/odom                      # Odométrie moteur brute
/imu/data                  # IMU brute

# Sorties du filtre
/imu/data_filtered         # IMU filtrée (20Hz cutoff)

# Sorties de SLAM Toolbox
/map                       # Carte d'occupation (STATIQUE!)
/slam_toolbox/pose         # Pose estimée par SLAM

# Sorties de fusion Kalman
/odom_filtered             # Meilleure estimation pose

# Transforms (TF)
map → odom → base_link
```

---

## 🔍 Différence: SLAM Toolbox vs Code Custom

### **SLAM Toolbox (Production):**
```python
✅ Robuste, testé en industrie
✅ Optimisé en C++
✅ Gère loop closure automatiquement
✅ Detects mouvements circulaires
✅ Utilise graphes (graph-SLAM)
✅ Peu de tuning nécessaire
```

### **Custom SLAM (Développement):**
```
❌ Sujette aux bugs
❌ Lente (Python)
❌ Pas de loop closure
❌ Map suivait le robot
❌ Grosse gestion manuelle
❌ Grosse tuning nécessaire
```

---

## 🎮 Commandes utiles

### **Lancer le stack complet:**
```bash
bash start_stack.sh
```

### **Arrêter proprement:**
```bash
Ctrl+C
```

### **Vérifier que SLAM Toolbox publie:**
```bash
ros2 topic echo /slam_toolbox/pose          # Pose estimée
ros2 topic echo /map                        # Carte (grille)
ros2 topic hz /slam_toolbox/pose            # Fréquence
```

### **Visualiser en RViz:**
```bash
# /map layer
# /slam_toolbox/pose
# /base_link, /map frames
```

---

## 📈 Prochaines étapes (Nav2)

Avec SLAM Toolbox qui fournit une **map STATIQUE** et une **pose estimée**, 
tu peux maintenant utiliser **Nav2** pour:

1. **Planification** → Calculer des trajets
2. **Contrôle** → Suivre les trajets  
3. **Évitement** → Éviter les obstacles dynamiques
4. **Navigation autonome** complète

Voir: `start_nav2_stack.sh` (si tu la as)

---

## 💡 Points clés à retenir

| Concept | Avant (custom) | Maintenant (Toolbox) |
|---------|---|---|
| **Map** | Suivait le robot | STATIQUE (frame /map) |
| **Pose** | Confus | SLAM Toolbox.pose |
| **Loop closing** | Aucun | Automatique |
| **Maintenance** | Difficile | Simple (config YAML) |
| **Production** | Non | Oui |

---

## 📚 Ressources officielles

- **PDF Cours:** `/home/matheo/Downloads/PT_ECEPILOT_2025_SLAM.pdf`
- **SLAM Toolbox Docs:** https://github.com/StanleyLeleLy/slam_toolbox
- **Nav2 Docs:** https://docs.nav2.org/
- **Tutoriels:** YouTube "ROS2 SLAM Toolbox"

---

**Fait: 11 Février 2026**  
**Configuré pour:** ECE Pilot - Autonomous Car Project
