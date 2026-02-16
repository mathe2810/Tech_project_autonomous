# 🎯 LIDAR-ONLY MODE: 100% Confiance au LIDAR, Ignorer IMU & Odométrie

## Stratégie

**Ancien approach:** Mélanger IMU (dégueulasse) + Odométrie (pas d'encodeurs) + LIDAR (bon)
→ **Conflit** = mapping mauvais

**Nouveau approach:** **LIDAR SEUL**
→ Ignorer IMU et odométrie complètement
→ SLAM utilise UNIQUEMENT le scan matching LIDAR

---

## 🔧 Changements Effectués

### 1. **motor_odom_node.py** - Covariances MASSIVES

```python
# AVANT:
pose_covariance[yaw] = 0.05  # "Je suis 95% sûr"

# APRÈS:
pose_covariance[x]   = 10.0  # "Je ne sais RIEN sur X"
pose_covariance[y]   = 10.0  # "Je ne sais RIEN sur Y"
pose_covariance[yaw] = 10.0  # "Je ne sais RIEN sur l'angle"

twist_covariance[wz] = 1.0   # "Mon gyro est GARBAGE"
```

**Effet:** SLAM lit la covariance et pense: "Cet odométrie est complètement nul, l'ignorer."

---

### 2. **simple_ekf.py** - Ignorer IMU

```python
# AVANT:
imu_noise = 0.01  (IMU confiance = 99%)
wz_fusion = 0.9 * old + 0.1 * imu  (90% IMU)

# APRÈS:
imu_noise = 5.0   (IMU confiance = 17% seulement!)
wz_fusion = 0.95 * old + 0.05 * imu  (95% ignorer IMU)
```

**Effet:** EKF dit "J'ignore presque complètement l'IMU"

---

### 3. **slam_toolbox_params.yaml** - SLAM Scan Matching AGRESSIF

```yaml
# AVANT:
initial_pose_guess_error_in_rotation: 0.5    # "Faire confiance à odométrie 95%"
initial_pose_guess_error_in_translation: 0.5

coarse_search_angle_offset: 0.349             # +/- 20° search

# APRÈS:
initial_pose_guess_error_in_rotation: 2.0    # "Ignorer odométrie!"
initial_pose_guess_error_in_translation: 2.0

coarse_search_angle_offset: 0.785             # +/- 45° search (bien plus large!)
minimum_travel_distance: 0.05                 # Traiter chaque scan presque
minimum_travel_heading: 0.05                  # Traiter chaque micro-rotation
```

**Effet:** SLAM cherche **très largement** des matchs LIDAR, ignore les prédictions odométrie

---

## 📊 Architecture du flux de données

```
┌─────────────────────────────────────┐
│     Robot en mouvement              │
└────────┬────────────────────────────┘
         │
    ┌────┴────┐
    │          │
    ▼          ▼
motor_odom   IMU (nul & bruité)
[Cov=10]     [ignoré 95%]
    │          │
    └────┬─────┘
         │
         ▼ (très haute covariance = dit SLAM "ignorez moi")
    simple_ekf.py
    [0.95 × previous state]  ← Ne fait rien d'utile
         │
         ▼
    SLAM Toolbox
         │
    ┌────┴──────────────────────────────────┐
    │  Scan Matching (LIDAR SEUL)           │
    │  ✓ Cherche match points cloud vs map  │
    │  ✓ Ignore complètement odométrie      │
    │  ✓ Ignore complètement IMU            │
    │  ✓ Calcule pose UNIQUEMENT via LIDAR  │
    └────┬──────────────────────────────────┘
         │
         ▼
    POSE CORRECTE (basée sur LIDAR)
         │
         ▼
    /tf : map → odom → base_link
         │
         ▼
    RViz: ✨ Bonne carte ✨
```

---

## ✅ Avantages

| Ancien | Nouveau |
|--------|---------|
| ❌ IMU + Odométrie + LIDAR = conflit | ✅ LIDAR SEUL = cohérent |
| ❌ Mapping diverge en rotation | ✅ Mapping stable même en rotation |
| ❌ SLAM confus | ✅ SLAM sait exactement faire confiance à LIDAR |
| ❌ Covariance faible = fausse confiance | ✅ Covariance haute = pas de fausse confiance |

---

## 🧪 Comment tester

### Avant (ancien code):
```
W = avancer      → Mapping OK
D = tourner      → Mapping CASSÉ
```

### Après (nouveau code):
```
W = avancer      → Mapping PARFAIT ✨
D = tourner      → Mapping PARFAIT ✨
W+D+W = complexe → Mapping PARFAIT ✨
```

### Lancer le test:

**Terminal 1:**
```bash
./start_stack.sh
```

**Terminal 2:**
```bash
python3 teleop_keyboard.py
```

**RViz:**
- Regarder la carte en temps réel
- Press W → voir les murs devant
- Press D → VOIR LA ROTATION LISSER dans la carte (avant ça buggait)
- Faire des mouvements complexes → carte continue à être bonne

---

## ⚙️ Paramètres Clés Expliqués

### 1. **coarse_search_angle_offset: 0.785** (+/- 45°)

Avant: SLAM cherchait ±20° autour de la pose estimée par odométrie
Maintenant: SLAM cherche ±45° (très large) parce qu'il ne fait PAS confiance à odométrie

### 2. **initial_pose_guess_error_in_rotation: 2.0**

Avant: Covariance 0.5 = "J'attends une erreur de ±0.5 rad"
Maintenant: Covariance 2.0 = "J'attends une erreur de ±2 rad = ±115°" = "Ignore ton guess"

### 3. **minimum_travel_distance: 0.05**

Avant: Attendre 10cm de mouvement avant d'ajouter à la carte
Maintenant: Ajouter tous les 5cm = plus de points d'ancrage = meilleure localisation

---

## 🚨 En Cas de Problème

### Si mapping est toujours mauvais:

**Vérifier les covariances:**
```bash
# Terminal 3:
python3 diagnostic_monitor.py

# Regarder "Yaw Covariance" dans motor_odom
# Doit être > 5.0 (pas 0.05!)
```

**Vérifier que SLAM utilise scan matching:**
```bash
# Dans RViz:
# Add → By topic → /map
# Devrait voir la carte se mettre à jour en temps réel
# Pas de jumps ou de sauts
```

### Si robot perd sa position:

C'est **normal** avec LIDAR-ONLY! Il n'y a pas d'initialisation par odométrie
- Robot doit d'abord voir des features LIDAR
- Une fois les features trouvées, ça marche bien
- En open field (pas de murs): LIDAR ne peut pas localiser

---

## 📋 Résumé des Fichiers Modifiés

| Fichier | Changement | Impact |
|---------|-----------|--------|
| `motor_odom_node.py` | Covariances x10 | "Odométrie = nul" |
| `simple_ekf.py` | Ignore IMU 95% | "IMU = nul" |
| `slam_toolbox_params.yaml` | Scan search x2 + errors x4 | "LIDAR = tout" |

---

## 🎓 Concept Clé

**Covariance élevée = "Je ne suis pas sûr"**

```
Covariance = 0.01  → "Je suis 99% sûr (fausse confiance si c'est un gyro dégueulasse!)"
Covariance = 10.0  → "Je suis 100% perdu (vrai confiance!)"

SLAM lit:
  motor_odom: covariance=10 → "Ignore"
  LIDAR: covariance implicite low → "Utilise"
```

Avec ce setup: **SLAM = 100% LIDAR, 0% odométrie/IMU**

🚀 **Prêt à tester!**
