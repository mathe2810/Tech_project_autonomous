# 🔍 AUDIT COMPLET - SÉCURITÉ ET PERFORMANCE

**Date**: 16 Feb 2026  
**Problème**: Robot "fonce dans le mur", mapping horrible en rotation

---

## 📊 RÉSUMÉ EXÉCUTIF

| Composant | Problème | Sévérité | Fix |
|-----------|----------|----------|-----|
| **d_stop** | 0.25m TROP petit | 🔴 **CRITIQUE** | → 0.35m |
| **d_slow** | 0.60m pas assez progressif | 🔴 **CRITIQUE** | → 0.80m |
| **w_max** | 2.5 rad/s (143°/s) | 🔴 **CRITIQUE** | → 0.5 rad/s max |
| **kp** | 2.0 trop agressif | 🟠 **HAUTE** | → 0.8 |
| **minimum_travel_heading** | 0.1 rad trop petit | 🟠 **HAUTE** | → 0.15 rad |
| **emergency_hold_time** | 0.35s trop court | 🟠 **HAUTE** | → 0.8s |
| **Front safety zone** | front_half_deg=10° | 🟡 **MOYEN** | → 15° |

---

## 🔴 CRITIQUE #1: Distance d'arrêt insuffisante

### Problème:
```
d_stop = 0.25m = 25cm

Temps de réaction typique @ 0.6 m/s:
  - SLAM: 100-200ms
  - Wall-follow loop: 100ms
  - Moteur: 50ms
  = Minimum 250ms total = 15cm de drift
  
Robot s'arrête à 25cm mais a déjà avancé 15cm depuis la détection!
```

### Pourquoi c'est dangereux:
- À 60 cm/s, le robot fait **6cm par 100ms**
- Détection obstacle → 100ms → robot a avancé 6cm
- Détection obstacle à 25cm → robot est déjà à 19cm
- Encore 1-2 cycles et COLLISION

### Fix:
```yaml
d_stop:  0.35m  # Au lieu de 0.25m (+40cm de buffer)
d_slow:  0.80m  # Au lieu de 0.60m (+ progressif)
```

Cela donne:
- **350mm d'arrêt de sécurité** (réaliste avec délai)
- **800mm de ralentissement progressif** (lisse)

---

## 🔴 CRITIQUE #2: w_max = 2.5 rad/s EST UNE BOMBE

### Calcul d'impact:
```
w_max = 2.5 rad/s
    = 2.5 × (180/π) deg/s
    = 143°/s  ← EXTRÊME

À 10Hz (SLAM publish rate):
    143° / 10 = 14.3°/frame
    
SLAM paramètre: minimum_travel_heading = 0.1 rad = 5.7°
Donc SLAM enregistre une pose tous les 5.7°, mais il reçoit 14.3° par frame!
    = 14.3° / 5.7° = 2.5 frames ignorées entre chaque pose SLAM!
    
= MAPPING DISCONTINU, pose estimée mauvaise
```

### Comparaison:
```
Version actuelle (v2):
  w_max = 2.5 rad/s = 143°/s → SLAM PERD LA POSE

ROS2 standard:
  max_angular_vel = 1.5 rad/s (nav2_params) = 86°/s

Robots mobiles réels:
  Turtlebot3: max 2.84 rad/s (mais a des encodeurs!)
  Lidar-only robots: 0.3-0.8 rad/s (pas d'odométrie fiable)
```

### Fix:
```python
# Limiter la vitesse angulaire DRASTIQUEMENT
w_max = 0.50  # rad/s = 28°/s
          # SLAM voit: 28° / 5.7° = 4.9 poses/sec = CONTINU!

# Dans emergency, utiliser vrai w_max:
w_emergency = 1.0  # rad/s = 57°/s (juste pour échapper)
```

---

## 🟠 HAUTE: PID instable

### Problème:
```python
kp = 2.0   # Gain proportionnel énorme
kd = 0.4   # Dérivée faible
```

**Exemple de scenario instable:**
- Robot très proche du mur (d = 0.1m)
- d_ref = 0.45m
- Erreur: e = 0.45 - 0.1 = 0.35m ← MASSIVE
- w = kp × e = 2.0 × 0.35 = **0.7 rad/s** (direct!)
- Avec w_max=2.5, clampe à 2.5 → rotation chaotique
- Puis d'un coup proche mur opposé
- Mêmes calculs inverses
- **OSCILLATION FOLLE**

### Fix:
```python
kp = 0.8   # Moins agressif
kd = 0.6   # Meilleure dérivée
```

---

## 🟠 HAUTE: Minimum travel heading trop permissif

### Problème:
```yaml
minimum_travel_heading: 0.1  # rad = 5.7°

Avec w_max = 2.5 rad/s et SLAM @ 10Hz:
  Rotation par frame = 2.5 / 10 = 0.25 rad = 14.3°
  
SLAM attend 5.7° mais robot tourne 14.3° par frame
= Plusieurs frames "ignorées" par SLAM
= Map clairsemée, pose est "jumping"
```

### Fix:
```yaml
minimum_travel_heading: 0.15  # rad = 8.6°
  # Avec w_max=0.5: 0.5/10 = 0.05 rad = 2.9°
  # SLAM voit: 2.9° < 8.6° → pas d'update → OK
  # Quand update: bien continu
```

---

## 🟠 HAUTE: Emergency latch trop court

### Problème:
```python
emergency_hold_time = 0.35  # secondes

À 60 cm/s, en 350ms le robot peut avancer 21cm!
Si detecté à d_stop=0.25m, déjà dans le mur en 2 cycles!
```

### Fix:
```python
emergency_hold_time = 0.8  # secondes
  # Donne 48cm de drift protection
  # À 0.5 rad/s, robot tourne 26° = dégagé du mur
```

---

## 🟡 MOYEN: Front safety zone trop étroite

### Problème:
```python
front_half_deg = 10.0  # ±10° devant

Robots petits, LIDAR peuvent avoir angles morts
10° c'est ≈ 8cm sur les côtés à 50cm = obstacle caché possible
```

### Fix:
```python
front_half_deg = 15.0  # ±15° = meilleure couverture
```

---

## 📋 CHECKLIST CONFIGURATION ACTUELLES

### ✅ SLAM Toolbox params (CORRECTS):
```yaml
scan_queue_size: 1              # ✓ Queue courte
throttle_scans: 1               # ✓ Traite chaque scan
minimum_travel_distance: 0.1    # ✓ OK
minimum_travel_heading: 0.1     # ⚠️ À augmenter à 0.15
use_scan_matching: true         # ✓ Obligatoire
```

### ✅ Scan Restamper (CORRECT):
```python
# Fixe timestamp sync SLAM/LIDAR ✓ WORKING
```

### ✅ Motor Odom Node (OK):
```
linear_vel_scale: 0.8
angular_vel_scale: 0.9
# Sans encodeurs, dérive inévitable - mais SLAM le corrige ✓
```

### ✅ Simple EKF (WORKING):
```
Fuse motor_odom + IMU gyro + SLAM pose
# Correct, pas le problème ✓
```

### ❌ Wall-Follow params (CRITIQUES):
```python
# TOUS les paramètres ci-dessus doivent être changés!
```

---

## 🎯 FIXED VALUES (À COPIER):

```python
# ===== SAFETY DISTANCES (FIXED) =====
d_stop = 0.35   # +40% marge (from 0.25)
d_slow = 0.80   # +33% plus progressif (from 0.60)

# ===== SPEED LIMITS (FIXED) =====
v_max = 0.50    # Ralenti un peu (from 0.60)
v_min = 0.10    # Ralenti un peu (from 0.15)
w_max = 0.50    # CRITIQUE: -80% (from 2.5!)
w_emergency = 1.0  # Pour l'urgence seulement

# ===== PID TUNING (FIXED) =====
kp = 0.8        # Moins agressif (from 2.0)
kd = 0.6        # Meilleure dérivée (from 0.4)

# ===== LIDAR PARAMS (FIXED) =====
front_half_deg = 15.0        # +50% couverture (from 10.0)
fallback_half_deg = 25.0     # +25% (from 20.0)

# ===== SAFETY TIMING (FIXED) =====
emergency_hold_time = 0.8    # +130% (from 0.35)
slam_settle_time = 0.5       # Min pause entre mouvements
```

---

## 🧪 TEST PROGRESSION:

1. **Test statique**: Arrêter le robot devant mur, vérifier d_stop
2. **Test linéaire**: Avancer lentement droit, checker mapping
3. **Test rotation lente**: Tourner @ 0.5 rad/s, vérifier SLAM continue
4. **Test wall-follow**: Contourner mur @ vitesse RÉDUITE
5. **Test stress**: Obstacle soudain = check emergency

---

## 📍 FICHIERS À MODIFIER:

1. **naif_autonome.py** → Lines 82-95 (params)
2. **algo_naif_v2.py** → Lines 82-97 (params + emergency)
3. **config/slam_toolbox_params.yaml** → Line 18 (minimum_travel_heading)

