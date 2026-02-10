# 📐 Diagramme et Visualisation de l'Algorithme

## Vue de haut: Principe de base

```
                    AVANT (0°)
                       |
                       ▲
        315°--------ROBOT--------45°
        /              |              \
    GAUCHE         |LIDAR|          DROITE
      /              |   |              \
  270°             CAPTEUR            90°
     \              |   |              /
      \             |   |             /
       180° --------+---+--------135°
                    |
                  ARRIÈRE

ZONEs:
  GAUCHE: angles 45° à 315° (180 points)
  DROITE: angles 0-45° + 315-360° (180 points)
```

## Algorithme simplifié

```
Récupérer les 360 points du LIDAR
        │
        ▼
┌─────────────────────┐
│ Compter les points  │
│ dans chaque zone:   │
│ - left_count        │
│ - right_count       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Calculer balance:   │
│ balance =           │
│ left_count -        │
│ right_count         │
└──────────┬──────────┘
           │
           ▼
┌──────────────────────────────┐
│ Générer commandes:           │
│ throttle = forward_speed     │
│ steering = balance * 255     │
│           / (valid_count+1)  │
└──────────┬───────────────────┘
           │
           ▼
   Envoyer aux moteurs
        │
        ▼
   🤖 Robot navigue!
```

## Diagramme de décision: Où tourner?

```
      Observations LIDAR
            │
            ▼
    ┌───────────────────────┐
    │ left_count > right    │
    │ count?                │
    │  (Plus à gauche)      │
    └───────┬───────────────┘
            │
    ┌───────┴───────┐
    │ YES      NO   │
    ▼              ▼
┌──────┐      ┌──────┐
│ Tourne│      │Tourne│
│ DROITE│      │GAUCHE│
│ (fuit)│      │(fuit)│
└──────┘      └──────┘
    │              │
    └──────┬───────┘
           │
           ▼
  ┌─────────────────┐
  │ Distance min    │
  │ devant < 30cm?  │
  └────────┬────────┘
           │
    ┌──────┴──────┐
    │ YES    NO   │
    ▼             ▼
┌──────────┐  ┌─────────┐
│ RECULE & │  │ AVANCE  │
│ TOURNE D │  │ DROIT   │
└──────────┘  └─────────┘
```

## Visualisation sur grille 5x5

### Scénario 1: Espace libre
```
Grille LIDAR vue du haut:

    0°  1°  2°  3° ... 358° 359°
GAUCHE                      DROITE
  ████████████████████████████
  ████████                ████
  ██                        ██
  ██                        ██
  ████████                ████
  ████████████████████████████

Résultat: left_count ≈ right_count
          balance ≈ 0
          steering ≈ 0
          
👉 Le robot avance tout droit! ✓
```

### Scénario 2: Mur à gauche
```
Grille LIDAR:

GAUCHE                      DROITE
  ██████████████████████
  ██                    ████
  ██                    ██  ██
  ██                    ██  ██
  ██                    ████
  ██████████████████████████

Résultat: left_count > right_count
          balance > 0
          steering > 0
          
👉 Le robot tourne à droite (vers l'espace libre)! ✓
```

### Scénario 3: Obstacle devant
```
Grille LIDAR:

        AVANT (danger!)
           ██████
           ██████
    ████████████████████
    ██              ██
    ██              ██

Résultat: min_front_dist < DANGER_THRESHOLD
          
👉 Le robot recule et tourne droite agressivement! ✓
```

## Graphique: Balance vs Steering

```
Steering (commande moteur)
     │
 128 │     ╱
     │    ╱
  64 │   ╱
     │  ╱
   0 ├─────────── balance (left_count - right_count)
     │ ╲
 -64 │  ╲
     │   ╲
-128 │    ╲
     └────────────────
    -200 -100  0  100  200

Interprétation:
- Pente positive: Plus de déséquilibre = plus de braquage
- Point (0,0): Équilibré = tout droit
- Marges [-128, +128]: Saturé = virage serré max
```

## Architecture complète

```
┌──────────────────────────────────────────────────────────┐
│                    ROBOT AUTONOME                        │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  ┌────────────┐      ┌────────────┐                    │
│  │   LIDAR    │ ───→ │ lidarTask  │                    │
│  │   SERIAL   │      │ (Tâche 4)  │                    │
│  └────────────┘      └──────┬─────┘                    │
│                             │                           │
│                       ┌─────▼────────┐                 │
│                       │ lidar_buffers│                 │
│                       │  (double-buf)│                 │
│                       └─────┬────────┘                 │
│                             │                           │
│                       ┌─────▼──────────────────────┐   │
│                       │ autonomousTrajectoryTask   │   │
│                       │ (Tâche 2, 20 Hz)           │   │
│                       │                            │   │
│                       │ 1. Lire lidar_buffers      │   │
│                       │ 2. naive_trajectory()      │   │
│                       │ 3. smart_trajectory()      │   │
│                       │ 4. motors_drive()          │   │
│                       └─────┬──────────────────────┘   │
│                             │                           │
│                       ┌─────▼────────────┐             │
│                       │  Motor Drivers   │             │
│                       │  (PWM) L/R       │             │
│                       └─────┬────────────┘             │
│                             │                           │
│  ┌────────────┐      ┌──────▼──────┐                 │
│  │   MOTEURS  │ ←─── │ Pont H      │                 │
│  │  (Gauche)  │      │ (L298N/etc) │                 │
│  │  (Droite)  │      │             │                 │
│  └────────────┘      └─────────────┘                 │
│                                                        │
│  Exécution parallèle:                                │
│  ├─ lidarTask (core 1, priorité 4)                  │
│  ├─ imuTask (core 1, priorité 3)                    │
│  ├─ autonomousTrajectoryTask (core 0, priorité 2) ◄─┤ NOUS!
│  ├─ publishTask (core 0, priorité 2)                │
│  └─ loop (core 1)                                    │
│                                                        │
└──────────────────────────────────────────────────────────┘
```

## Diagramme temporel: Timing

```
Temps
 ▲
 │
 │ autonomousTrajectoryTask (20 Hz, 50ms)
 │
100ms├─────────────────────────────────────────────────
     │ Iteration 1  │  Iter 2  │  Iter 3  │  Iter 4
 50ms├──┬─────────────┬────────┬─────────┬────────────
     │  │             │        │         │
  0ms└──┴─────────────┴────────┴─────────┴────────────
        │ Traitement │ Attente │ PWM     │
        │   (< 1ms)  │ (49ms)  │  envoyé │
        │            │         │         │
        │Compute     │Wait     │Apply    │
        │lidar       │FreeRTOS │motors   │
```

## États du robot: Machine d'état

```
┌──────────────┐
│  ROBOT STOP  │◄─────────┐
│ (motors=0)   │           │ autonomous_mode = false
└───────┬──────┘           │
        │                  │
        │ autonomous_mode = true
        ▼
┌──────────────────────────┐
│  ROBOT AUTONOME          │
│ (navigating)             │
├──────────────────────────┤
│ • Lit LIDAR              │◄───┐
│ • Calcule direction      │    │
│ • Envoie PWM aux moteurs │    │
│ • Avance/Tourne/Recule   │    │
└───────┬──────────────────┘    │
        │                       │
        └───────── Boucle ──────┘
            (toutes les 50ms)
```

## Heatmap: Distribution des décisions

```
Exemple: Exploration d'un espace L-shape

ZONE 1 (Vertical)   ZONE 2 (Horizontal)
Avance droit        ▲ Tourne droite
        │           │
════════╋═══════════●═════════════
        │           │
Espace libre        Espace libre

Trace du robot:
●───────────┐
│           │
│  ●──●     │  LIDAR détecte le tournant
│  │  │     │  et tourne naturellement!
│  └──┘     │
│        ●  │
│        │  │
│        └──┘
```

## Comparaison: Naïf vs Smart

```
Scénario: Obstacle proche devant

       NAIVE:                    SMART:
    ┌─────────┐               ┌─────────┐
    │ Avance  │               │ Détecte │
    │ Recule  │ ─→ Collision  │ Distance│
    │ Tourne  │               └────┬────┘
    └─────────┘                    │
                                   ▼
                            ┌──────────────┐
                            │ min_dist<30cm│
                            │ Recule + Turn│
                            │ Évite!  ✓    │
                            └──────────────┘
```

---

**Visualisations créées pour mieux comprendre l'algorithme**
