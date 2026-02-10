# Algorithme de Trajectoire Naïve

## Vue d'ensemble

Cet algorithme contrôle le robot en analysant la distribution des points détectés par le lidar. Le principe est simple et intuitif:

- **Plus de points à gauche → Le robot tourne à droite**
- **Plus de points à droite → Le robot tourne à gauche**

Cela permet au robot d'éviter naturellement les obstacles en se dirigeant vers les espaces libres.

## Fonctionnement détaillé

### 1. Fonction `naive_trajectory()`

C'est la base de l'algorithme. Elle divise le champ du lidar en deux zones:

**Zone gauche**: angles 45° à 315° (moitié gauche du robot)
**Zone droite**: angles 315° à 45° (moitié droite du robot)

```
        0° (AVANT)
        |
  315°--+--45°
  /     |     \
270°    |      90°
  \     |     /
  225° ROBOT 135°
       (VUE DU HAUT)

Gauche: angles 45-315°
Droite: angles 315-360° + 0-45°
```

#### Algorithme:

1. **Compter les points**: 
   - `left_count`: nombre de points détectés à gauche
   - `right_count`: nombre de points détectés à droite
   - Uniquement les points entre 0 et `max_range` sont comptés

2. **Calculer le balance**:
   ```
   balance = left_count - right_count
   ```

3. **Commandes moteur**:
   - **Throttle (avant/arrière)**: Vitesse constante vers l'avant
   - **Steering (direction)**: Proportionnel au déséquilibre
     - Si `balance > 0` (plus à gauche) → steering positif (tourne droite)
     - Si `balance < 0` (plus à droite) → steering négatif (tourne gauche)
     - Si `balance ≈ 0` (équilibré) → steering ≈ 0 (avance droit)

### 2. Fonction `smart_trajectory()` (Version améliorée)

Extension de l'algorithme naïf avec détection d'obstacles proches:

- Détecte les obstacles **devant le robot** (angles 330-30°)
- Si un obstacle est détecté à distance < `danger_threshold` (30 cm par défaut):
  - **Recule** immédiatement
  - **Tourne à droite** agressivement
- Sinon utilise l'algorithme naïf normal

## Code d'utilisation

### Activation du mode autonome

Dans votre code ou via une interface de contrôle:

```cpp
// Activer le mode autonome
autonomous_mode = true;
autonomous_throttle = 150;  // Vitesse avant (0-255)

// Plus tard: désactiver
autonomous_mode = false;
motors_stop();  // Arrêter le robot
```

### Tâche `autonomousTrajectoryTask`

Cette tâche FreeRTOS:
- S'exécute à **20 Hz** (tous les 50ms)
- Récupère les données du lidar
- Calcule la commande de trajectoire
- Envoie les commandes aux moteurs via `motors_drive()`

## Paramètres configurables

Dans la fonction `autonomousTrajectoryTask()`:

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| `MAX_RANGE` | 300.0f cm | Distance maximale du lidar considérée |
| `DANGER_THRESHOLD` | 30.0f cm | Distance min avant obstacle (recul automatique) |
| `BASE_THROTTLE` | 150 | Vitesse avant (0-255) |

## Limitations et améliorations possibles

### Limitations actuelles:
1. **Pas de mémoire**: Chaque décision est prise indépendamment
2. **Pas d'optimisation**: Pas de planification de trajectoire long terme
3. **Sensible au bruit**: Les faux points du lidar peuvent affecter la direction
4. **Oscillations possibles**: Si les obstacles sont symétriques

### Améliorations futures:
1. **Filtrage temporel**: Moyenne mobile des dernières décisions
2. **Zones de priorité**: Donner plus de poids aux points proches
3. **Détection d'impasse**: Reconnaître les situations bloquées
4. **Mémoire spatiale**: Se souvenir des chemins déjà explorés
5. **Accélération progressive**: Adapter la vitesse selon les obstacles

## Exemple: Sortie de labyrinth

Avec cet algorithme:
1. Le robot avance tout droit si l'espace est libre
2. Quand il approche un mur:
   - L'un des côtés verra plus de points
   - Il tournera naturellement de l'autre côté
3. Pas besoin de logique complexe, juste la physique de l'espace libre!

## Intégration avec ROS 2

- Le lidar publie les données via `/scan` (topic ROS2)
- La tâche autonome lit directement les buffers internes
- Pas de latence ROS - traitement temps réel
- Les commandes moteurs sont appliquées directement

## Structure de contrôle

```
┌─────────────────┐
│  Lidar SERIAL   │
└────────┬────────┘
         │ (données brutes)
         ▼
┌─────────────────┐
│  lidarTask      │ (Tâche 4, core 1)
└────────┬────────┘
         │ (met à jour lidar_buffers)
         ▼
┌─────────────────────────────────────┐
│  autonomousTrajectoryTask           │ (Tâche 2, core 0)
│  • Lit lidar_buffers                │
│  • Calcule naive_trajectory()       │
│  • Appelle motors_drive()           │
└────────┬────────────────────────────┘
         │ (commandes PWM)
         ▼
┌─────────────────┐
│  Motor drivers  │
└─────────────────┘
```

## Debugging

La tâche autonome affiche toutes les secondes:
```
[TRAJ] throttle=150 steering=32 | left=45 right=38 total=83
```

- `throttle`: Vitesse avant actuellement appliquée
- `steering`: Angle de braquage (-128 à +128)
- `left`: Nombre de points détectés à gauche
- `right`: Nombre de points détectés à droite
- `total`: Nombre total de points valides

## Exemples de scénarios

### Situation 1: Couloir droit
```
LIDAR vue de haut:

LEFT  │  FRONT  │  RIGHT
█████ │  ▓▓▓▓▓  │ █████
█████ │  ▓▓▓▓▓  │ █████

Résultat: left_count ≈ right_count → steering ≈ 0 → avance droit ✓
```

### Situation 2: Mur à gauche
```
LEFT  │  FRONT  │  RIGHT
██ ▓▓ │  ▓▓▓▓▓  │ █████
██ ▓▓ │  ▓▓▓▓▓  │ █████

Résultat: left_count > right_count → steering > 0 → tourne droite ✓
```

### Situation 3: Obstacle devant
```
LEFT  │  FRONT  │  RIGHT
█████ │  ███░░░ │ █████
█████ │  ███░░░ │ █████

Résultat: min_front_dist < danger_threshold → recule et tourne ✓
```
