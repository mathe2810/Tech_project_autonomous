# Algorithme de Trajectoire Naïve - Vue d'ensemble

## 🤖 Description

Implémentation d'un **algorithme de navigation autonome naïf** pour un robot mobile basé sur les données du **LIDAR**.

**Concept clé**: L'algorithme compte les obstacles à gauche et à droite, puis tourne vers l'espace le plus libre.

```
Si plus d'obstacles à gauche  → Tourne à droite
Si plus d'obstacles à droite  → Tourne à gauche
Si équilibré                  → Avance tout droit
```

## 📋 Fichiers créés/modifiés

| Fichier | Description |
|---------|-------------|
| `main.cpp` | Code principal avec l'implémentation de l'algorithme |
| `motors.h` | Header contrôle moteurs (déjà existant) |
| `motors.cpp` | Implémentation contrôle moteurs (déjà existant) |
| `ALGORITHM.md` | Documentation détaillée de l'algorithme |
| `EXAMPLES.cpp` | Exemples d'utilisation |
| `README.md` | Ce fichier |

## ⚙️ Composants principaux

### 1. **Structures de données**

```cpp
// Commande de trajectoire (sortie de l'algorithme)
typedef struct {
  int throttle;   // Vitesse avant/arrière (-255 à +255)
  int steering;   // Direction (-255 à +255)
} TrajectoryCommand;
```

### 2. **Fonction naïve: `naive_trajectory()`**

Implémentation basique du concept:

```cpp
TrajectoryCommand naive_trajectory(float ranges[], float max_range, int forward_speed)
```

**Paramètres:**
- `ranges[]`: Tableau des distances du lidar (360 points)
- `max_range`: Distance maximale en cm (par défaut 300)
- `forward_speed`: Vitesse avant souhaitée (0-255)

**Retour:**
- Structure `TrajectoryCommand` avec throttle et steering

**Fonctionnement:**
1. Divise le lidar en deux zones (gauche/droite)
2. Compte les points valides dans chaque zone
3. Calcule le déséquilibre (balance)
4. Convertit en commandes moteur

### 3. **Fonction intelligente: `smart_trajectory()`**

Version améliorée avec détection d'obstacles proches:

```cpp
TrajectoryCommand smart_trajectory(float ranges[], float max_range, 
                                   int forward_speed, float danger_threshold)
```

**Améliorations:**
- Détecte les obstacles frontaux
- Recule automatiquement si trop proche
- Sinon utilise l'algorithme naïf

### 4. **Tâche FreeRTOS: `autonomousTrajectoryTask()`**

Boucle infinie qui:
- S'exécute à 20 Hz (tous les 50ms)
- Récupère les données du lidar
- Appelle `smart_trajectory()`
- Envoie les commandes aux moteurs

## 🚀 Utilisation

### Activation simple

```cpp
// Dans le setup() ou loop()
autonomous_mode = true;  // Activer l'autonomie
autonomous_throttle = 150;  // Vitesse (0-255)

// Plus tard
autonomous_mode = false;  // Arrêter
motors_stop();
```

### Via la console Serial

```
'a' - Autonomie ON
's' - Stop (OFF)
'+' - Augmenter vitesse
'-' - Diminuer vitesse
```

## 📊 Monitoring/Debugging

Chaque seconde, la tâche affiche:

```
[TRAJ] throttle=150 steering=32 | left=45 right=38 total=83
```

- **throttle**: Vitesse avant actuelle (-255 à +255)
- **steering**: Angle braquage (-128 à +128)
- **left**: Points obstacles à gauche
- **right**: Points obstacles à droite
- **total**: Points valides détectés

## 🔧 Paramètres configurables

Dans `autonomousTrajectoryTask()`:

```cpp
const float MAX_RANGE = 300.0f;        // Distance max LIDAR (cm)
const float DANGER_THRESHOLD = 30.0f;  // Distance de sécurité (cm)
const int BASE_THROTTLE = 150;         // Vitesse avant (0-255)
```

## 📈 Diagramme d'exécution

```
setup()
├─ motors_init()              ← Initialiser moteurs
├─ lidarTask (Tâche 4)        ← Lire LIDAR
├─ imuTask (Tâche 3)          ← Lire IMU
├─ autonomousTrajectoryTask (Tâche 2) ← ALGORITHME ← ICI!
└─ publishTask (Tâche 2)      ← Publier sur ROS2

autonomousTrajectoryTask (20 Hz)
├─ if autonomous_mode == true
├─ Récupérer lidar_buffers
├─ Appeler smart_trajectory()
├─ Appeler motors_drive()
└─ Attendre 50ms
```

## ✅ Avantages de l'implémentation

| Avantage | Description |
|----------|-------------|
| **Temps réel** | Pas d'overhead ROS, traitement direct |
| **Parallèle** | Tâche FreeRTOS indépendante |
| **Simple** | ~100 lignes de code |
| **Robuste** | Gère les données manquantes du lidar |
| **Configurable** | Plusieurs paramètres ajustables |
| **Debuggable** | Logs détaillées chaque seconde |

## ⚠️ Limitations

| Limitation | Impact |
|-----------|--------|
| Pas de mémoire | Chaque décision est indépendante |
| Pas de planification | Pas de chemin optimisé |
| Sensible au bruit | Faux points du lidar → mauvaise direction |
| Peut osciller | Dans espaces symétriques |
| Pas anti-deadlock | Peut se coincer en impasse |

## 🔮 Améliorations futures possibles

1. **Filtrage temporel**: Moyenne mobile des commandes
2. **Pondération par distance**: Donner plus de poids aux points proches
3. **Détection d'impasse**: Si pas de points → spirale de sortie
4. **Mémoire spatiale**: Se souvenir des zones explorées
5. **Mode "wall-following"**: Suivre les murs
6. **Machine Learning**: Apprendre les meilleurs paramètres

## 📝 Exemple de code basique

```cpp
// Dans loop() ou une fonction
void main_autonomous_example() {
  // Activer
  autonomous_mode = true;
  
  // Observer la navigation pendant 30 secondes
  // Les logs Serial montrent:
  // [TRAJ] throttle=150 steering=32 | left=45 right=38 total=83
  
  delay(30000);
  
  // Arrêter
  autonomous_mode = false;
  motors_stop();
}
```

## 🔗 Dépendances

- `Arduino.h` - Framework Arduino
- `motors.h` - Contrôle moteurs
- `FreeRTOS` - Tâches parallèles
- Données LIDAR via `lidar_buffers`

## 📞 Troubleshooting

| Problème | Solution |
|----------|----------|
| Robot n'avance pas | Vérifier `autonomous_mode = true` |
| Tourne en boucle | Augmenter `MAX_RANGE` ou diminuer vitesse |
| Saccades | Réduire `BASE_THROTTLE` |
| Ne voit pas obstacles | Vérifier LIDAR dans Serial |
| Logs manquants | Vérifier Serial à 115200 baud |

## 📊 Comparaison: Naïf vs Smart

| Aspect | Naïf | Smart |
|--------|------|-------|
| Complexité | Faible | Faible+ |
| Détection obstacles loin | Oui | Oui |
| Détection obstacles proches | Oui | Oui + Recul automatique |
| Temps calcul | ~1ms | ~2ms |
| Recommandé pour | Espaces ouverts | Partout |

## 🎯 Cas d'usage

✅ **Bon pour:**
- Navigation d'exploration simple
- Évitement d'obstacles basique
- Prototype/prototype rapide
- Apprentissage (comprendre comment ça marche)

❌ **Pas idéal pour:**
- Navigation précise/optimisée
- Espaces très encombrés
- Trajectoires définiées à l'avance
- Production critique

## 📚 Pour en savoir plus

- Voir `ALGORITHM.md` pour les détails techniques
- Voir `EXAMPLES.cpp` pour des exemples de code
- Consulter les commentaires dans `main.cpp`

---

**Créé pour le projet technique ING4**
**Date: Février 2026**
