# 🚀 Quick Start - Algorithme de Trajectoire Naïve

## TL;DR (Trop Long; Pas Lu)

Vous avez maintenant un **algorithme de navigation autonome** qui fait fonctionner votre robot avec le LIDAR!

## ⚡ 5 minutes pour démarrer

### 1. Vérifier la structure
```
naive/
├── main.cpp          ← Code principal (MODIFIÉ)
├── motors.h          ← Contrôle moteurs
├── motors.cpp        ← Implémentation moteurs
└── Documentation:
    ├── README.md
    ├── ALGORITHM.md
    ├── EXAMPLES.cpp
    └── ... (autres fichiers d'aide)
```

### 2. Activer le mode autonome

**Option A: Simple** (dans votre `setup()` ou une fonction)
```cpp
void start_autonomous() {
  autonomous_mode = true;
  // Le robot navigue tout seul!
}
```

**Option B: Avec contrôle de vitesse**
```cpp
void start_with_speed(int speed) {
  autonomous_throttle = speed;    // 0-255
  autonomous_mode = true;
}
```

### 3. Arrêter
```cpp
void stop_autonomous() {
  autonomous_mode = false;
  motors_stop();
}
```

## 📊 Comment ça marche en 30 secondes

```
┌─────────────────┐
│   LIDAR (360°)  │
└────────┬────────┘
         │
  Compte les obstacles:
   Gauche (45-315°)  vs  Droite (0-45°, 315-360°)
         │
   Plus à gauche? → Tourne droite
   Plus à droite? → Tourne gauche
   Équilibré?      → Avance droit
         │
  ┌─────┴─────┐
  │  Moteurs  │
  └───────────┘
      ↓
   🤖 ROBOT NAVIGUE!
```

## 🎮 Contrôle via Serial (optionnel)

Ajoutez cette fonction dans votre `loop()`:

```cpp
void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();
    if(cmd == 'a') autonomous_mode = true;
    if(cmd == 's') { autonomous_mode = false; motors_stop(); }
    if(cmd == '+') autonomous_throttle += 10;
    if(cmd == '-') autonomous_throttle -= 10;
  }
}
```

Puis dans Serial Monitor:
- `a` = Autonomie ON
- `s` = Stop
- `+` = Plus rapide
- `-` = Plus lent

## 📈 Observer l'algorithme

Chaque seconde, vous verrez:
```
[TRAJ] throttle=150 steering=32 | left=45 right=38 total=83
       └──────┬──────┘ └─────┬─────┘ └──────────┬──────────┘
          Moteurs    Direction   Points lidar détectés
```

Cela montre:
- **throttle**: Vitesse forward (-255 à +255)
- **steering**: Direction (-128 à +128) - positif = droite
- **left/right/total**: Nombre de points dans chaque zone

## 🔧 Paramètres à ajuster

Dans `main.cpp`, fonction `autonomousTrajectoryTask()`:

```cpp
const float MAX_RANGE = 300.0f;        // ← Distance max LIDAR
const float DANGER_THRESHOLD = 30.0f;  // ← Distance min avant recul
const int BASE_THROTTLE = 150;         // ← Vitesse de base
```

### Quand les ajuster:

| Problème | Solution |
|----------|----------|
| Robot oscille | Diminuer `BASE_THROTTLE` |
| Ne voit pas obstacles | Augmenter `MAX_RANGE` |
| Trop agressif | Réduire `MAX_RANGE` |
| Recule trop facilement | Augmenter `DANGER_THRESHOLD` |
| Trop lent | Augmenter `BASE_THROTTLE` |

## 🧪 Tester rapidement

### Test 1: Espace ouvert
```cpp
autonomous_throttle = 150;
autonomous_mode = true;
delay(5000);
autonomous_mode = false;
// → Robot devrait avancer tout droit
```

### Test 2: Avec obstacle
```cpp
// Placer une main/obstacle devant le LIDAR
autonomous_mode = true;
// → Robot devrait reculer et tourner
autonomous_mode = false;
```

### Test 3: Navigation courte
```cpp
autonomous_mode = true;
delay(30000);  // 30 secondes
autonomous_mode = false;
// → Observer comment il navigue/évite les obstacles
```

## 🎯 Ce que l'algorithme fait bien

✅ Évite les obstacles automatiquement
✅ Navigue vers les espaces libres
✅ Fonctionne en temps réel (20 Hz)
✅ Pas de configuration complexe
✅ Robuste aux données manquantes du LIDAR

## ⚠️ Ce que l'algorithme ne fait pas

❌ Optimise la trajectoire (chemin le plus court)
❌ Se souvient des endroits visités
❌ Détecte les objets spécifiques
❌ Crée une carte

## 📚 Pour plus d'infos

- **ALGORITHM.md** - Explication technique complète
- **EXAMPLES.cpp** - 7 exemples de code
- **TEST_TRAJECTORY.cpp** - Tests et validation
- **README.md** - Vue d'ensemble détaillée

## ✅ Checklist avant de lancer

- [ ] Moteurs initialisés (`motors_init()` dans setup)
- [ ] LIDAR connecté et publiant des données
- [ ] Tâche autonome créée (`autonomousTrajectoryTask`)
- [ ] Serial Monitor à 115200 baud
- [ ] Espace de test libre d'obstacles

## 🚀 Démarrer en 3 lignes

```cpp
// Dans setup():
motors_init();
autonomous_mode = true;

// ✓ C'est tout! Le robot navigue maintenant.
```

## 🆘 Troubleshooting rapide

| Symptôme | Solution |
|----------|----------|
| Rien ne se passe | Vérifier `autonomous_mode = true` |
| Robot tourne en place | LIDAR détecte un obstacle? |
| Erreurs de compilation | Vérifier `#include "motors.h"` |
| Serial show rien | Vérifier baud rate 115200 |
| Moteurs ne tournent pas | Vérifier pins moteurs dans config.h |

## 💡 Astuces

- Commencez avec `BASE_THROTTLE = 100` (lent et safe)
- Augmentez graduellement jusqu'à 200 max
- Testez d'abord en espace ouvert (parking, garage)
- Observez les logs Serial pour déboguer
- Les obstacles symétriques peuvent faire osciller

## 🎓 Ce que vous avez appris

✓ Traitement temps réel des données LIDAR
✓ Algorithmes de navigation autonome
✓ Programmation FreeRTOS multi-tâches
✓ Contrôle moteur en boucle fermée (implicite)
✓ Debugging robotique

---

**C'est prêt! Amusez-vous bien avec votre robot autonome! 🤖**

Questions? Consultez:
1. **README.md** - Vue d'ensemble
2. **ALGORITHM.md** - Comment ça marche
3. **EXAMPLES.cpp** - Code à réutiliser
4. **Serial Monitor** - Logs de debug
