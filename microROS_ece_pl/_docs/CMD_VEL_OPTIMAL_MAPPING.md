# CMD_VEL optimal pour mapping stable (anti-drift)

Ce document résume les valeurs `cmd_vel` les plus stables pour éviter le drift en rotation et en déplacement.
Objectif: mapping fluide et fiable, même si le robot est lent.

## 1) Valeurs recommandées (profil SAFE)

## Utilisation pratique
- Déplacement linéaire recommandé: `linear.x = 0.10` à `0.14` m/s
- Rotation pure recommandée: `angular.z = 1.30` rad/s
- Rotation max tolérée (à éviter en continu): `angular.z = 1.50` rad/s
- Éviter les changements brusques de consigne (accélération instantanée)

## Profil SAFE conseillé
- `linear.x = 0.12` m/s
- `angular.z = 1.30` rad/s
- Durée rotation 360° à 1.30 rad/s: ~`4.83 s`

## Pourquoi ce profil
- Plus stable que les rotations lentes qui ne vainquent pas la friction
- Moins agressif que 1.5 rad/s qui peut sur-réagir
- Bon compromis précision/réactivité pour SLAM + RF2O

---

## 2) Résultats de tests rotation (historique)

## Test `test_rotation_drift.py` (ancien réglage 1.0)
- Consigne: rotation à `1.0` rad/s
- Résultat mesuré: `112.6°` (au lieu de 360°)
- Drift linéaire: `7.23 cm`
- Conclusion: **rotation trop faible / friction / sous-rotation**

## Test `test_rotation_drift.py` (réglage 1.5)
- Consigne: rotation à `1.5` rad/s
- Résultat mesuré: `693.7°`
- Drift linéaire: `13.75 cm`
- Conclusion: **robot tourne, mais sur-réaction / sur-estimation en rotation**

## Retour terrain utilisateur
- `1.3` rad/s en rotation pure: **passe mieux**
- Recommandation: garder `1.3` pour mapping (plus stable)

---

## 3) Résumé tests `diagnose_rotation.py`

Le script compare rotation commandée vs rotation mesurée (`/odom`).
Interprétation:
- Ratio < 40%: sous-réaction forte (friction/slip, vitesse trop basse)
- Ratio 70-90%: correct avec compensation SLAM
- Ratio > 120%: sur-réaction/sur-estimation (trop agressif)

Avec vos essais:
- Zone basse (~1.0): sous-réaction
- Zone haute (~1.5): sur-réaction
- **Zone cible pratique: ~1.3 rad/s**

---

## 4) Bonnes pratiques cmd_vel anti-drift

- Privilégier des commandes constantes (pas de pulses rapides)
- En rotation, éviter `> 1.4` rad/s sur longues séquences
- Faire des arcs (linéaire + angulaire modéré) plutôt que pivot agressif prolongé
- Ajouter une petite pause (0.3-0.5 s) entre phases: ligne droite -> rotation -> ligne droite
- Garder la surface la plus adhérente possible (moins de glissement)

---

## 5) Presets conseillés

## Preset Mapping Stable (recommandé)
- `linear.x = 0.12`
- `angular.z = 1.30`

## Preset Rotation Fine (zones serrées)
- `linear.x = 0.08`
- `angular.z = 1.10`

## Preset Rapide (à utiliser peu)
- `linear.x = 0.15`
- `angular.z = 1.50` (courte durée)

---

## 6) Commandes de vérification (pendant test)

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic hz /tf
```

Cibles pratiques:
- `/odom`: stable, sans trous
- `/map`: continu (pas besoin d'être très haut)
- Pas de dérive visible après boucle fermée

---

## 7) Décision finale actuelle

Pour votre robot et vos moteurs:
- **Valeur rotation recommandée: `1.3 rad/s`**
- **Valeur déplacement recommandée: `0.12 m/s`**

Si drift réapparaît:
1. baisser rotation à `1.2`
2. réduire vitesse linéaire à `0.10`
3. éviter rotations longues en place
