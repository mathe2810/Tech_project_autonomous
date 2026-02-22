# SLAM Working Branch - Résumé des Modifications

## Points Clés
- Limitation globale de la rotation (angular_z ≤ 1.3 rad/s) dans tous les scripts Python (wall_centering_node.py, teleop_keyboard.py).
- Correction des cas spéciaux (obstacle frontal) pour éviter les rotations violentes.
- Saturation de angular_z uniquement au moment de la publication.
- Logging CSV unique pour les tests.
- Calibration du front LiDAR (-90°) appliquée.
- Stack SLAM lancée sans RViz pour économiser CPU, puis RViz lancé séparément.

## Procédure de Lancement
1. Lancer la stack SLAM sans RViz :
   ```bash
   ./start_slam_no_rviz.sh
   ```
2. Lancer RViz dans un autre terminal :
   ```bash
   rviz2 -d rviz_config.rviz
   ```
3. Lancer le script Python de contrôle (ex : wall_centering_node.py).

## Prochaine étape
- Tester sur le circuit réel.
- Publier tout sur la branche GitHub `slam-working`.

## Fichiers modifiés
- wall_centering_node.py
- teleop_keyboard.py
- scan_restamper_simple.py
- README.md
- CMD_VEL_OPTIMAL_MAPPING.md

## Historique des tests
- Mapping stable avec angular_z = 1.3
- Stack SLAM + RF2O validée
- CPU surveillé lors de tests simultanés

---
Pour plus de détails, voir les autres fichiers MD (CMD_VEL_OPTIMAL_MAPPING.md, MAPPING_ROTATION_FIX.md, RF2O_SETUP.md, etc.).
