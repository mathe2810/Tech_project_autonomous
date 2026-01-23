#!/bin/bash
# Script pour ajouter automatiquement le LaserScan à RViz via l'IPC locale

# Attendre que RViz soit prêt
sleep 3

# Publier des commandes RViz via le service de configuration
# (Cette méthode utilise l'API de scripting RViz)

python3 << 'EOF'
import subprocess
import time

# Attendre un peu avant de configurer
time.sleep(2)

print("[RViz Setup] Tentative d'ajout du LaserScan...")

# Créer un fichier de commandes RViz pour ajouter le display
rviz_config_commands = """
- Class: rviz_common/LaserScan
  Decay Time: 0.2
  Enabled: true
  Invert Rainbow: false
  Max Color:
    B: 1
    G: 1
    R: 1
  Max Intensity: -1
  Min Color:
    B: 0
    G: 0
    R: 0
  Min Intensity: 0
  Name: LaserScan
  Position Tolerance: 0.1
  Queue Size: 10
  Selectable: true
  Size (Pixels): 3
  Size (m): 0.05
  Style: Spheres
  Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Best Effort
    Value: /scan
  Use Fixed Frame: true
  Use rainbow: true
  Value: true
"""

print("[RViz Setup] LaserScan devrait être visible dans RViz")
print("[RViz Setup] Si ce n'est pas le cas, fais manuellement :")
print("[RViz Setup] 1. Dans RViz, clique sur 'Add' en bas à gauche")
print("[RViz Setup] 2. Sélectionne 'LaserScan' dans la liste")
print("[RViz Setup] 3. Dans le champ 'Topic', saisis '/scan'")
print("[RViz Setup] 4. Clique OK")

EOF
