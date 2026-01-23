# Guide GitHub - Upload et Configuration

Ce guide vous explique comment uploader ce projet sur GitHub et comment le configurer pour le partage.

## 📋 Table des Matières

1. [Prérequis](#prérequis)
2. [Créer un Repository sur GitHub](#créer-un-repository-sur-github)
3. [Upload du Projet](#upload-du-projet)
4. [Vérification](#vérification)
5. [Configuration Supplémentaire](#configuration-supplémentaire)
6. [Troubleshooting](#troubleshooting)

---

## 🔧 Prérequis

### 1. Compte GitHub

Si vous n'avez pas de compte GitHub:
- Allez sur https://github.com/join
- Créez un compte (gratuit)

### 2. Git installé

```bash
sudo apt install git
git --version
```

### 3. Authentification GitHub

Vous avez 3 options:

#### Option A: GitHub CLI (Recommandé)
```bash
# Installation
sudo apt install gh

# Authentification
gh auth login
# Suivez les instructions (choisissez HTTPS)

# Vérification
gh auth status
```

#### Option B: Token d'Accès Personnel (PAT)
1. Allez sur https://github.com/settings/tokens
2. Cliquez "Generate new token" → "Generate new token (classic)"
3. Sélectionnez les scopes:
   - ☑️ repo (accès complet aux repos)
   - ☑️ workflow (pour les actions)
4. Générez et copiez le token
5. Utilisez le token comme mot de passe lors du push

#### Option C: SSH Key
```bash
# Générer une clé
ssh-keygen -t ed25519 -C "votre_email@example.com"

# Afficher la clé
cat ~/.ssh/id_ed25519.pub

# Ajoutez la clé à GitHub:
# Allez sur https://github.com/settings/keys
# Cliquez "New SSH key" et collez le contenu
```

---

## 🆕 Créer un Repository sur GitHub

### Via GitHub Web Interface

1. Allez sur https://github.com/new
2. Remplissez les champs:

```
Repository name:        ece-ros2-lidar
(ou votre nom préféré)

Description:            ECE ROS2 + microROS LIDAR 360° Project
                        Complete workspace for LIDAR visualization 
                        using ROS2, microROS, and ESP32

Visibility:             ☑️ Public (ou Private si vous préférez)

Initialize:             ☐ NO - nous avons déjà git initialisé
```

3. Cliquez **"Create repository"**

4. Vous verrez une page avec votre URL:
   ```
   https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git
   ```

### Via GitHub CLI (Alternative)

```bash
gh repo create ece-ros2-lidar \
  --public \
  --source=. \
  --remote=origin \
  --push
```

---

## 📤 Upload du Projet

### Méthode Automatique (Script)

```bash
cd /home/matheo/ros_test

# Rendre le script exécutable
chmod +x push_to_github.sh

# Lancer le script
./push_to_github.sh VOTRE_USERNAME/ece-ros2-lidar
```

Le script va:
- ✅ Ajouter tous les fichiers
- ✅ Créer un commit initial
- ✅ Configurer le remote
- ✅ Pusher vers GitHub

### Méthode Manuelle (Commandes)

```bash
cd /home/matheo/ros_test

# 1. Ajouter les fichiers
git add .

# 2. Créer un commit
git commit -m "Initial commit: ECE ROS2 + microROS LIDAR 360° Project"

# 3. Renommer la branche (optionnel, pour utiliser 'main' au lieu de 'master')
git branch -m main

# 4. Ajouter le remote
git remote add origin https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git

# 5. Pusher
git push -u origin main
```

---

## ✅ Vérification

### Sur GitHub

1. Allez sur https://github.com/VOTRE_USERNAME/ece-ros2-lidar
2. Vous devriez voir:
   - ✅ Tous vos fichiers et dossiers
   - ✅ Le README.md affiché
   - ✅ La structure du projet

### En Local

```bash
cd /home/matheo/ros_test

# Vérifier le remote
git remote -v
# Devrait afficher:
# origin  https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git (fetch)
# origin  https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git (push)

# Vérifier le log
git log
# Devrait afficher votre commit initial
```

---

## ⚙️ Configuration Supplémentaire

### 1. Ajouter une Description et un Logo

Sur GitHub:
1. Allez sur les settings du repo
2. Remplissez "Description" et "Website URL" (optionnel)
3. Ajoutez des "Topics" pour faciliter la découverte:
   - `ros2`
   - `lidar`
   - `microros`
   - `esp32`
   - `robotics`

### 2. Configurer les Pages GitHub (Documentation)

1. Allez dans **Settings** → **Pages**
2. Sous "Build and deployment", sélectionnez:
   - **Source**: Deploy from a branch
   - **Branch**: main, /root

Cela générera un site de documentation à partir de votre README.

### 3. Ajouter des Badges

Optionnel: Ajoutez des badges à votre README:

```markdown
# ECE ROS2 & microROS Lidar 360° Project

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![microROS](https://img.shields.io/badge/microROS-latest-green)](https://micro.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-blue)](https://www.python.org/)
```

### 4. Ajouter une Licence

Créez un fichier `LICENSE` à la racine:

```bash
cd /home/matheo/ros_test

# Créer une licence MIT
cat > LICENSE << 'EOF'
MIT License

Copyright (c) 2026 Matheo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
EOF

# Ajouter et pusher
git add LICENSE
git commit -m "docs: Add MIT License"
git push origin main
```

### 5. Créer un .gitignore Personnalisé

(Déjà fait, mais vous pouvez l'améliorer)

```bash
# Voir le contenu actuel
cat /home/matheo/ros_test/.gitignore

# Pusher les mises à jour si nécessaire
git add .gitignore
git commit -m "refactor: Improve .gitignore"
git push origin main
```

---

## 🔄 Workflows GitHub Actions (Optionnel)

Vous pouvez ajouter des tests automatiques. Créez:

```bash
mkdir -p .github/workflows
cat > .github/workflows/ci.yml << 'EOF'
name: CI/CD

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    
    steps:
      - uses: actions/checkout@v3
      
      - name: Install ROS2
        run: |
          sudo apt-get update
          sudo apt-get install -y ros-humble-desktop
      
      - name: Build ROS2 Workspace
        run: |
          source /opt/ros/humble/setup.bash
          cd ros2_ece_ws
          colcon build
EOF

git add .github/workflows/ci.yml
git commit -m "ci: Add GitHub Actions workflow"
git push origin main
```

---

## 🚀 Cloner le Repo (Test)

Pour tester que tout fonctionne:

```bash
cd /tmp
git clone https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git
cd ece-ros2-lidar
ls -la
cat README.md
```

---

## 📚 Commandes Git Usuelles

Après le push initial, voici les commandes courantes:

```bash
# Ajouter des changements
git add src/cpp_pubsub/  # Ajouter un dossier spécifique
git add .               # Ajouter tous les changements

# Voir les changements avant de committer
git status              # Voir ce qui a changé
git diff                # Voir les différences exactes

# Committer
git commit -m "Description du changement"

# Pusher vers GitHub
git push origin main    # Pusher la branche main
git push origin --all   # Pusher toutes les branches

# Récupérer les changements (si vous travaillez en équipe)
git pull origin main
```

---

## 🐛 Troubleshooting

### Erreur: "fatal: not a git repository"

```bash
cd /home/matheo/ros_test
git init
```

### Erreur: "Repository not found"

Vérifiez:
1. L'URL du repository est correcte
2. Le repo existe sur GitHub
3. Vous êtes authentifié avec Git

```bash
# Vérifier l'authentification
git remote -v

# Vérifier la connexion
ssh -T git@github.com  # Pour SSH
gh auth status         # Pour GitHub CLI
```

### Erreur: "Permission denied"

Si vous utilisez SSH:
```bash
# Vérifier la clé SSH
ssh-keygen -t ed25519 -C "votre_email@example.com"
ssh-add ~/.ssh/id_ed25519

# Ajouter la clé à GitHub:
# https://github.com/settings/keys
cat ~/.ssh/id_ed25519.pub
```

Si vous utilisez HTTPS, utilisez un PAT au lieu de votre mot de passe.

### La branche est "master" au lieu de "main"

```bash
git branch -m main
git push -u origin main

# Optionnel: Définir la branche par défaut sur GitHub
# Settings → Branches → Default branch → main
```

### J'ai oublié d'ajouter le .gitignore

Les fichiers ignorés ne seront pas supprimés. Pour les retirer du tracking:

```bash
git rm --cached build/ install/ log/ -r
git commit -m "refactor: Remove build artifacts from tracking"
git push origin main
```

---

## 📞 Support

Si vous avez des problèmes:

1. **Vérifiez le status:**
   ```bash
   git status
   git log -1
   git remote -v
   ```

2. **Consultez la documentation:**
   - GitHub Docs: https://docs.github.com
   - Git Docs: https://git-scm.com/doc

3. **Posez une question:**
   - GitHub Issues: https://github.com/VOTRE_USERNAME/ece-ros2-lidar/issues
   - Stack Overflow: https://stackoverflow.com/questions/tagged/git

---

## ✨ Résumé

```bash
# 5 étapes pour uploader sur GitHub:

1. Créez un repo sur https://github.com/new
2. Notez l'URL: https://github.com/VOTRE_USERNAME/ece-ros2-lidar.git
3. Allez dans le dossier du projet
4. Exécutez: ./push_to_github.sh VOTRE_USERNAME/ece-ros2-lidar
5. Vérifiez sur GitHub! ✅
```

---

**Date:** Janvier 2026  
**Dernière modification:** Janvier 2026
