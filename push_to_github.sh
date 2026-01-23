#!/bin/bash

###############################################################################
# SCRIPT DE DÉPLOIEMENT ET PUSH GITHUB - ECE ROS2 + microROS Project
# 
# Usage: ./push_to_github.sh <username/repo>
# Example: ./push_to_github.sh matheo/ece-ros2-lidar
#
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║ $1${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}→ $1${NC}"
}

# Check if repo argument is provided
if [ -z "$1" ]; then
    print_error "Usage: ./push_to_github.sh <username/repo>"
    print_info "Example: ./push_to_github.sh matheo/ece-ros2-lidar"
    exit 1
fi

REPO_URL="https://github.com/$1.git"
BRANCH="main"

print_header "ECE ROS2 + microROS GitHub Push"

# Check git installation
print_info "Vérification de Git..."
if ! command -v git &> /dev/null; then
    print_error "Git n'est pas installé. Installez Git d'abord."
    exit 1
fi
print_success "Git trouvé"

# Check current directory
if [ ! -f "README.md" ]; then
    print_error "README.md non trouvé. Assurez-vous d'être dans le répertoire racine du projet."
    exit 1
fi

print_success "Répertoire correct"

# Show current status
print_header "État Actuel du Repo"
git status

# Add all files
print_header "Ajout des Fichiers"
print_info "Ajout de tous les fichiers..."
git add .
print_success "Fichiers ajoutés"

# Show what will be committed
echo ""
print_info "Fichiers à committer:"
git diff --cached --name-only | head -20
if [ $(git diff --cached --name-only | wc -l) -gt 20 ]; then
    echo "... et $(expr $(git diff --cached --name-only | wc -l) - 20) autres fichiers"
fi

# Commit
print_header "Création du Commit"
COMMIT_MESSAGE="Initial commit: ECE ROS2 + microROS LIDAR 360° Project

Includes:
- ROS2 Workspace (Humble)
- microROS ESP32 Firmware (PlatformIO)
- LIDAR 360° Documentation
- RViz Configuration
- Complete Build & Deploy Scripts

Project: LIDAR 360° Visualization System
Author: Matheo
Date: $(date +'%Y-%m-%d')"

print_info "Message du commit:"
echo "$COMMIT_MESSAGE"

git commit -m "$COMMIT_MESSAGE"
print_success "Commit créé"

# Rename branch to main if master
if git rev-parse --abbrev-ref HEAD | grep -q "master"; then
    print_info "Renommage de la branche master -> main..."
    git branch -m main
    print_success "Branche renommée"
fi

# Add remote
print_header "Configuration du Repository Distant"
print_info "Vérification du remote 'origin'..."

if git remote | grep -q "origin"; then
    print_info "Remote 'origin' existe. Mise à jour..."
    git remote set-url origin "$REPO_URL"
else
    print_info "Ajout du remote 'origin'..."
    git remote add origin "$REPO_URL"
fi

print_success "Remote configuré: $REPO_URL"

# Show remote
echo ""
print_info "Remote actuel:"
git remote -v

# Push to GitHub
print_header "Push vers GitHub"
echo ""
print_info "Cette étape nécessite une authentification GitHub."
print_info "Options:"
echo "  1. GitHub CLI (gh auth login)"
echo "  2. Token d'accès personnel (PAT)"
echo "  3. SSH key"
echo ""
print_info "Tentative de push..."

if git push -u origin main 2>&1 | grep -q "remote: Repository not found"; then
    print_error "Repository non trouvé!"
    print_info "Assurez-vous d'avoir créé le repo sur GitHub: https://github.com/$1"
    exit 1
elif git push -u origin main; then
    print_success "Push réussi!"
else
    print_error "Erreur lors du push. Vérifiez votre authentification."
    echo ""
    print_info "Solutions:"
    echo "  1. Utilisez GitHub CLI: gh auth login"
    echo "  2. Générez un PAT et utilisez-le comme mot de passe"
    echo "  3. Configurez une clé SSH: ssh-keygen -t ed25519"
    exit 1
fi

# Verify push
print_header "Vérification"
print_info "Repo distant: $REPO_URL"
print_success "Push complété avec succès!"

echo ""
print_header "Prochaines Étapes"
echo "1. Vérifiez le repo sur GitHub:"
echo "   https://github.com/$1"
echo ""
echo "2. Clonez le repo pour tester:"
echo "   git clone $REPO_URL"
echo ""
echo "3. Ajoutez une description du projet sur GitHub"
echo ""
echo "4. (Optionnel) Configurez les protections de branche:"
echo "   - Require pull request reviews"
echo "   - Require status checks to pass"
echo ""

print_success "Tout est fait! 🎉"
