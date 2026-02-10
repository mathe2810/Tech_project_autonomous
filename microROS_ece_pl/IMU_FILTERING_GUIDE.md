# IMU Filtering Strategy - MPU6050

## Spécifications MPU6050

### Fréquence d'échantillonnage
- **Horloge interne**: ~8 MHz
- **Taux d'échantillonnage par défaut**: 1 kHz (configurable via registre 0x19)
- **Diviseur (SMPRT_DIV)**: `actual_rate = 1000 / (1 + SMPRT_DIV)`
  - SMPRT_DIV=9 → 100 Hz ✅ (recommandé pour robots)
  - SMPRT_DIV=4 → 200 Hz (plus rapide)
  - SMPRT_DIV=19 → 50 Hz (plus lent)

### Spécifications I2C
- Standard I2C: jusqu'à 400 kHz
- Vitesse de transfert typique: 100-400 kHz
- **Latence**: ~10-20 ms

### Gammes de mesure (configurables)
- **Accéléromètre**: ±2g, ±4g, ±8g, ±16g
- **Gyroscope**: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s

---

## Problèmes courants de bruit

### Sources de bruit typiques
1. **Vibrations moteur** (50-100 Hz)
2. **Frequency de PWM** (5-10 kHz, rarement visible dans l'accélération)
3. **Bruit thermique** (blanc, large bande)
4. **Bruit I2C/électrique** (dépend du câblage)
5. **Alimentation**: Fréquences harmoniques de la tension d'alimentation

---

## Workflow de filtrage

### Étape 1: Analyse FFT
```bash
# Démarrer le stack normalement (agent + IMU)
./start_stack.sh

# Dans un autre terminal, capturer et analyser
python3 imu_fft_analysis.py --duration 30 --sample-rate 100
```

**Cela produit:**
- `imu_raw_data.csv` - Données brutes
- `imu_fft_analysis.png` - Graphique FFT avec pics identifiés
- Console - Fréquences parasites principales

### Étape 2: Conception du filtre FIR

Basé sur l'analyse FFT, choisir:

#### Option A: Filtre passe-bas simple (pour bruit blanc)
```bash
# Cutoff à 20 Hz (bon pour robots lents)
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100
```

#### Option B: Filtre avec notch (pour fréquence parasite spécifique)
```bash
# Cutoff à 20 Hz + notches à 50 Hz et 100 Hz (vibrations moteur typiques)
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100 --notch 50 100
```

#### Option C: Filtre moins agressif (pour préserver la dynamique)
```bash
# Cutoff à 30 Hz (laisse plus de fréquences)
python3 imu_fir_filter.py --cutoff 30 --order 15 --sample-rate 100
```

---

## Interprétation des résultats FFT

### Exemple: Résultats typiques
```
Accel x: Top peaks:
  45.2Hz: 0.234    ← Vibration moteur
  92.1Hz: 0.089    ← Harmonique 2x
  Bruit blanc: 0.001-0.010

Gyro z: Top peaks:
  45.2Hz: 0.012    ← Même fréquence, amplitude plus faible
```

**Interprétation:**
- Pic dominant à ~45 Hz → filtre notch à 45 Hz
- Harmonic à 92 Hz → peut aussi ajouter notch à 92 Hz
- Bruit blanc bas → filtre passe-bas suffisant

---

## Paramètres du filtre FIR

### Cutoff Frequency
- **Trop bas** (5 Hz): Filtre excellent mais très lent (lag)
- **Recommandé** (15-25 Hz): Bon équilibre vitesse/précision
- **Trop haut** (50+ Hz): Ne filtre pas assez

### FIR Order (Ordre du filtre)
- **Ordre 11**: Rapide, léger filtrage
- **Ordre 21** (recommandé): Bon équilibre
- **Ordre 31+**: Meilleur filtrage, plus de latence

**Règle**: `Latency ≈ (Order / 2) / sample_rate`
- Ordre 21 @ 100 Hz → latence ≈ 100 ms
- Ordre 15 @ 100 Hz → latence ≈ 75 ms

### Notch Filters
- Utilisez si FFT montre un pic très dominant
- Exemple: 50 Hz (fréquence du secteur), 60 Hz (États-Unis)
- Max 2-3 notches (au-delà, instabilité)

---

## Comparaison: EMA vs FIR vs Kalman

| Filtre | Latence | Bruit blanc | Bruits parasites | Complexité |
|--------|---------|-------------|------------------|-----------|
| **EMA** | Faible | Moyen | Mauvais | Très faible |
| **FIR** (ce script) | Moyen | Très bon | Très bon | Moyen |
| **Kalman** | Faible | Excellent | Moyen | Haut |

**Pour ce projet**: FIR + Kalman Filter Fusion = optimal

---

## Configuration recommandée pour votre robot

### Configuration par défaut (robot lent, bruyant)
```bash
# Dans start_stack.sh, remplacer:
python3 imu_kalman_filter.py

# Par:
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100
```

### Configuration pour robot rapide (moins de lag)
```bash
python3 imu_fir_filter.py --cutoff 25 --order 15 --sample-rate 100
```

### Configuration après analyse FFT
```bash
# Si on voit pic dominant à 45 Hz:
python3 imu_fir_filter.py --cutoff 20 --order 21 --sample-rate 100 --notch 45
```

---

## Débogage

### Vérifier la latence
```bash
# Comparer timestamps /imu/data vs /imu/data_filtered
ros2 topic echo /imu/data | grep "sec"
ros2 topic echo /imu/data_filtered | grep "sec"
```

### Vérifier le filtrage
```bash
# Capturer et comparer brut vs filtré
python3 -c "
import csv
import numpy as np

with open('imu_raw_data.csv') as f:
    reader = csv.DictReader(f)
    raw = [float(row['accel_x']) for row in reader]

print(f'Raw std: {np.std(raw):.4f}')
print(f'Raw mean: {np.mean(raw):.4f}')
print(f'Raw range: {np.max(raw)-np.min(raw):.4f}')
"
```

### Réglage fin
Si le filtrage est trop agressif:
- Diminuer cutoff de 2-3 Hz
- Diminuer FIR order de 5-10

Si le bruit persiste:
- Augmenter cutoff de 5 Hz
- Augmenter FIR order de 10
- Ajouter notches aux fréquences parasites

---

## Prochaines étapes

1. ✅ Lancer `imu_fft_analysis.py` (capturer 30s)
2. ✅ Analyser `imu_fft_analysis.png` (identifier pics)
3. ✅ Configurer `imu_fir_filter.py` avec paramètres adaptés
4. ✅ Valider avec le Kalman Filter Fusion
5. ✅ Tester la navigation avec Nav2
