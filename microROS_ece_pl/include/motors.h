#ifndef MOTORS_H
#define MOTORS_H

// ============================================================
// Contrôle moteurs - Interface
// ============================================================

#include <Arduino.h>
#include "config.h"

#if ENABLE_MOTORS

// Initialisation des pins et PWM
void motors_init();

// Contrôle moteur gauche (-255 à +255)
void motor_left(int speed);

// Contrôle moteur droit (-255 à +255)
void motor_right(int speed);

// Arrêt des deux moteurs
void motors_stop();

// Contrôle différentiel (throttle + steering)
// throttle: -255 (arrière) à +255 (avant)
// steering: -255 (gauche) à +255 (droite)
void motors_drive(int throttle, int steering);

#else

// Stubs vides si moteurs désactivés
inline void motors_init() {}
inline void motor_left(int) {}
inline void motor_right(int) {}
inline void motors_stop() {}
inline void motors_drive(int, int) {}

#endif // ENABLE_MOTORS

#endif // MOTORS_H
