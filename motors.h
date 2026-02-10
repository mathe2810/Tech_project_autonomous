// ============================================================
// Contrôle moteurs - Version simplifiée
// ============================================================

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// Initialisation des moteurs
void motors_init();

// Contrôle moteur gauche: speed = -255 à +255
void motor_left(int speed);

// Contrôle moteur droit: speed = -255 à +255
void motor_right(int speed);

// Arrêt complet
void motors_stop();

// Contrôle type voiture RC: throttle (-255 à +255), steering (-255 à +255)
void motors_drive(int throttle, int steering);

#endif // MOTORS_H
