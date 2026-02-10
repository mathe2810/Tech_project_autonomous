#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Configuration globale
// ============================================================

// Activation/Désactivation des systèmes
#define ENABLE_MOTORS 0
#define ENABLE_LIDAR 1
#define ENABLE_IMU 1

// ============================================================
// Configuration moteurs
// ============================================================

#if ENABLE_MOTORS

// Moteur A (gauche)
#define MOTOR_LEFT_DIR1 21    // GPIO21 - AIN1
#define MOTOR_LEFT_DIR2 17    // GPIO17 - AIN2
#define MOTOR_LEFT_PWM 25     // GPIO25 - PWMA

// Moteur B (droit)
#define MOTOR_RIGHT_DIR1 22   // GPIO22 - BIN1
#define MOTOR_RIGHT_DIR2 23   // GPIO23 - BIN2
#define MOTOR_RIGHT_PWM 26    // GPIO26 - PWMB

// PWM configuration
#define PWM_FREQ 100000       // 100 kHz (comme Waveshare)
#define PWM_RESOLUTION 8      // 8-bit (0-255)
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define MAX_SPEED 255         // 0-255

#endif // ENABLE_MOTORS

#endif // CONFIG_H
