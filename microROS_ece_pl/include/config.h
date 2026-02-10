#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Configuration globale
// ============================================================

// Activation/Désactivation des systèmes
#define ENABLE_MOTORS 1
#define ENABLE_LIDAR 1
#define ENABLE_IMU 1

// ============================================================
// Configuration moteurs
// ============================================================

#if ENABLE_MOTORS

// Moteur gauche
#define MOTOR_LEFT_DIR1 25    // GPIO25 - AIN1
#define MOTOR_LEFT_DIR2 26    // GPIO26 - AIN2
#define MOTOR_LEFT_PWM 27     // GPIO27 - PWMA

// Moteur droit
#define MOTOR_RIGHT_DIR1 32   // GPIO32 - BIN1
#define MOTOR_RIGHT_DIR2 33   // GPIO33 - BIN2
#define MOTOR_RIGHT_PWM 34    // GPIO34 - PWMB

// PWM configuration
#define PWM_FREQ 5000         // 5 kHz
#define PWM_RESOLUTION 8      // 8-bit (0-255)
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define MAX_SPEED 255         // 0-255

#endif // ENABLE_MOTORS

#endif // CONFIG_H
