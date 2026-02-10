// ============================================================
// Contrôle moteurs - Implémentation
// ============================================================

#include "motors.h"
#include "config.h"

#if ENABLE_MOTORS

void motors_init() {
    // Configuration des pins moteur gauche
    pinMode(MOTOR_LEFT_DIR1, OUTPUT);
    pinMode(MOTOR_LEFT_DIR2, OUTPUT);
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHANNEL_LEFT);
    
    // Configuration des pins moteur droit
    pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHANNEL_RIGHT);
    
    // Arrêt initial
    motors_stop();
    
    Serial.println("[MOTORS] Initialized");
}

void motor_left(int speed) {
    // Limite la vitesse (MIN_PWM pour éviter stall)
    const int MIN_PWM = MAX_SPEED / 5;  // 51 (20% de 255)
    
    if (speed > 0) {
        // Avant - Selon doc: AIN1=LOW, AIN2=HIGH
        digitalWrite(MOTOR_LEFT_DIR1, LOW);
        digitalWrite(MOTOR_LEFT_DIR2, HIGH);
        // Appliquer PWM minimum pour éviter stall
        int pwm = constrain(abs(speed), MIN_PWM, MAX_SPEED);
        ledcWrite(PWM_CHANNEL_LEFT, pwm);
    } else if (speed < 0) {
        // Arrière - Selon doc: AIN1=HIGH, AIN2=LOW
        digitalWrite(MOTOR_LEFT_DIR1, HIGH);
        digitalWrite(MOTOR_LEFT_DIR2, LOW);
        int pwm = constrain(abs(speed), MIN_PWM, MAX_SPEED);
        ledcWrite(PWM_CHANNEL_LEFT, pwm);
    } else {
        // Arrêt
        digitalWrite(MOTOR_LEFT_DIR1, LOW);
        digitalWrite(MOTOR_LEFT_DIR2, LOW);
        ledcWrite(PWM_CHANNEL_LEFT, 0);
    }
}

void motor_right(int speed) {
    // Limite la vitesse (MIN_PWM pour éviter stall)
    const int MIN_PWM = MAX_SPEED / 5;  // 51 (20% de 255)
    
    if (speed > 0) {
        // Avant - Selon doc: BIN1=LOW, BIN2=HIGH
        digitalWrite(MOTOR_RIGHT_DIR1, LOW);
        digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
        int pwm = constrain(abs(speed), MIN_PWM, MAX_SPEED);
        ledcWrite(PWM_CHANNEL_RIGHT, pwm);
    } else if (speed < 0) {
        // Arrière - Selon doc: BIN1=HIGH, BIN2=LOW
        digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
        digitalWrite(MOTOR_RIGHT_DIR2, LOW);
        int pwm = constrain(abs(speed), MIN_PWM, MAX_SPEED);
        ledcWrite(PWM_CHANNEL_RIGHT, pwm);
    } else {
        // Arrêt
        digitalWrite(MOTOR_RIGHT_DIR1, LOW);
        digitalWrite(MOTOR_RIGHT_DIR2, LOW);
        ledcWrite(PWM_CHANNEL_RIGHT, 0);
    }
}

void motors_stop() {
    motor_left(0);
    motor_right(0);
}

void motors_drive(int throttle, int steering) {
    // Convertit commandes type joystick en vitesses différentielles
    // throttle: -255 (arrière) à +255 (avant)
    // steering: -255 (gauche) à +255 (droite)
    
    throttle = constrain(throttle, -MAX_SPEED, MAX_SPEED);
    steering = constrain(steering, -MAX_SPEED, MAX_SPEED);
    
    // Calcul vitesses différentielles
    int leftSpeed = throttle + steering;
    int rightSpeed = throttle - steering;
    
    // Limite pour éviter dépassement
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    // Debug: afficher les commandes moteur
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500 && (throttle != 0 || steering != 0)) {
        Serial.printf("[MOTORS] Thr:%d Ste:%d → L:%d R:%d\n", 
                     throttle, steering, leftSpeed, rightSpeed);
        lastDebug = millis();
    }
    
    motor_left(leftSpeed);
    motor_right(rightSpeed);
}

#else

// Stubs quand moteurs désactivés
void motors_init() { Serial.println("[MOTORS] Skipped init (disabled)"); }
void motor_left(int) {}
void motor_right(int) {}
void motors_stop() {}
void motors_drive(int, int) {}
#endif
