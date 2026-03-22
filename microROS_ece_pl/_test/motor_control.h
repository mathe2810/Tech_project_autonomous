// Motor control for Waveshare rover
// Based on Waveshare TB6612 tutorial

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Motor A (left)
#define MOTOR_A_PWM 25
#define MOTOR_A_IN1 21
#define MOTOR_A_IN2 17

// Motor B (right)
#define MOTOR_B_PWM 26
#define MOTOR_B_IN1 22
#define MOTOR_B_IN2 23

// PWM config
#define PWM_FREQ 100000
#define PWM_RESOLUTION 8  // 0-255
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

void motor_init() {
  // Set pin modes
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM, PWM_CHANNEL_A);
  
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_PWM, PWM_CHANNEL_B);
  
  // Stop motors
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(PWM_CHANNEL_B, 0);
  
  Serial.println("[MOTORS] Initialized");
}

// Motor A control
// speed: -255 to +255 (negative=backward, positive=forward)
void motor_a(int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_A, speed);
  } else if (speed < 0) {
    // Backward
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, -speed);
  } else {
    // Stop
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
  }
}

// Motor B control
// speed: -255 to +255
void motor_b(int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_B, speed);
  } else if (speed < 0) {
    // Backward
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    ledcWrite(PWM_CHANNEL_B, -speed);
  } else {
    // Stop
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    ledcWrite(PWM_CHANNEL_B, 0);
  }
}

// Control both motors (differential drive)
// linear: forward speed (-1 to +1)
// angular: rotation speed (-1 to +1)
void motor_control(float linear, float angular) {
  // Convert to motor speeds
  // Forward: both motors same speed
  // Turn: left slower, right faster
  
  int base_speed = (int)(linear * 255.0f);  // 0-255
  int turn_speed = (int)(angular * 100.0f);  // Turning magnitude
  
  int speed_left = base_speed - turn_speed;
  int speed_right = base_speed + turn_speed;
  
  // Clamp to [-255, 255]
  speed_left = constrain(speed_left, -255, 255);
  speed_right = constrain(speed_right, -255, 255);
  
  motor_a(speed_left);
  motor_b(speed_right);
}

#endif
