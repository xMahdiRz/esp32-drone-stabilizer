#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Function prototypes
void setupMotors();
void setMotorsThrottle(int throttleA, int throttleB, int throttleC, int throttleD);
void emergencyShutdown();

// Motor pin definitions
extern const int MOTOR_A_PIN;  // Right Front motor control pin
extern const int MOTOR_B_PIN;  // Right Rear motor control pin
extern const int MOTOR_C_PIN;  // Left Front motor control pin
extern const int MOTOR_D_PIN;  // Left Rear motor control pin

// LED and buzzer pin definitions
extern const int LED_GREEN_PIN;   // Green status LED (system ready)
extern const int LED_RED_PIN;     // Red status LED (error/offline)
extern const int LED_YELLOW_PIN;  // Yellow status LED (calibrating)
extern const int BUZZER_PIN;      // Buzzer for alerts and notifications

// PWM Configuration
extern const int pwmFreq;        // 50Hz standard for ESCs
extern const int pwmResolution;  // 16-bit resolution (0-65535)

// System state
extern bool systemActive;

#endif // MOTOR_CONTROL_H
