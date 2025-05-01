#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// Function prototypes
float computePID(float currentAngle);

// PID Tuning Parameters (adjustable via Bluetooth)
extern float Kp;               // Proportional gain - immediate response to error
extern float Ki;               // Integral gain - eliminates steady-state error
extern float Kd;               // Derivative gain - reduces oscillation
extern float targetAngle;      // Desired stabilization angle (degrees)

// PID state variables
extern float errorSum;           // PID integral error accumulator
extern float lastError;          // Previous error for derivative calculation
extern unsigned long lastTime;   // Timing variable for control loop

#endif // PID_CONTROLLER_H
