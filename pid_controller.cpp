#include <Arduino.h>
#include "pid_controller.h"

// === PID CONTROLLER ===
float computePID(float currentAngle) {
  /**
   * Calculates PID output for angle stabilization
   * 
   * @param currentAngle: Measured angle from IMU (degrees)
   * @return: Control signal to adjust motors (-50 to +50 range)
   * 
   * PID Formula: output = Kp*error + Ki*∫error*dt + Kd*d(error)/dt
   */
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;  // Convert time delta to seconds
  
  // Limit minimum time step to prevent division by zero or extreme values
  if (dt < 0.001f) {
    dt = 0.001f;
  }
  
  // Calculate error (difference between target and current angle)
  float error = targetAngle - currentAngle;
  
  // Calculate the integral term (accumulated error over time)
  errorSum += error * dt;
  
  // Anti-windup: Limit the integral term to prevent excessive buildup
  errorSum = constrain(errorSum, -10.0f, 10.0f);
  
  // Calculate the derivative term (rate of change of error)
  float dError = (error - lastError) / dt;
  
  // Calculate PID output by combining all three terms
  float output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
  
  // Limit output to a reasonable range to prevent extreme motor commands
  output = constrain(output, -50.0f, 50.0f);
  
  // Store current values for next iteration
  lastError = error;
  lastTime = now;
  
  return output;
}
