#include <Arduino.h>
#include <BluetoothSerial.h>
#include "motor_control.h"

// Define PWM channels for right and left motor groups
#define RIGHT_MOTORS_CHANNEL 0  // Channel for motors A and B (right side)
#define LEFT_MOTORS_CHANNEL 1   // Channel for motors C and D (left side)

// External references to variables defined in main file
extern BluetoothSerial SerialBT;

void setupMotors() {
  // Configure motor control pins for PWM output
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  pinMode(MOTOR_C_PIN, OUTPUT);
  pinMode(MOTOR_D_PIN, OUTPUT);
  
  // Configure LED pins and buzzer
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initial state - all LEDs off, buzzer off
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Setup PWM channels for right and left motor groups
  if (!ledcAttachChannel(MOTOR_A_PIN, pwmFreq, pwmResolution, RIGHT_MOTORS_CHANNEL)) {
    Serial.printf("PWM init failed on Right Front motor (A)\n");
  }
  if (!ledcAttachChannel(MOTOR_B_PIN, pwmFreq, pwmResolution, RIGHT_MOTORS_CHANNEL)) {
    Serial.printf("PWM init failed on Right Rear motor (B)\n");
  }
  if (!ledcAttachChannel(MOTOR_C_PIN, pwmFreq, pwmResolution, LEFT_MOTORS_CHANNEL)) {
    Serial.printf("PWM init failed on Left Front motor (C)\n");
  }
  if (!ledcAttachChannel(MOTOR_D_PIN, pwmFreq, pwmResolution, LEFT_MOTORS_CHANNEL)) {
    Serial.printf("PWM init failed on Left Rear motor (D)\n");
  }
}

// === MOTOR CONTROL FUNCTION ===
void setMotorsThrottle(int throttleA, int throttleB, int throttleC, int throttleD) {
  /**
   * Sets the throttle values for all four motors
   * 
   * @param throttleA: Right Front motor throttle (1000-2000μs)
   * @param throttleB: Right Rear motor throttle (1000-2000μs)
   * @param throttleC: Left Front motor throttle (1000-2000μs)
   * @param throttleD: Left Rear motor throttle (1000-2000μs)
   * 
   * Note: 1000μs = minimum throttle (idle), 2000μs = maximum throttle
   */
  
  // Safety check - constrain all values to valid ESC range
  throttleA = constrain(throttleA, 1000, 2000);
  throttleB = constrain(throttleB, 1000, 2000);
  throttleC = constrain(throttleC, 1000, 2000);
  throttleD = constrain(throttleD, 1000, 2000);
  
  // Convert throttle values (1000-2000μs) to PWM duty cycles (3276-6553)
  int dutyA = map(throttleA, 1000, 2000, 3276, 6553);
  int dutyB = map(throttleB, 1000, 2000, 3276, 6553);
  int dutyC = map(throttleC, 1000, 2000, 3276, 6553);
  int dutyD = map(throttleD, 1000, 2000, 3276, 6553);
  
  // Apply PWM to right and left motor groups using ledcWriteChannel
  // Note: Since motors on the same side share a channel, we only need to write once per side
  // But we'll write to both pins to ensure compatibility if the API changes
  ledcWriteChannel(RIGHT_MOTORS_CHANNEL, dutyA); // Controls both A and B motors
  ledcWrite(MOTOR_B_PIN, dutyB);                // Redundant but kept for safety
  
  ledcWriteChannel(LEFT_MOTORS_CHANNEL, dutyC);  // Controls both C and D motors
  ledcWrite(MOTOR_D_PIN, dutyD);                // Redundant but kept for safety
}

// === SAFETY FUNCTIONS ===
void emergencyShutdown() {
  // Immediately stop all motors
  setMotorsThrottle(1000, 1000, 1000, 1000);
  
  // Detach PWM from all motor pins
  ledcDetach(MOTOR_A_PIN);
  ledcDetach(MOTOR_B_PIN);
  ledcDetach(MOTOR_C_PIN);
  ledcDetach(MOTOR_D_PIN);
  
  // Set pins to output LOW
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  pinMode(MOTOR_C_PIN, OUTPUT);
  pinMode(MOTOR_D_PIN, OUTPUT);
  digitalWrite(MOTOR_A_PIN, LOW);
  digitalWrite(MOTOR_B_PIN, LOW);
  digitalWrite(MOTOR_C_PIN, LOW);
  digitalWrite(MOTOR_D_PIN, LOW);
  
  // Update system state
  systemActive = false;
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, LOW);
  
  // Sound emergency alarm with buzzer
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
  
  // Send emergency notification
  Serial.println("EMERGENCY SHUTDOWN ACTIVATED - ALL MOTORS STOPPED");
  SerialBT.println("EMERGENCY SHUTDOWN - EXCESSIVE TILT DETECTED");
  SerialBT.println("RESET ESP32 TO RESTORE OPERATION");
  
  // Enter infinite error state loop with LED blinking and buzzer
  // System will require hardware reset to recover
  while (true) {
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(300);
  }
}
