/*******************************
 *        DRONE CONTROLLER      *
 *    ESP32 + MPU6050 Version   *
 *      With MPU Calibration    *
 *                             *
 * 1-Axis Stabilization System *
 * Side-Based Motor Control    *
 * Persistent Calibration      *
 * Modular Code Structure      *
 *******************************/

// === LIBRARY INCLUDES ===
#include <Wire.h>             // I2C communication for sensor interface
#include <BluetoothSerial.h>  // Bluetooth functionality for wireless control
#include <MPU6050.h>          // MPU6050 6-axis IMU sensor library
#include <Preferences.h>      // For saving calibration to ESP32 flash memory

// Include our modular components
#include "motor_control.h"
#include "pid_controller.h"
#include "imu_sensor.h"
#include "bluetooth_control.h"

// === CONSTANT DEFINITIONS ===

// Pin Assignments
const int LED_GREEN_PIN = 18;   // Green status LED (system ready)
const int LED_RED_PIN = 5;      // Red status LED (error/offline)
const int LED_YELLOW_PIN = 17;  // Yellow status LED (calibrating)
const int BUZZER_PIN = 16;      // Buzzer for alerts and notifications

// Motors Layout:
//        FRONT
//    A(R)     C(L)
//       [DRONE]
//    B(R)     D(L)
//        REAR
const int MOTOR_A_PIN = 23;  // Right Front motor control pin
const int MOTOR_B_PIN = 27;  // Right Rear motor control pin
const int MOTOR_C_PIN = 26;  // Left Front motor control pin
const int MOTOR_D_PIN = 19;  // Left Rear motor control pin

const int I2C_SDA_PIN = 21;  // I2C data pin for MPU6050
const int I2C_SCL_PIN = 22;  // I2C clock pin for MPU6050

// PWM Configuration
const int pwmFreq = 50;        // 50Hz standard for ESCs
const int pwmResolution = 16;  // 16-bit resolution (0-65535)

// Flight Control Parameters
const float FLIP_THRESHOLD = 75.0;  // Tilt angle for emergency cutoff (degrees)

// PID Tuning Parameters (adjustable via Bluetooth)
float Kp = 1.0;               // Proportional gain - immediate response to error
float Ki = 0.0;               // Integral gain - eliminates steady-state error
float Kd = 0.0;               // Derivative gain - reduces oscillation
float targetAngle = 0.0;      // Desired stabilization angle (degrees)
int baseThrottle = 1100;      // Base throttle value (μs) - adjustable via Bluetooth

// === GLOBAL VARIABLES ===
BluetoothSerial SerialBT;     // Bluetooth interface object
MPU6050 mpu;                  // MPU6050 sensor object
Preferences prefs;            // Preferences object for flash storage

// System State Tracking
bool systemActive = false;    // Control system enabled flag
bool mpuReady = false;        // IMU initialization status
float errorSum = 0;           // PID integral error accumulator
float lastError = 0;          // Previous error for derivative calculation
unsigned long lastTime = 0;   // Timing variable for control loop

// MPU Calibration Values
float ax_raw, az_raw;                // Raw accelerometer values
float ax_smoothed = 0, az_smoothed = 0;  // Smoothed accelerometer values
float ax_offset = 0, az_offset = 0;      // Calibration offsets
float angle_offset = 0.0;            // Reference angle offset for zeroing position
float alpha = 0.9;                   // Smoothing factor (0-1) - adjustable via Bluetooth

// === SETUP FUNCTION ===
void setup() {
  // Initialize serial communications for debugging
  Serial.begin(115200);
  Serial.println("ESP32 Drone Controller Initializing...");
  
  // Initialize Bluetooth with device name
  SerialBT.begin("ESP32_DRONE");
  Serial.println("Bluetooth started as 'ESP32_DRONE'");

  // Configure I2C with custom pins for MPU6050
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Initialize LED and buzzer pins
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initial state - red LED on (not ready)
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Startup sound - single short beep
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize Bluetooth communication
  setupBluetooth();
  
  // Initialize IMU sensor
  mpuReady = setupIMU();
  
  // Set initial motor throttle to minimum (idle)
  setMotorsThrottle(1000, 1000, 1000, 1000);
  
  // Initialize timing variables
  lastTime = millis();
  // Calibrate the reference angle at startup
  // This sets the current orientation as "level"
  Serial.println("Calibrating angle reference...");
  calibrateAngleReference();
  Serial.printf("Angle reference calibrated to: %.2f°\n", angle_offset);
  Serial.println("Current orientation set as zero reference");
  
  Serial.println("Setup complete - Ready for Bluetooth commands");
  Serial.println("Available commands: START, STOP, CALIBRATE, ZERO, SET:param=value");
  
  // System ready - turn on green LED, turn off red LED
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, LOW);
  
  // System ready sound - ascending three beeps
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100 + i * 50);  // Increasing duration for ascending tone effect
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// === MAIN CONTROL LOOP ===
void loop() {
  // Process any incoming Bluetooth commands
  handleBluetooth();

  // Safety: Stay in idle mode if IMU not ready or system not active
  if (!mpuReady || !systemActive) {
    setMotorsThrottle(1000, 1000, 1000, 1000);  // Minimum safe throttle for all motors
    return;
  }
  
  // === IMU DATA ACQUISITION ===
  // Read raw motion data from MPU6050
  int16_t ax, ay, az;  // Accelerometer values
  int16_t gx, gy, gz;  // Gyroscope values (not used in this 1-axis implementation)
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);  // Reading gyro data even though not used yet

  // === ACCELEROMETER PROCESSING ===
  // Convert raw values to g-force units (-1.0 to +1.0g)
  ax_raw = ax / 16384.0;  // 16384 LSB/g for ±2g range
  az_raw = az / 16384.0;

  // Apply exponential smoothing filter to reduce noise
  ax_smoothed = alpha * ax_smoothed + (1 - alpha) * ax_raw;
  az_smoothed = alpha * az_smoothed + (1 - alpha) * az_raw;

  // Apply calibration offsets
  float ax_cal = ax_smoothed - ax_offset;
  float az_cal = az_smoothed - az_offset;

  // === ANGLE CALCULATION ===
  // Calculate tilt angle using accelerometer data with arctangent
  float rawAngle = atan2(ax_cal, az_cal) * 180.0 / PI;  // Convert radians to degrees
  float currentAngle = rawAngle - angle_offset;  // Apply zero reference offset
  
  // === SAFETY CHECK ===
  // Check if drone is tilted beyond the safe threshold
  float absTiltAngle = abs(currentAngle);
  if (absTiltAngle >= FLIP_THRESHOLD) {
    Serial.printf("DANGER: Excessive tilt detected! Angle: %.1f°\n", currentAngle);
    emergencyShutdown();
  }
  
  // === PID CONTROL ===
  // Get PID controller output based on current angle error
  float pidOutput = computePID(currentAngle);

  // Map PID output to motor adjustment range (-100 to +100)
  int pidAdjust = map(pidOutput, -50, 50, -100, 100);

  // === MOTOR CONTROL STRATEGY ===
  // Group motors by side (right/left) for better balance
  // Right side motors (A and B) increase throttle when tilting right
  // Left side motors (C and D) increase throttle when tilting left
  int rightThrottle = constrain(baseThrottle + pidAdjust, 1000, 2000);
  int leftThrottle = constrain(baseThrottle - pidAdjust, 1000, 2000);

  // Apply the same throttle to motors on the same side
  int throttleA = rightThrottle;  // Right front
  int throttleB = rightThrottle;  // Right rear
  int throttleC = leftThrottle;   // Left front
  int throttleD = leftThrottle;   // Left rear

  // Apply throttle values to all motors
  setMotorsThrottle(throttleA, throttleB, throttleC, throttleD);

  // === DEBUG OUTPUT ===
  // Commented out more verbose output for cleaner serial monitor
  // Serial.printf("Target: %.1f° | Current: %.1f° | PID: %.2f\n",  
  //              targetAngle, currentAngle, pidOutput);

  // Print variables in format suitable for Arduino Serial Plotter
  // Adding dummy min/max values (-360/360) to set fixed Y-axis range
  Serial.print("Min:");
  Serial.print(-360);
  Serial.print(",Max:");
  Serial.print(360);
  Serial.print(",Angle:");
  Serial.print(currentAngle);
  Serial.print(",Target:");
  Serial.print(targetAngle);
  Serial.print(",RightMotors:");
  // Scale motor values to fit within the plot range
  Serial.print(map(throttleA, 1000, 2000, -360, 360));  // Using A to represent both right motors
  Serial.print(",LeftMotors:");
  Serial.print(map(throttleC, 1000, 2000, -360, 360));  // Using C to represent both left motors
  Serial.print(",PID:");
  Serial.println(pidOutput);

  // Control loop delay for ~50Hz update rate
  delay(20);
}

// This file serves as the main entry point for the ESP32 Drone Stabilizer
// All functionality has been modularized into separate files:
// - motor_control.h/cpp: Motor control functions
// - pid_controller.h/cpp: PID control algorithm
// - imu_sensor.h/cpp: IMU sensor handling and calibration
// - bluetooth_control.h/cpp: Bluetooth communication
