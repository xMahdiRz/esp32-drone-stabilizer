/*******************************
 *        DRONE CONTROLLER      *
 *    ESP32 + MPU6050 Version   *
 *      With MPU Calibration    *
 *                             *
 * 1-Axis Stabilization System *
 * Side-Based Motor Control    *
 * Persistent Calibration      *
 *******************************/

// === LIBRARY INCLUDES ===
#include <Wire.h>             // I2C communication for sensor interface
#include <BluetoothSerial.h>  // Bluetooth functionality for wireless control
#include <MPU6050.h>          // MPU6050 6-axis IMU sensor library
#include <Preferences.h>      // For saving calibration to ESP32 flash memory

// === CONSTANT DEFINITIONS ===

// Pin Assignments
const int LED_GREEN_PIN = 18;  // Green status LED (system ready)
const int LED_RED_PIN = 5;     // Red status LED (error/offline)

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

// === FUNCTION PROTOTYPES ===
void setMotorsThrottle(int throttleA, int throttleB, int throttleC, int throttleD);
float computePID(float currentAngle);
void handleBluetooth();
void emergencyShutdown();
void calibrateMPU();
void calibrateAngleReference();

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
  delay(2000);  // Allow time for sensor power stabilization
  Serial.printf("I2C initialized on pins SDA:%d, SCL:%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

  // Configure status LEDs
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);   // Start with error indication
  digitalWrite(LED_GREEN_PIN, LOW);
  Serial.println("Status LEDs initialized");

  // Initialize motor pins as outputs
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  pinMode(MOTOR_C_PIN, OUTPUT);
  pinMode(MOTOR_D_PIN, OUTPUT);
  Serial.println("Motor control pins initialized");
  
  // Setup PWM for each motor pin
  // PWM channels are automatically assigned by ledcAttach
  if (!ledcAttach(MOTOR_A_PIN, pwmFreq, pwmResolution)) {
    Serial.printf("PWM init failed on Right Front motor (A) pin %d\n", MOTOR_A_PIN);
  }
  if (!ledcAttach(MOTOR_B_PIN, pwmFreq, pwmResolution)) {
    Serial.printf("PWM init failed on Right Rear motor (B) pin %d\n", MOTOR_B_PIN);
  }
  if (!ledcAttach(MOTOR_C_PIN, pwmFreq, pwmResolution)) {
    Serial.printf("PWM init failed on Left Front motor (C) pin %d\n", MOTOR_C_PIN);
  }
  if (!ledcAttach(MOTOR_D_PIN, pwmFreq, pwmResolution)) {
    Serial.printf("PWM init failed on Left Rear motor (D) pin %d\n", MOTOR_D_PIN);
  }
  Serial.println("PWM initialized for all motors");
  
  // Initialize all motors to minimum throttle
  setMotorsThrottle(1000, 1000, 1000, 1000);
  Serial.println("Motors set to minimum throttle");

  // Initialize MPU6050 sensor
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  mpuReady = mpu.testConnection();

  if (!mpuReady) {
    Serial.println("ERROR: MPU6050 connection failed! Check wiring.");
    return;  // Keep red LED on to indicate failure
  }
  Serial.println("MPU6050 connection successful");

  // Try to load saved calibration from flash memory
  prefs.begin("mpu-cal", false);  // Open namespace in read-write mode
  if (prefs.isKey("ax_offset")) {
    // Calibration data exists in flash, load it
    ax_offset = prefs.getFloat("ax_offset");
    az_offset = prefs.getFloat("az_offset");
    Serial.printf("Loaded calibration data: ax_offset=%.4f, az_offset=%.4f\n", 
                  ax_offset, az_offset);
  } else {
    // No calibration data found, perform calibration
    Serial.println("No calibration data found, calibrating now...");
    calibrateMPU();
    
    // Save calibration data to flash
    prefs.putFloat("ax_offset", ax_offset);
    prefs.putFloat("az_offset", az_offset);
    Serial.println("Calibration saved to flash memory");
  }
  prefs.end();  // Close the Preferences

  // All systems go, switch status LEDs
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
  Serial.println("MPU6050 connected and calibrated successfully");
  
  // Calibrate the reference angle at startup
  // This sets the current orientation as "level"
  Serial.println("Calibrating angle reference...");
  calibrateAngleReference();
  Serial.printf("Angle reference calibrated to: %.2f°\n", angle_offset);
  Serial.println("Current orientation set as zero reference");
  
  Serial.println("Setup complete - Ready for Bluetooth commands");
  Serial.println("Available commands: START, STOP, CALIBRATE, ZERO, SET:param=value");
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
  Serial.print("Angle:");
  Serial.print(currentAngle);
  Serial.print(",Target:");
  Serial.print(targetAngle);
  Serial.print(",RightMotors:");
  Serial.print(throttleA);  // Using A to represent both right motors
  Serial.print(",LeftMotors:");
  Serial.print(throttleC);  // Using C to represent both left motors
  Serial.print(",PID:");
  Serial.println(pidOutput);

  // Control loop delay for ~50Hz update rate
  delay(20);
}

// === MOTOR CONTROL FUNCTION ===
void setMotorsThrottle(int throttleA, int throttleB, int throttleC, int throttleD) {
  /**
   * Sets throttle levels for all four motors individually
   * 
   * @param throttleA: Right front motor throttle (μs, 1000-2000)
   * @param throttleB: Right rear motor throttle (μs, 1000-2000)
   * @param throttleC: Left front motor throttle (μs, 1000-2000)
   * @param throttleD: Left rear motor throttle (μs, 1000-2000)
   * 
   * Standard ESC protocol:
   * - 1000μs = minimum throttle (motors off/idle)
   * - 2000μs = maximum throttle
   * 
   * PWM Conversion:
   * - PWM period at 50Hz = 20000μs
   * - Duty cycle calculation: (throttle / 20000) * 65535
   */
  
  // Convert throttle values (1000-2000μs) to PWM duty cycles (3276-6553)
  int dutyA = map(throttleA, 1000, 2000, 3276, 6553);
  int dutyB = map(throttleB, 1000, 2000, 3276, 6553);
  int dutyC = map(throttleC, 1000, 2000, 3276, 6553);
  int dutyD = map(throttleD, 1000, 2000, 3276, 6553);
  
  // Apply PWM to each motor individually
  if (!ledcWrite(MOTOR_A_PIN, dutyA)) {
    Serial.printf("PWM write error on Right Front motor (A)\n");
  }
  
  if (!ledcWrite(MOTOR_B_PIN, dutyB)) {
    Serial.printf("PWM write error on Right Rear motor (B)\n");
  }
  
  if (!ledcWrite(MOTOR_C_PIN, dutyC)) {
    Serial.printf("PWM write error on Left Front motor (C)\n");
  }
  
  if (!ledcWrite(MOTOR_D_PIN, dutyD)) {
    Serial.printf("PWM write error on Left Rear motor (D)\n");
  }
}


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
  lastTime = now;

  // Calculate error terms
  float error = targetAngle - currentAngle;  // Positive when need to tilt right
  
  // Anti-windup: Prevent integral term from growing too large
  if (abs(error) < 30.0) {  // Only accumulate error when within reasonable range
    errorSum += error * dt;  // Integral term
  }
  
  float dError = (error - lastError) / dt;  // Derivative term (rate of change)
  lastError = error;

  // Compute each PID component separately
  float P = Kp * error;           // Proportional component
  float I = Ki * errorSum;        // Integral component
  float D = Kd * dError;          // Derivative component

  // Combine components for final output
  float output = P + I + D;

  // Additional output for serial plotter visualization
  Serial.print("P:");
  Serial.print(P);
  Serial.print(",I:");
  Serial.print(I);
  Serial.print(",D:");
  Serial.print(D);
  Serial.print(",PID:");
  Serial.println(output);

  return output;
}

// === BLUETOOTH HANDLER ===
void handleBluetooth() {
  /**
   * Processes incoming Bluetooth commands:
   * - START/STOP: Enable/disable control system
   * - CALIBRATE: Run MPU6050 calibration routine
   * - ZERO: Set current orientation as level reference
   * - SET: Adjust parameters (Kp, Ki, Kd, TA, BT, ALPHA)
   */
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();  // Remove any whitespace or newlines
    Serial.printf("BT Command received: %s\n", command.c_str());

    // === System control commands ===
    if (command == "START") {
      systemActive = true;
      SerialBT.println("CONTROL SYSTEM ACTIVE");
      Serial.println("Control system activated");
    } else if (command == "STOP") {
      systemActive = false;
      SerialBT.println("CONTROL SYSTEM STOPPED");
      Serial.println("Control system stopped");
    } 
    // === Calibration commands ===
    else if (command == "CALIBRATE") {
      systemActive = false;  // Safety: disable control during calibration
      setMotorsThrottle(1000, 1000, 1000, 1000);  // Motors to minimum
      SerialBT.println("CALIBRATING MPU... KEEP DRONE FLAT AND STILL");
      
      // Run MPU calibration routine
      calibrateMPU();
      
      // Save calibration to flash memory
      prefs.begin("mpu-cal", false);
      prefs.putFloat("ax_offset", ax_offset);
      prefs.putFloat("az_offset", az_offset);
      prefs.end();
      
      SerialBT.printf("CALIBRATION COMPLETE: AX=%.4f, AZ=%.4f\n", ax_offset, az_offset);
      Serial.println("MPU calibration complete and saved");
    } else if (command == "ZERO") {
      systemActive = false;  // Safety: disable control during calibration
      setMotorsThrottle(1000, 1000, 1000, 1000);  // Motors to minimum
      SerialBT.println("CALIBRATING ANGLE REFERENCE... HOLD DRONE LEVEL");
      
      // Run angle reference calibration
      calibrateAngleReference();
      
      SerialBT.printf("CURRENT ORIENTATION SET AS ZERO (OFFSET: %.2f°)\n", angle_offset);
      Serial.printf("Angle reference calibrated to: %.2f°\n", angle_offset);
    } 
    // === Parameter adjustment commands ===
    else if (command.startsWith("SET:")) {
      command.remove(0, 4);  // Remove "SET:" prefix
      int separator = command.indexOf('=');
      
      if (separator != -1) {
        String param = command.substring(0, separator);
        float value = command.substring(separator + 1).toFloat();

        // Update appropriate parameter based on name
        if (param == "Kp") {
          Kp = value;
          SerialBT.printf("Updated proportional gain to %.2f\n", value);
        }
        else if (param == "Ki") {
          Ki = value;
          SerialBT.printf("Updated integral gain to %.2f\n", value);
        }
        else if (param == "Kd") {
          Kd = value;
          SerialBT.printf("Updated derivative gain to %.2f\n", value);
        }
        else if (param == "TA") {
          targetAngle = value;
          SerialBT.printf("Updated target angle to %.2f°\n", value);
        }
        else if (param == "BT") {
          baseThrottle = (int)value;
          baseThrottle = constrain(baseThrottle, 1000, 1800);  // Safety limit
          SerialBT.printf("Updated base throttle to %d μs\n", baseThrottle);
        }
        else if (param == "ALPHA") {
          alpha = constrain(value, 0.0, 1.0);  // Ensure alpha is between 0 and 1
          SerialBT.printf("Updated smoothing factor to %.2f\n", alpha);
        }
        else {
          SerialBT.printf("Unknown parameter: %s\n", param.c_str());
        }

        Serial.printf("Parameter %s updated to %.2f\n", param.c_str(), value);
      }
    }
    else {
      SerialBT.println("UNKNOWN COMMAND. Available: START, STOP, CALIBRATE, ZERO, SET:param=value");
    }
  }
}

// === SAFETY FUNCTIONS ===
void emergencyShutdown() {
  /**
   * Emergency shutdown procedure
   * 
   * This is a critical safety function that:
   * 1. Immediately stops all motors
   * 2. Disables the control system
   * 3. Enters a failsafe state requiring reset
   * 
   * Called when:
   * - Excessive tilt detected (beyond FLIP_THRESHOLD)
   * - Can be expanded for other emergency conditions
   */
  
  // First force all motors to minimum throttle
  ledcWrite(MOTOR_A_PIN, 3276);  // Minimum PWM duty cycle
  ledcWrite(MOTOR_B_PIN, 3276);
  ledcWrite(MOTOR_C_PIN, 3276);
  ledcWrite(MOTOR_D_PIN, 3276);
  
  // Then detach PWM to ensure motors stop completely
  ledcDetach(MOTOR_A_PIN);
  ledcDetach(MOTOR_B_PIN);
  ledcDetach(MOTOR_C_PIN);
  ledcDetach(MOTOR_D_PIN);
  
  // Set all motor pins to OUTPUT LOW for additional safety
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
  
  // Send emergency notification
  Serial.println("EMERGENCY SHUTDOWN ACTIVATED - ALL MOTORS STOPPED");
  SerialBT.println("EMERGENCY SHUTDOWN - EXCESSIVE TILT DETECTED");
  SerialBT.println("RESET ESP32 TO RESTORE OPERATION");
  
  // Enter infinite error state loop with LED blinking
  // System will require hardware reset to recover
  while (true) {
    delay(250);
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));  // Blink LED
    Serial.println("EMERGENCY: Drone shut down. Reset required.");
    delay(750);  // Reduced alert frequency after initial shutdown
  }
}

// === MPU CALIBRATION FUNCTION ===
void calibrateMPU() {
  /**
   * Calibrates the MPU6050 accelerometer
   * 
   * This procedure:
   * 1. Takes multiple samples of accelerometer data while drone is flat/still
   * 2. Calculates average offsets to normalize readings
   * 3. Updates global offset variables
   * 
   * The drone must remain completely still and on a flat surface during calibration
   */
  const int samples = 500;  // Number of samples to average
  long ax_sum = 0, az_sum = 0;

  Serial.println("Calibrating MPU... Keep it flat and still.");
  
  // Flash the green LED during calibration
  digitalWrite(LED_GREEN_PIN, HIGH);
  
  // Collect calibration samples
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    ax_sum += ax;  // Sum raw X acceleration
    az_sum += (az - 16384);  // Sum Z accel with gravity compensation
                             // 16384 = 1g in the MPU6050's scale

    // Visual feedback that calibration is in progress
    if (i % 50 == 0) {
      digitalWrite(LED_GREEN_PIN, !digitalRead(LED_GREEN_PIN));
      Serial.printf("Calibration progress: %d%%\n", (i * 100) / samples);
    }
    
    delay(5);  // Short delay between samples
  }

  // Calculate average offsets (normalized to 1g scale)
  ax_offset = ax_sum / (float)samples / 16384.0;
  az_offset = az_sum / (float)samples / 16384.0;

  // Calibration complete
  digitalWrite(LED_GREEN_PIN, HIGH);
  Serial.printf("Calibration complete. Offsets: AX=%.4f, AZ=%.4f\n", ax_offset, az_offset);
}

// === ANGLE REFERENCE CALIBRATION ===
void calibrateAngleReference() {
  /**
   * Calibrates the angle reference point
   * 
   * This sets the current physical orientation as the "zero" reference:
   * 1. Takes multiple angle readings at current orientation
   * 2. Sets the average as offset to be subtracted from future readings
   * 3. Allows the drone to be mounted at an angle but still use 0° as level
   */
  const int samples = 100;  // Number of samples to average
  float angleSum = 0.0;
  
  Serial.println("Calibrating angle reference... Keep drone in desired zero position.");
  
  // Alternate green LED during calibration
  digitalWrite(LED_GREEN_PIN, LOW);
  
  // First prepare smoothed values with a few initial readings
  for (int i = 0; i < 10; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    ax_raw = ax / 16384.0;
    az_raw = az / 16384.0;
    
    ax_smoothed = alpha * ax_smoothed + (1 - alpha) * ax_raw;
    az_smoothed = alpha * az_smoothed + (1 - alpha) * az_raw;
    
    delay(10);
  }
  
  // Take multiple samples to get a stable reference angle
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Convert raw values to g units and apply smoothing
    ax_raw = ax / 16384.0;
    az_raw = az / 16384.0;
    
    ax_smoothed = alpha * ax_smoothed + (1 - alpha) * ax_raw;
    az_smoothed = alpha * az_smoothed + (1 - alpha) * az_raw;
    
    // Apply existing calibration offsets
    float ax_cal = ax_smoothed - ax_offset;
    float az_cal = az_smoothed - az_offset;
    
    // Calculate raw angle in degrees
    float rawAngle = atan2(ax_cal, az_cal) * 180.0 / PI;
    angleSum += rawAngle;
    
    // Visual feedback that calibration is in progress
    if (i % 20 == 0) {
      digitalWrite(LED_GREEN_PIN, !digitalRead(LED_GREEN_PIN));
      Serial.printf("Angle reference progress: %d%%\n", (i * 100) / samples);
    }
    
    delay(10);  // Short delay between samples
  }
  
  // Set the current average angle as the new reference "zero"
  angle_offset = angleSum / samples;
  
  // Calibration complete
  digitalWrite(LED_GREEN_PIN, HIGH);
  Serial.printf("Angle reference calibrated to: %.2f°\n", angle_offset);
}
