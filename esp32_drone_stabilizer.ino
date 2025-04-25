/*******************************
 *        DRONE CONTROLLER      *
 * ESP32 Flight Controller Code *
 *******************************/

// === LIBRARY INCLUDES ===
#include <Wire.h>            // I2C communication
#include <BluetoothSerial.h> // Bluetooth functionality
#include <MPU9250.h>         // IMU sensor handling

// === CONSTANT DEFINITIONS ===

// Pin Assignments
const int LED_GREEN_PIN = 18;  // Green status LED (ready state)
const int LED_RED_PIN = 5;     // Red status LED (error/offline)
const int motorPins[4] = {23, 22, 21, 19};  // Motor control pins
const int I2C_SDA_PIN = 26;    // I2C data pin
const int I2C_SCL_PIN = 27;    // I2C clock pin

// PWM Configuration
const int pwmFreq = 50;        // 50Hz standard for ESCs
const int pwmResolution = 16;  // 16-bit resolution (0-65535)

// Flight Control Parameters
const float FLIP_THRESHOLD = 80.0;  // Tilt angle for emergency cutoff (degrees)

// PID Tuning Parameters
float Kp = 2.0;              // Proportional gain
float Ki = 0.5;              // Integral gain
float Kd = 1.0;              // Derivative gain
float targetAngle = 0.0;     // Desired stabilization angle

// === GLOBAL VARIABLES ===
BluetoothSerial SerialBT;    // Bluetooth interface object
MPU9250 mpu;                 // IMU sensor object

// System State Tracking
bool systemActive = false;   // Control system enabled flag
bool mpuReady = false;       // IMU initialization status
float errorSum = 0;          // PID integral accumulator
float lastError = 0;         // Previous error for derivative calculation
unsigned long lastTime = 0;  // Timing for control loop

// === FUNCTION PROTOTYPES ===
void setMotorsThrottle(int us);
float computePID(float currentAngle);
void handleBluetooth();
void emergencyShutdown();

// === SETUP FUNCTION ===
void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  SerialBT.begin("ESP32_DRONE");  // Bluetooth device name
  
  // Configure I2C with custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(2000);  // Allow IMU power stabilization

  // Configure status LEDs
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);   // Start with error state
  digitalWrite(LED_GREEN_PIN, LOW);

  // Initialize motor PWM channels
  for (int i = 0; i < 4; i++) {
    if (!ledcAttach(motorPins[i], pwmFreq, pwmResolution)) {
      Serial.printf("PWM init failed on pin %d\n", motorPins[i]);
    }
  }

  // Initialize IMU sensor
  mpuReady = mpu.setup(0x68);  // Default MPU9250 I2C address
  if (mpuReady) {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    Serial.println("IMU initialized successfully");
  } else {
    Serial.println("IMU initialization failed");
  }
}

// === MAIN CONTROL LOOP ===
void loop() {
  handleBluetooth();  // Process incoming commands

  // Safety: Idle motors when inactive
  if (!mpuReady || !systemActive) {
    setMotorsThrottle(1000);  // Minimum safe throttle
    return;
  }

  // IMU data acquisition
  if (mpu.update()) {
    // Calculate tilt angle using accelerometer data
    float ax = mpu.getAccX();
    float az = mpu.getAccZ();
    float currentAngle = atan2(ax, az) * 180.0 / PI;

    // Run PID controller
    float pidOutput = computePID(currentAngle);
    
    // Convert PID output to PWM signal (1100-1900μs typical range)
    int baseThrottle = 1100;  // Minimum operational throttle
    int throttle = constrain(
      baseThrottle + map(pidOutput, -50, 50, -100, 100),
      1000,  // Absolute minimum (stop motors)
      2000   // Maximum allowed throttle
    );
    
    // Apply to all motors
    setMotorsThrottle(throttle);

    // Debug output
    Serial.printf("Target: %.1f° | Current: %.1f° | PID: %.2f\n", 
                 targetAngle, currentAngle, pidOutput);
  }

  delay(20);  // ~50Hz control loop (adjust as needed)
}

// === MOTOR CONTROL FUNCTION ===
void setMotorsThrottle(int pulseMicroseconds) {
  /**
   * Converts throttle pulse width (μs) to PWM duty cycle
   * ESC Protocol: 1000μs (min) - 2000μs (max)
   * PWM Calculation: (μs / 20000μs period) * 65535
   */
  int duty = map(pulseMicroseconds, 1000, 2000, 3276, 6553);
  
  for (int i = 0; i < 4; i++) {
    if (!ledcWrite(motorPins[i], duty)) {
      Serial.printf("PWM write error on pin %d\n", motorPins[i]);
    }
  }
}

// === PID CONTROLLER ===
float computePID(float currentAngle) {
  /**
   * Calculates PID output for angle stabilization
   * @param currentAngle: Measured angle from IMU
   * @return: Control signal to adjust motors
   */
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;  // Convert to seconds
  lastTime = now;

  // Calculate error terms
  float error = targetAngle - currentAngle;
  errorSum += error * dt;                  // Integral term
  float dError = (error - lastError) / dt; // Derivative term
  lastError = error;

  // Compute PID components
  float P = Kp * error;
  float I = Ki * errorSum;
  float D = Kd * dError;

  // Compute PID output
  float output = P + I + D;

  // === Serial Plotter Output ===
  // Format: label1:value1,label2:value2,...
  Serial.print("Target:");
  Serial.print(targetAngle);
  Serial.print(",Current:");
  Serial.print(currentAngle);
  Serial.print(",Error:");
  Serial.print(error);
  Serial.print(",P:");
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
   * - SET: Adjust parameters (Kp, Ki, Kd, TA)
   */
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();

    // System control commands
    if (command == "START") {
      systemActive = true;
      SerialBT.println("CONTROL SYSTEM ACTIVE");
    } else if (command == "STOP") {
      systemActive = false;
      SerialBT.println("CONTROL SYSTEM STOPPED");
    }
    // Parameter adjustment commands
    else if (command.startsWith("SET:")) {
      command.remove(0, 4);  // Remove command prefix
      int separator = command.indexOf('=');
      
      if (separator != -1) {
        String param = command.substring(0, separator);
        float value = command.substring(separator + 1).toFloat();

        // Update appropriate parameter
        if (param == "Kp") Kp = value;
        else if (param == "Ki") Ki = value;
        else if (param == "Kd") Kd = value;
        else if (param == "TA") targetAngle = value;

        SerialBT.printf("Updated %s to %.2f\n", param.c_str(), value);
      }
    }
  }
}

// === SAFETY FUNCTIONS ===
void emergencyShutdown() {
  /**
   * Cuts all motor power and enters failsafe state
   * To be called when critical failure detected (e.g., flip)
   */
  setMotorsThrottle(1000);         // Minimum throttle pulse
  systemActive = false;            // Disable control system
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);
  Serial.println("EMERGENCY SHUTDOWN ACTIVATED");

  // Enter infinite error state loop
  while (true) {
    delay(250);
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));  // Blink LED
  }
}
