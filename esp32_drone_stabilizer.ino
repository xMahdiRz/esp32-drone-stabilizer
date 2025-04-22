#include <Wire.h>
#include <BluetoothSerial.h>
#include <MPU9250.h>

BluetoothSerial SerialBT;
MPU9250 mpu;

// === Pin Definitions ===
const int LED_GREEN_PIN = 18;
const int LED_RED_PIN = 5;
const int motorPins[4] = { 23, 22, 21, 19 };
const int I2C_SDA_PIN = 26;
const int I2C_SCL_PIN = 27;

// === PWM Settings ===
const int pwmFreq = 50;              // 50 Hz PWM frequency (standard for ESCs)
const int pwmResolution = 16;        // 16-bit resolution (max 65535 for duty cycle)

// === PID Parameters ===
float Kp = 2.0, Ki = 0.5, Kd = 1.0;   // PID gains
float targetAngle = 0.0;             // Desired angle for stabilization
float errorSum = 0, lastError = 0;   // Integral and derivative terms

// === System State Flags ===
bool systemActive = false;           // Indicates if control system is active
bool mpuReady = false;               // True if MPU initialized successfully
unsigned long lastTime = 0;          // Time tracking for PID

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_DRONE");     // Bluetooth device name

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C with custom pins
  delay(2000);                          // Give MPU time to power up

  // Setup status LEDs
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);     // Red LED on = error/not ready
  digitalWrite(LED_GREEN_PIN, LOW);    // Green LED on = ready

  // Attach PWM channels to motor pins
  for (int i = 0; i < 4; i++) {
    if (!ledcAttach(motorPins[i], pwmFreq, pwmResolution)) {
      Serial.printf("Failed to attach motor pin %d\n", motorPins[i]);
    }
  }

  // Initialize MPU9250
  if (mpu.setup(0x68)) {               // Default I2C address for MPU9250
    mpuReady = true;
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    Serial.println("MPU9250 initialized.");
  } else {
    Serial.println("MPU9250 setup failed.");
  }
}

void loop() {
  handleBluetooth();  // Check for incoming Bluetooth commands

  if (!mpuReady || !systemActive) {
    setMotorsThrottle(1000); // Idle throttle when not ready or inactive
    return;
  }

  if (mpu.update()) {
    // Get accelerometer readings
    float ax = mpu.getAccX();
    float az = mpu.getAccZ();

    // Estimate tilt angle from accelerometer
    float angle = atan2(ax, az) * 180.0 / PI;

    // Compute control signal using PID
    float controlSignal = computePID(angle);

    // Convert control output to motor PWM signal
    int baseThrottle = 1100;
    int pwmSignal = constrain(baseThrottle + map(controlSignal, -50, 50, -100, 100), 1000, 2000);

    // Send PWM to all motors
    setMotorsThrottle(pwmSignal);

    // Debug info over serial
    Serial.printf("Target: %.1f°, Angle: %.1f°, PID: %.2f\n", targetAngle, angle, controlSignal);
  }

  delay(20); // Control loop timing
}

// Converts microseconds to PWM duty and sends to ESCs
void setMotorsThrottle(int us) {
  int duty = map(us, 1000, 2000, 3276, 6553); // Map microseconds to 16-bit duty
  for (int i = 0; i < 4; i++) {
    if (!ledcWrite(motorPins[i], duty)) {
      Serial.printf("Failed to write PWM to pin %d\n", motorPins[i]);
    }
  }
}

// PID controller for angle stabilization
float computePID(float currentAngle) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // Delta time in seconds
  lastTime = now;

  float error = targetAngle - currentAngle;
  errorSum += error * dt; // Integral
  float dError = (error - lastError) / dt; // Derivative

  float output = Kp * error + Ki * errorSum + Kd * dError;
  lastError = error;

  return output;
}

// Handles Bluetooth commands to start/stop system or update parameters
void handleBluetooth() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START") {
      systemActive = true;
      SerialBT.println("System started");
    } else if (cmd == "STOP") {
      systemActive = false;
      SerialBT.println("System stopped");
    } else if (cmd.startsWith("SET:")) {
      cmd.remove(0, 4); // Remove "SET:" prefix
      int eq = cmd.indexOf('=');
      if (eq != -1) {
        String key = cmd.substring(0, eq);
        float val = cmd.substring(eq + 1).toFloat();

        if (key == "Kp") Kp = val;
        else if (key == "Ki") Ki = val;
        else if (key == "Kd") Kd = val;
        else if (key == "TA") targetAngle = val;

        SerialBT.printf("Set %s to %.2f\n", key.c_str(), val);
      }
    }
  }
}
