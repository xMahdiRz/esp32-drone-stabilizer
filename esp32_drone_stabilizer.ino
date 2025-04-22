#include <Wire.h>
#include <BluetoothSerial.h>
#include <MPU9250.h>

BluetoothSerial SerialBT;
MPU9250 mpu(Wire, 0x68);

// === Pin Definitions ===
const int LED_GREEN_PIN = 18;
const int LED_RED_PIN = 5;

// Motor pins
const int motorPins[4] = {23, 22, 21, 19};

// I2C Pins
const int I2C_SDA_PIN = 26;
const int I2C_SCL_PIN = 27;

// === PWM Settings ===
const int pwmFreq = 50;          // 50Hz for ESCs
const int pwmResolution = 16;    // 16-bit resolution

// PID Parameters
float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float targetAngle = 0.0;
float errorSum = 0, lastError = 0;

// Flags
bool systemActive = false;
bool mpuReady = false;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_DRONE");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  // Attach PWM pins using the new API
  for (int i = 0; i < 4; i++) {
    if (!ledcAttach(motorPins[i], pwmFreq, pwmResolution)) {
      Serial.printf("Failed to attach motor pin %d\n", motorPins[i]);
    }
  }

  // Initialize MPU
  if (mpu.begin() == 0) {
    Serial.println("MPU9250 connected.");
    delay(1000);
    mpu.calibrateAccelGyro();
    mpuReady = true;
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
  } else {
    Serial.println("MPU9250 connection failed.");
  }
}

void loop() {
  handleBluetooth();

  if (!mpuReady || !systemActive) {
    setMotorsThrottle(1000);
    return;
  }

  mpu.readSensor();
  float ax = mpu.getAccelX_mss();
  float az = mpu.getAccelZ_mss();
  float angle = atan2(ax, az) * 180.0 / PI;

  float controlSignal = computePID(angle);
  int baseThrottle = 1100;
  int pwmSignal = constrain(baseThrottle + map(controlSignal, -50, 50, -100, 100), 1000, 2000);
  setMotorsThrottle(pwmSignal);

  Serial.printf("Target: %.1f°, Angle: %.1f°, PID: %.2f\n", targetAngle, angle, controlSignal);

  delay(20);
}

void setMotorsThrottle(int us) {
  int duty = map(us, 1000, 2000, 3276, 6553); // 16-bit scale
  for (int i = 0; i < 4; i++) {
    if (!ledcWrite(motorPins[i], duty)) {
      Serial.printf("Failed to write PWM to pin %d\n", motorPins[i]);
    }
  }
}

float computePID(float currentAngle) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float error = targetAngle - currentAngle;
  errorSum += error * dt;
  float dError = (error - lastError) / dt;

  float output = Kp * error + Ki * errorSum + Kd * dError;
  lastError = error;

  return output;
}

void handleBluetooth() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START") {
      systemActive = true;
      SerialBT.println("System started");
    } 
    else if (cmd == "STOP") {
      systemActive = false;
      SerialBT.println("System stopped");
    } 
    else if (cmd.startsWith("SET:")) {
      cmd.remove(0, 4);
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
