#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include "imu_sensor.h"

// External references to variables defined in main file
extern BluetoothSerial SerialBT;
extern Preferences prefs;
extern const int I2C_SDA_PIN;
extern const int I2C_SCL_PIN;

bool setupIMU() {
  // Initialize I2C communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C clock for faster communication
  
  // Initialize MPU6050 sensor
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  // Verify connection to MPU6050
  Serial.println("Testing MPU6050 connection...");
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
    
    // Configure MPU6050 for flight control
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g range for better precision
    
    // Load calibration values from flash memory
    prefs.begin("drone", false);
    ax_offset = prefs.getFloat("ax_offset", 0);
    az_offset = prefs.getFloat("az_offset", 0);
    angle_offset = prefs.getFloat("ang_offset", 0);
    prefs.end();
    
    Serial.println("Loaded calibration values from flash:");
    Serial.printf("AX Offset: %.4f, AZ Offset: %.4f, Angle Offset: %.2f°\n", 
                 ax_offset, az_offset, angle_offset);
    
    return true;
  } else {
    Serial.println("MPU6050 connection failed");
    return false;
  }
}

float getCurrentAngle() {
  // Read raw motion data from MPU6050
  int16_t ax, ay, az;  // Accelerometer values
  int16_t gx, gy, gz;  // Gyroscope values (not used in this 1-axis implementation)
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);  // Reading gyro data even though not used yet

  // Convert raw values to g-force units (-1.0 to +1.0g)
  ax_raw = ax / 16384.0;  // 16384 LSB/g for ±2g range
  az_raw = az / 16384.0;

  // Apply exponential smoothing filter to reduce noise
  ax_smoothed = alpha * ax_smoothed + (1 - alpha) * ax_raw;
  az_smoothed = alpha * az_smoothed + (1 - alpha) * az_raw;

  // Apply calibration offsets
  float ax_cal = ax_smoothed - ax_offset;
  float az_cal = az_smoothed - az_offset;

  // Calculate tilt angle using accelerometer data with arctangent
  float rawAngle = atan2(ax_cal, az_cal) * 180.0 / PI;  // Convert radians to degrees
  float currentAngle = rawAngle - angle_offset;  // Apply zero reference offset
  
  return currentAngle;
}

// === MPU CALIBRATION FUNCTION ===
void calibrateMPU() {
  Serial.println("Starting MPU6050 calibration...");
  SerialBT.println("CALIBRATING SENSORS - KEEP DRONE LEVEL");
  
  // Turn on yellow LED to indicate calibration in progress
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  
  // Short beep to indicate calibration start
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Variables for averaging multiple readings
  const int numSamples = 100;
  float ax_sum = 0;
  float az_sum = 0;
  
  // Collect multiple samples to average out noise
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    // Get raw accelerometer readings
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Convert to g-force units
    float ax_g = ax / 16384.0;
    float az_g = az / 16384.0;
    
    // Add to running sum
    ax_sum += ax_g;
    az_sum += az_g;
    
    // Visual feedback during calibration
    if (i % 10 == 0) {
      Serial.print(".");
    }
    
    delay(20);  // Short delay between readings
  }
  
  // Calculate average values
  ax_offset = ax_sum / numSamples;
  az_offset = az_sum / numSamples - 1.0;  // Subtract 1g from Z-axis (gravity)
  
  // Save calibration values to flash memory
  prefs.begin("drone", false);
  prefs.putFloat("ax_offset", ax_offset);
  prefs.putFloat("az_offset", az_offset);
  prefs.end();
  
  Serial.println("\nCalibration complete!");
  Serial.printf("AX Offset: %.4f, AZ Offset: %.4f\n", ax_offset, az_offset);
  SerialBT.println("SENSOR CALIBRATION COMPLETE");
  
  // Turn off yellow LED when calibration is complete
  digitalWrite(LED_YELLOW_PIN, LOW);
  
  // Double beep to indicate calibration complete
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Reset smoothed values with new offsets
  ax_smoothed = 0;
  az_smoothed = 0;
}

// === ANGLE REFERENCE CALIBRATION ===
void calibrateAngleReference() {
  Serial.println("Calibrating angle reference (zero position)...");
  SerialBT.println("SETTING CURRENT ORIENTATION AS LEVEL");
  
  // Turn on yellow LED to indicate calibration in progress
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  
  // Short beep to indicate calibration start
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Variables for averaging multiple readings
  const int numSamples = 50;
  float angle_sum = 0;
  
  // Collect multiple samples to average out noise
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    // Get raw accelerometer readings
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Convert to g-force units
    float ax_g = ax / 16384.0;
    float az_g = az / 16384.0;
    
    // Apply existing calibration offsets
    float ax_cal = ax_g - ax_offset;
    float az_cal = az_g - az_offset;
    
    // Calculate raw angle
    float rawAngle = atan2(ax_cal, az_cal) * 180.0 / PI;
    
    // Add to running sum
    angle_sum += rawAngle;
    
    // Visual feedback during calibration
    if (i % 10 == 0) {
      Serial.print(".");
    }
    
    delay(20);  // Short delay between readings
  }
  
  // Calculate average angle
  float avgAngle = angle_sum / numSamples;
  
  // Set this as the new zero reference
  angle_offset = avgAngle;
  
  // Save to flash memory
  prefs.begin("drone", false);
  prefs.putFloat("ang_offset", angle_offset);
  prefs.end();
  
  Serial.println("\nAngle reference calibration complete!");
  Serial.printf("New angle offset: %.2f°\n", angle_offset);
  SerialBT.println("ZERO REFERENCE SET SUCCESSFULLY");
  
  // Turn off yellow LED when calibration is complete
  digitalWrite(LED_YELLOW_PIN, LOW);
  
  // Double beep to indicate calibration complete
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}
