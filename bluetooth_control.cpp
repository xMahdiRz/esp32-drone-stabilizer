#include <Arduino.h>
#include "bluetooth_control.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "imu_sensor.h"

// External references to variables defined in main file
extern int baseThrottle;
extern bool systemActive;
extern float alpha;

void setupBluetooth() {
  // Initialize Bluetooth Serial communication
  SerialBT.begin("ESP32-Drone");
  Serial.println("Bluetooth device started, discoverable as 'ESP32-Drone'");
}

// === BLUETOOTH HANDLER ===
void handleBluetooth() {
  /**
   * Processes incoming Bluetooth commands
   * 
   * Command format:
   * - START: Activates the control system
   * - STOP: Deactivates the control system
   * - CALIBRATE: Runs sensor calibration
   * - ZERO: Sets current orientation as level reference
   * - SET:param=value: Updates a control parameter
   *   - Supported parameters: kp, ki, kd, target, throttle, alpha
   */
  
  if (SerialBT.available()) {
    // Read incoming message
    String message = SerialBT.readStringUntil('\n');
    message.trim();  // Remove leading/trailing whitespace
    
    Serial.printf("Received BT command: %s\n", message.c_str());
    
    // Process command based on content
    if (message.equals("START")) {
      // Activate the control system
      systemActive = true;
      digitalWrite(LED_GREEN_PIN, HIGH);
      digitalWrite(LED_RED_PIN, LOW);
      SerialBT.println("SYSTEM ACTIVATED");
      Serial.println("System activated via Bluetooth");
      
    } else if (message.equals("STOP")) {
      // Deactivate the control system
      systemActive = false;
      setMotorsThrottle(1000, 1000, 1000, 1000);  // Set all motors to idle
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, HIGH);
      SerialBT.println("SYSTEM DEACTIVATED");
      Serial.println("System deactivated via Bluetooth");
      
    } else if (message.equals("CALIBRATE")) {
      // Run sensor calibration routine
      systemActive = false;  // Safety: disable system during calibration
      setMotorsThrottle(1000, 1000, 1000, 1000);  // Set all motors to idle
      calibrateMPU();
      
    } else if (message.equals("ZERO")) {
      // Set current orientation as level reference
      systemActive = false;  // Safety: disable system during calibration
      setMotorsThrottle(1000, 1000, 1000, 1000);  // Set all motors to idle
      calibrateAngleReference();
      
    } else if (message.startsWith("SET:")) {
      // Extract parameter and value from SET command
      int colonPos = message.indexOf(':');
      int equalsPos = message.indexOf('=');
      
      if (colonPos != -1 && equalsPos != -1 && equalsPos > colonPos) {
        String param = message.substring(colonPos + 1, equalsPos);
        String valueStr = message.substring(equalsPos + 1);
        float value = valueStr.toFloat();
        
        // Update the appropriate parameter
        if (param.equals("kp")) {
          Kp = value;
          SerialBT.printf("Kp set to %.2f\n", Kp);
          Serial.printf("Updated Kp to %.2f\n", Kp);
          
        } else if (param.equals("ki")) {
          Ki = value;
          SerialBT.printf("Ki set to %.2f\n", Ki);
          Serial.printf("Updated Ki to %.2f\n", Ki);
          
        } else if (param.equals("kd")) {
          Kd = value;
          SerialBT.printf("Kd set to %.2f\n", Kd);
          Serial.printf("Updated Kd to %.2f\n", Kd);
          
        } else if (param.equals("target")) {
          targetAngle = value;
          SerialBT.printf("Target angle set to %.1f°\n", targetAngle);
          Serial.printf("Updated target angle to %.1f°\n", targetAngle);
          
        } else if (param.equals("throttle")) {
          // Convert percentage to microseconds (1000-2000 range)
          int throttleValue = 1000 + (int)(value * 10);
          throttleValue = constrain(throttleValue, 1000, 2000);
          baseThrottle = throttleValue;
          SerialBT.printf("Base throttle set to %d (%.0f%%)\n", 
                         baseThrottle, (baseThrottle - 1000) / 10.0);
          Serial.printf("Updated base throttle to %d\n", baseThrottle);
          
        } else if (param.equals("alpha")) {
          // Constrain alpha to valid range (0-1)
          alpha = constrain(value, 0.0, 1.0);
          SerialBT.printf("Smoothing factor set to %.2f\n", alpha);
          Serial.printf("Updated smoothing factor to %.2f\n", alpha);
          
        } else {
          SerialBT.printf("Unknown parameter: %s\n", param.c_str());
          Serial.printf("Unknown parameter received: %s\n", param.c_str());
        }
      } else {
        SerialBT.println("Invalid SET command format. Use SET:param=value");
        Serial.println("Invalid SET command format received");
      }
    } else {
      // Unknown command
      SerialBT.printf("Unknown command: %s\n", message.c_str());
      SerialBT.println("Available commands: START, STOP, CALIBRATE, ZERO, SET:param=value");
      Serial.printf("Unknown command received: %s\n", message.c_str());
    }
  }
}
