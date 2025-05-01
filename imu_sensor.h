#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <MPU6050.h>

// Function prototypes
bool setupIMU();
void calibrateMPU();
void calibrateAngleReference();
float getCurrentAngle();

// MPU6050 sensor object
extern MPU6050 mpu;
extern bool mpuReady;

// MPU Calibration Values
extern float ax_raw, az_raw;                // Raw accelerometer values
extern float ax_smoothed, az_smoothed;      // Smoothed accelerometer values
extern float ax_offset, az_offset;          // Calibration offsets
extern float angle_offset;                  // Reference angle offset for zeroing position
extern float alpha;                         // Smoothing factor (0-1)

// Constants
extern const float FLIP_THRESHOLD;          // Tilt angle for emergency cutoff (degrees)

// I2C Pin Definitions
extern const int I2C_SDA_PIN;              // I2C data pin for MPU6050
extern const int I2C_SCL_PIN;              // I2C clock pin for MPU6050

// LED and Buzzer Pin Definitions
extern const int LED_GREEN_PIN;            // Green status LED (system ready)
extern const int LED_RED_PIN;              // Red status LED (error/offline)
extern const int LED_YELLOW_PIN;           // Yellow status LED (calibrating)
extern const int BUZZER_PIN;               // Buzzer for alerts and notifications

#endif // IMU_SENSOR_H
