# ESP32 Drone Stabilization System (1-Axis)

This project implements a basic 1-axis drone stabilization system using an ESP32, MPU6050 IMU, and PID control. It supports wireless tuning and control via Bluetooth and is designed for testing fixed-axis balance with individual motor control for left/right sides.

---

## Table of Contents

- [Features](#features)
- [Code Structure](#code-structure)
- [Hardware Used](#hardware-used)
- [Wiring](#wiring)
- [PID Control Parameters](#pid-control-parameters)
- [Bluetooth Commands](#bluetooth-commands)
- [Installation](#installation)
- [Calibration](#calibration)
- [Serial Plotting](#serial-plotting)
- [Notes](#notes)
- [Future Work](#future-work)
- [License](#license)

---

## Features

- 1-axis stabilization using a PID controller
- Bluetooth communication for real-time tuning and control
- LED status indicators for system readiness
- Compatible with ESCs via PWM control
- MPU6050 sensor for orientation feedback
- Persistent calibration data storage in ESP32 flash memory
- Sensor smoothing with adjustable alpha parameter
- Side-based motor control (independent left/right sides)
- Zero-angle reference calibration for flexible mounting
- Emergency shutdown safety system
- Modular code structure for better organization and maintainability
- Fixed Y-axis range (-360° to 360°) for Arduino Serial Plotter

---

## Code Structure

The project is organized into modular components for better maintainability:

- **esp32_drone_stabilizer.ino**: Main entry point with setup and loop functions
- **motor_control.h/cpp**: Motor control functions and emergency shutdown
- **pid_controller.h/cpp**: PID control algorithm implementation
- **imu_sensor.h/cpp**: IMU sensor handling and calibration functions
- **bluetooth_control.h/cpp**: Bluetooth communication and command processing

This modular approach makes it easier to understand, maintain, and extend the codebase.

---

## Hardware Used

- **ESP32** development board
- **MPU6050** 6-axis IMU
- **4 Brushless Motors** with ESCs (Electronic Speed Controllers)
- **LEDs** for status indicators (Green, Red, and Yellow)
- **Buzzer** for audio feedback and alerts
- **Bluetooth (Serial)** via built-in ESP32 module

---

## Wiring

| Component     | Pin    |
|---------------|--------|
| MPU6050 SDA   | GPIO 21 |
| MPU6050 SCL   | GPIO 22 |
| Motor A (Right Front) | GPIO 23 |
| Motor B (Right Rear)  | GPIO 27 |
| Motor C (Left Front)  | GPIO 26 |
| Motor D (Left Rear)   | GPIO 19 |
| Green LED     | GPIO 18 |
| Red LED       | GPIO 5  |
| Yellow LED    | GPIO 17 |
| Buzzer        | GPIO 16 |

---

## PID Control Parameters

You can adjust the PID gains and other parameters in real-time using Bluetooth commands:

| Parameter     | Description              | Default Value |
|---------------|--------------------------|--------------|
| `Kp`          | Proportional gain        | 1.0          |
| `Ki`          | Integral gain            | 0.0          |
| `Kd`          | Derivative gain          | 0.0          |
| `TA`          | Target angle (degrees)   | 0.0          |
| `BT`          | Base throttle value      | 1100         |
| `ALPHA`       | Smoothing factor (0-1)   | 0.9          |

---

## Bluetooth Commands

- `START` – Starts the stabilization system
- `STOP` – Stops all motors
- `CALIBRATE` – Initiates MPU6050 calibration (place drone on flat surface)
- `ZERO` – Sets current orientation as zero reference angle
- `SET:<param>=<value>` – Sets parameters (e.g., `SET:Kp=2.0`)

---

## Installation

1. Install the following libraries in the Arduino IDE:
   - `MPU6050` (not MPU9250)
   - `BluetoothSerial`
   - `Wire` (built-in)
   - `Preferences` (built-in)
2. Connect the components as per the wiring table
3. Upload the sketch to your ESP32 board
4. Pair your computer or phone with the ESP32 over Bluetooth (`ESP32_DRONE`)
5. Use a Bluetooth terminal app (e.g., Serial Bluetooth Terminal for Android) to send commands

---

## Calibration

The system performs two types of calibration:

### MPU6050 Sensor Calibration
- Automatically performed at first boot
- Saved to ESP32 flash memory for future use
- Can be manually triggered with the `CALIBRATE` command
- During calibration, the yellow LED will turn on and the buzzer will beep once
- When calibration completes, the yellow LED turns off and the buzzer beeps twice
- During calibration, place the drone on a flat, level surface

### Zero-Angle Reference Calibration
- Sets the current orientation as the "zero" reference
- Useful when the MPU6050 is not mounted perfectly level
- Automatically performed at startup
- Can be manually triggered with the `ZERO` command
- During calibration, the yellow LED will turn on and the buzzer will beep once
- When calibration completes, the yellow LED turns off and the buzzer beeps twice
- During calibration, hold the drone in the desired "level" position

---

## Serial Plotting

The system outputs data in a format optimized for the Arduino Serial Plotter:

- **Fixed Y-axis Range**: The system outputs dummy min/max values (-360°/360°) to set a consistent Y-axis range in the Serial Plotter
- **Plotted Values**:
  - `Min`: -360° (dummy value for Y-axis scaling)
  - `Max`: 360° (dummy value for Y-axis scaling)
  - `Angle`: Current tilt angle in degrees
  - `Target`: Target angle in degrees
  - `RightMotors`: Right motors throttle (scaled to fit in plot range)
  - `LeftMotors`: Left motors throttle (scaled to fit in plot range)
  - `PID`: PID controller output
  - `P`: Proportional component
  - `I`: Integral component
  - `D`: Derivative component

This approach provides a consistent visualization even when values fluctuate widely, making it easier to tune the PID controller and monitor system performance.

---

## Notes

- Motors are grouped by left and right sides for differential control
- Motor layout: Front motors: A (Right), C (Left) / Rear motors: B (Right), D (Left)
- Emergency shutdown activates if tilt exceeds 75 degrees with buzzer alarm
- LED status indicators:
  - Green LED: System ready and operational
  - Red LED: Error state or emergency shutdown
  - Yellow LED: Calibration in progress
- Buzzer provides audio feedback for:
  - System startup (single beep)
  - System ready (three ascending beeps)
  - Calibration start (single beep)
  - Calibration complete (double beep)
  - Emergency shutdown (three alarm beeps followed by continuous alarm)
- Calibration data is persistent across power cycles

---

## Future Work

- Add support for full 2-axis or 3-axis stabilization
- Implement complementary or Kalman filter for sensor fusion
- Log flight data to SD card or over Wi-Fi
- Build a mobile app for PID tuning and live monitoring
- Add altitude hold capabilities
- Implement low battery detection and warning

---

## License

This project is licensed under the MIT License.
