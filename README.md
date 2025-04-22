# ESP32 Drone Stabilization System (1-Axis)

This project implements a basic 1-axis drone stabilization system using an ESP32, MPU9250 IMU, and PID control. It supports wireless tuning and control via Bluetooth and is designed for testing fixed-axis balance.

---

## Table of Contents

- [Features](#features)
- [Hardware Used](#hardware-used)
- [Wiring](#wiring)
- [PID Control Parameters](#pid-control-parameters)
- [Bluetooth Commands](#bluetooth-commands)
- [Installation](#installation)
- [Notes](#notes)
- [Future Work](#future-work)
- [License](#license)

---

## Features

- 1-axis stabilization using a PID controller
- Bluetooth communication for real-time tuning and control
- LED status indicators for system readiness
- Compatible with ESCs via PWM control
- MPU9250 sensor for orientation feedback

---

## Hardware Used

- **ESP32** development board
- **MPU9250** 9-axis IMU
- **4 Brushless Motors** with ESCs (Electronic Speed Controllers)
- **LEDs** for status indicators (Green and Red)
- **Bluetooth (Serial)** via built-in ESP32 module

---

## Wiring

| Component     | Pin    |
|---------------|--------|
| MPU9250 SDA   | GPIO 26 |
| MPU9250 SCL   | GPIO 27 |
| ESC Motor 1   | GPIO 23 |
| ESC Motor 2   | GPIO 22 |
| ESC Motor 3   | GPIO 21 |
| ESC Motor 4   | GPIO 19 |
| Green LED     | GPIO 18 |
| Red LED       | GPIO 5  |

---

## PID Control Parameters

You can adjust the PID gains and target angle in real-time using Bluetooth commands:

| Command       | Description               |
|---------------|---------------------------|
| `SET:Kp=2.0`  | Set proportional gain     |
| `SET:Ki=0.5`  | Set integral gain         |
| `SET:Kd=1.0`  | Set derivative gain       |
| `SET:TA=0.0`  | Set target angle (degrees)|

---

## Bluetooth Commands

- `START` – Starts the stabilization system.
- `STOP` – Stops all motors.
- `SET:<param>=<value>` – Sets PID parameters or target angle.

---

## Installation

1. Install the following libraries in the Arduino IDE:
   - `MPU9250`
   - `BluetoothSerial`
2. Connect the components as per the wiring table.
3. Upload the sketch to your ESP32 board.
4. Pair your computer or phone with the ESP32 over Bluetooth (`ESP32_DRONE`).
5. Use a Bluetooth terminal app (e.g., Serial Bluetooth Terminal for Android) to send commands.

---

## Notes

- All motors are controlled identically in this demo version for 1-axis stabilization testing.
- Make sure your ESCs are properly calibrated before use.
- Use a stable power supply to avoid brownouts or resets during motor spin-up.
- System readiness is indicated by a green LED; failure to initialize the MPU9250 will show a red LED.

---

## Future Work

- Add support for full 2-axis or 3-axis stabilization
- Integrate Kalman filter for improved sensor fusion
- Log flight data to SD card or over Wi-Fi
- Build a mobile app for PID tuning and live monitoring
- Implement separate motor control for roll/pitch correction

---

## License

This project is licensed under the MIT License.
