# esp32-self-balancing-robot
Firmware and hardware architecture for a two-wheeled balancing robot using ESP32, MPU6050 sensor fusion, and Nema17 stepper motors.

# Human-Controlled Self-Balancing Robot

## Overview

This repository contains the firmware and hardware architecture for a two-wheeled self-balancing robot. The system utilizes an ESP32 microcontroller and an MPU6050 IMU to maintain dynamic upright stability. It processes real-time angle feedback through a complementary filter and applies a Proportional-Integral-Derivative (PID) control loop to drive stepper motors. It also supports manual directional control and live PID tuning over Bluetooth.

## Core Features

**Automated Stability:** Implements a high-frequency PID control loop to dynamically adjust motor torque and direction to counteract tilt.

**Real-Time Sensor Fusion:** Processes raw accelerometer and gyroscope data using a complementary filter to generate a clean, drift-free tilt angle estimate.

**Wireless Tuning & Control:** Integrates Bluetooth communication to allow real-time tuning of PID constants (Kp, Ki, Kd) and directional control via Serial Bluetooth Terminal.

**Persistent Memory:** Saves calibrated PID parameters and target equilibrium angles directly to the ESP32's flash memory, ensuring persistence across power cycles.


## Hardware Stack
 
**Microcontroller:** ESP32 

**IMU:** MPU6050 (6-axis motion tracking) 
 
**Motor Control:** A4988 microstepping drivers 

**Actuators:**  NEMA 17 Stepper Motors

**Power:** Li-ion battery with DC-DC voltage regulation 


## Control Algorithm & Sensor Fusion

The balancing logic relies on sensor fusion to calculate the exact tilt angle. The raw gyroscope angular rate is integrated over time, while the accelerometer provides a long-term absolute reference to correct gyroscope drift.

The complementary filter is implemented using the following formula:

Current Angle = [ 0.98 × (Previous Angle + (Gyroscope Rate × Time)) ] + [ 0.02 × Accelerometer Angle ]

How it works:

Current Angle: The final, filtered tilt angle that is fed directly into the PID control loop.

0.98 (High-Pass Filter): We trust the gyroscope for 98% of the measurement. The gyroscope is extremely fast and responsive, making it perfect for reacting to sudden drops or manual movements.

Previous Angle + (Gyroscope Rate × Time): This is the integration of the gyroscope's angular velocity over the duration of a single control loop (dt). It calculates the robot's new position based on how fast it just rotated.

0.02 (Low-Pass Filter): We trust the accelerometer for 2% of the measurement. By continuously blending in a tiny fraction of the accelerometer's absolute gravity vectors, we permanently cancel out the natural "drift" of the gyroscope over time.

Accelerometer Angle: The tilt angle calculated using absolute gravity. It provides the true "zero" reference point to keep the robot upright long-term.

## Pin Configuration (ESP32)
 
**I2C (MPU6050):** SDA: GPIO 21, SCL: GPIO 22 

**Motor 1 (A4988):** STEP: GPIO 18, DIR: GPIO 19 

**Motor 2 (A4988):** STEP: GPIO 25, DIR: GPIO 26 

**Power:** ESP32 logic powered via 5V buck converter; Motors powered directly via 12V battery line.


## Demonstration

*(Insert a GIF or link to a YouTube video of the robot successfully balancing here)*
