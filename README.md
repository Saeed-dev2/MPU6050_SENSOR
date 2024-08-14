# MPU6050 Sensor 

This project demonstrates how to interface the MPU6050 sensor with an ESP32 microcontroller using the I2C protocol. The Code initializes the MPU6050, reads data from its Accelerometer and Gyroscope, and logs the results using the ESP-IDF framework.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Hardware Required](#hardware-required)
- [Software Required](#software-required)
- [Wiring Diagram](#wiring-diagram)
- [Installation and Setup](#installation-and-setup)
- [Usage](#usage)

## Introduction
The MPU6050 is a popular sensor that combines a 3-axis accelerometer and a 3-axis gyroscope. This project illustrates how to set up the MPU6050 with an ESP32 microcontroller to read the sensor data and log it using the ESP-IDF framework.

## Features
- I2C communication with MPU6050 sensor.
- Initialization and configuration of the MPU6050.
- Reading accelerometer and gyroscope data.
- Logging sensor data to the console.

## Hardware Required
- ESP32 Development Board
- MPU6050 Sensor Module
- Jumper Wires
- Breadboard (optional)

## Software Required
- [ESP-IDF v4.4.4](https://github.com/espressif/esp-idf) (or compatible version)
- [VS Code with ESP-IDF Plugin](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) (optional)
- Serial Monitor (e.g., minicom, putty, or Arduino IDE)

## Wiring Diagram
Here is the wiring configuration:

| MPU6050 Pin | ESP32 Pin |
|-------------|-----------|
| VCC         | 3.3V      |
| GND         | GND       |
| SCL         | GPIO 22   |
| SDA         | GPIO 21   |

## Installation and Setup

### 1. Clone the Repository
```bash
git clone https://github.com/Saeed-dev2/MPU6050_SENSOR.git
cd mpu6050-esp32
```

### 2. Build and Flash the Project
```
idf.py set-target esp32
idf.py build
idf.py flash
idf.py monitor
```
## Usage
After flashing the firmware, the ESP32 will start reading data from the `MPU6050 sensor` and log the `accelerometer and gyroscope` values to the console every second.

`The output will look like this:`
```
I (1234) MPU6050: Accel: (123, 456, 789), Gyro: (-123, -456, -789)
```
