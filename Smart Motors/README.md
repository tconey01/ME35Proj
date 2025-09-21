# Dual Motor Wrist Tilt Controller

Control two motors simultaneously using wrist tilt gestures detected by an accelerometer.

## Overview

This project uses an ESP32 with a LIS3DHTR accelerometer to control dual motors based on wrist tilt angle. Tilt your wrist up for more speed, down for less speed, or keep it level for medium throttle.

## Hardware Required

- ESP32 development board
- LIS3DHTR accelerometer (I2C)
- 2x DC motors with encoders
- 2x Motor driver circuits
- 2x Push buttons

## Pin Connections

```
Motor 1: Pins 13,12 + Encoders 32,39
Motor 2: Pins 27,14 + Encoders 25,33
Accelerometer: SDA=21, SCL=22
Buttons: Train=34, Play=35
```

## Usage

1. Press **D34** to calibrate wrist angles (5 positions from down to up)
2. Press **D35** to start/stop throttle control
3. Tilt wrist to control both motors simultaneously

## Features

- Calibrated wrist angle mapping
- Smooth interpolation between positions
- Real-time encoder feedback
- Dual motor synchronization

## Full Project

[https://jconey.notion.site/Smart-motor-26f1b210db528070b012e1240111beae?source=copy_link]

---

*MicroPython implementation for ESP32*
