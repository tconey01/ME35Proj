# Camera-Tracking Platform

Real-time object tracking system that uses OpenCV to detect red objects and rotates a motorized platform via BLE to keep them centered.

## System Components

- **Laptop (cam.py)**: OpenCV tracking + BLE central
- **ESP32 (motor.py)**: BLE peripheral + motor control

## Hardware

- Laptop with webcam
- ESP32 + DC motor + motor driver
- Rotating platform

## Setup

**Laptop:**
```
pip install opencv-python numpy bleak
```

**ESP32:**
- MicroPython with BLE_CEEO library

**Configuration:**
- Update `DEVICE_ADDRESS` in cam.py with your ESP32 MAC
- Adjust `CATCH_THRESHOLD` (default: 500px) if needed

## Usage

1. Run `motor.py` on ESP32
2. Run `python cam.py` on laptop
3. Show red object to camera
4. Platform auto-rotates to center object
5. Press 'q' to quit

## How It Works

1. Camera detects red objects via color filtering
2. Calculates offset from center frame
3. Sends BLE commands when object exceeds threshold:
   - `catch L` / `catch R`: Rotate platform
4. ESP32 drives motor to recenter object

## Protocol

- Nordic UART Service (NUS) for BLE
- 100ms throttling between messages
- Heartbeat keeps connection alive
