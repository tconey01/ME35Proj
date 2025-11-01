# 🌪️ Tornado Warning System

A dual ESP32 BLE communication system that simulates tornado events and triggers real-time visual/audio alarms.

## Overview

This project demonstrates wireless communication between two ESP32 devices: a **Tornado Simulator** that generates random tornado events (EF0-EF5 scale) and an **Alarm System** that responds with intensity-matched LED patterns and buzzer alerts.

## Hardware Components

### Tornado Simulator (Yeller)
- ESP32 microcontroller
- DC motor (simulates wind speed)
- Servo motor (physical wind movement)
- Push button (manual tornado trigger)

### Alarm System (Listener)
- ESP32 microcontroller
- 5 LEDs (visual alerts)
- Buzzer (audible alarms)

## Features

- **6 Tornado Intensity Levels**: EF0 (gentle) to EF5 (extreme)
- **Dynamic Response**: LED blink rates and buzzer patterns scale with tornado severity
- **BLE Communication**: Simple message protocol (`START:X` / `STOP:X`)
- **Auto-start**: First tornado triggers automatically on connection
- **Manual Mode**: Button press generates additional random tornado events

## How It Works

1. Tornado Simulator connects to Alarm System via BLE
2. Random tornado event generated (EF0-EF5)
3. `START` message sent with EF level
4. DC motor spins, servo shakes for 5 seconds
5. Alarm system activates LEDs/buzzer based on intensity
6. `STOP` message sent when tornado passes
7. System resets, ready for next event

## Setup

1. Upload `yeller.py` to Tornado Simulator ESP32
2. Upload `listener.py` to Alarm System ESP32
3. Power both devices
4. Devices auto-connect and first tornado triggers automatically
5. Press button for additional tornado events

## Team

Built by Jevon, Jocasta, and Nik for the Remote Controlled Bot project.
