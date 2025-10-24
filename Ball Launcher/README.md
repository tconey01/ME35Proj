# Catapult Ball Launcher

A DC motor-driven catapult system that launches table tennis balls to precise distances, controlled by an ESP32 microcontroller.

## Features

- **Customizable Distance Control**: Adjusts launch distance through motor speed (PWM) and launch angle
- **Precise Positioning**: Uses rotary encoder feedback for accurate angle control
- **Overshoot Compensation**: Accounts for motor momentum to hit target angles accurately
- **ESP32 Distance Input**: Enter target distance to calculate optimal launch parameters

## Hardware

- ESP32 microcontroller
- DC motor with rotary encoder (3829 counts/revolution)
- Motor driver
- Catapult mechanism
- Table tennis ball

## Pin Configuration

- Motor Control: GPIO 13, 12
- Encoder: GPIO 16, 17

## How It Works

The code implements two main classes:

1. **Count**: Handles rotary encoder readings using interrupt-driven quadrature decoding
2. **Motor**: Controls motor movement with position feedback
   - `move_to_angle()`: Moves catapult arm to specific angle with overshoot compensation
   - `angle()`: Returns current arm position in degrees
   - `start()`: Controls motor direction and speed (0-100%)

## Usage
```python
# Initialize motor
Motor1 = Motor(13, 12, 16, 17, counts_per_rev=3829)

# Launch at 120° angle with 80% speed
Motor1.move_to_angle(120, speed=80, overshoot_comp=20)
```

## Demo Results

Successfully launched ball into target cup at randomly selected distance. Effective range: [insert your range].

## Demo Video

[Insert demo video/GIF here]
