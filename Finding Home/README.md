# Line Following Robot

A MicroPython-based line following robot that uses color detection and search algorithms to navigate black lines and respond to colored checkpoints.

## Hardware Requirements

- **Cytron Robo ESP32** microcontroller
- **TT Motors** (left and right drive wheels)
- **Rotary Encoders** for wheel position tracking
- **VEML6040 Color Sensor** (Grove V3.0 compatible)
- **Buzzer** for audio feedback
- **I2C Interface** for sensor communication

## Pin Connections

| Component | Pin | Notes |
|-----------|-----|-------|
| Left Motor | 27, 14 | PIN1/PIN2 |
| Right Motor | 13, 12 | PIN1/PIN2 |
| Left Encoder | 25 | Pull-up enabled |
| Right Encoder | 32 | Pull-up enabled |
| Buzzer | 23 | PWM controlled |
| I2C (Color Sensor) | SCL: 22, SDA: 21 | 400kHz |

## Features

### Line Following
- Follows black lines on white backgrounds
- Uses color sensor to detect line presence
- Pulsed motor control for smooth movement

### Search Algorithm
When line is lost:
1. **Right Search** - turns right for ~700 encoder counts
2. **Return to Center** - returns to starting position
3. **Left Search** - turns left for wider search area
4. **Resume** - continues forward if no line found

### Color Checkpoints
Detects colored markers and provides audio feedback:
- **Green** - High tone (1000Hz)
- **Red** - Medium tone (500Hz)  
- **Blue** - Low tone (250Hz)

### State Machine
- `FOLLOWING_LINE` - Normal line tracking
- `SEARCHING_RIGHT/LEFT` - Lost line recovery
- `RETURNING_TO_CENTER` - Navigate back from failed search
- `CLEARING_CHECKPOINT` - Move past colored markers
- `COLOR_DETECTED` - Process checkpoint colors

## Usage

1. Place robot on black line
2. Run the script: `python main.py`
3. Robot will automatically follow the line and respond to colored checkpoints
4. Press Ctrl+C to stop

## Configuration

Adjust these constants for different environments:
- `SEARCH_COUNTS` - Search duration (700 default)
- Color thresholds in `read_color()` function
- Motor timing: `NORMAL_SPEED_ON_TIME`, `TURN_SPEED_ON_TIME`

## Dependencies

- MicroPython
- machine module (Pin, PWM, I2C)
- ustruct module
