#  2-Link Robotic Arm Controller

A MicroPython implementation of a 2-link planar robotic arm with inverse kinematics for coordinate-based positioning.

---

##  Overview

This project controls a 2-DOF (degree of freedom) robotic arm that can move to specified x,y coordinates in its workspace. The system uses inverse kinematics to calculate the required joint angles and coordinates servo and motor movements to follow a predefined trajectory.

---

##  Features

- **Inverse Kinematics** - Converts cartesian coordinates (x, y) to joint angles
- **Workspace Validation** - Ensures target positions are within reachable area
- **Synchronized Motion** - Coordinates servo and motor movements with calculated timing
- **Trajectory Planning** - Executes a series of waypoints sequentially

---

##  Dependencies

```python
from machine import Pin, PWM, I2C
import lis3dh      # Accelerometer library
import servo       # Servo motor control
import encoder2    # Motor control with encoder feedback
```

---

##  Arm Configuration

| Parameter | Value |
|-----------|-------|
| **Link 1 Length** | 85.08 mm |
| **Link 2 Length** | 105.43 mm |
| **Base Angle** | 48.26° |
| **Workspace X** | -130 to 130 mm |
| **Workspace Y** | 70 to 139 mm |

---

##  Main Functions

### `calcPositions(x3, y3)`

Calculates the joint angles required to reach target coordinates using inverse kinematics.

**Parameters:**
- `x3` *(float)*: Target x-coordinate in mm
- `y3` *(float)*: Target y-coordinate in mm

**Returns:**
- `(theta1D, theta2D)` *(tuple)*: Joint angles in degrees
- `None`: If position is unreachable

**Example:**
```python
angles = calcPositions(100, 120)
if angles:
    theta1, theta2 = angles
    print(f"Horizontal: {theta1}°, Vertical: {theta2}°")
```

---

##  Usage

The program automatically:

1. **Generates Waypoints** - Creates 12 coordinate pairs along a diagonal path
2. **Calculates Angles** - Computes inverse kinematics for each position
3. **Executes Trajectory** - Moves through waypoints with timed motor control
4. **Synchronizes Motion** - Coordinates servos and motors for smooth movement

---

## 📐 Coordinate System

```
        Y (mm)
        ↑
   139  |     Reachable
        |     Workspace
    70  |_______________→ X (mm)
      -130              130
```

- **Origin**: Base of the arm
- **X-axis**: Horizontal movement (-130 to 130 mm)
- **Y-axis**: Vertical reach (70 to 139 mm)

---

##  Example Output

```
Input (-130, 70) → Output (45.2, 28.7)
Input (-105, 76) → Output (38.6, 31.4)
Input (-80, 82) → Output (32.1, 34.8)
...
```

---
