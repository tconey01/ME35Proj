2-Link Robotic Arm Controller
A MicroPython implementation of a 2-link planar robotic arm with inverse kinematics for coordinate-based positioning.
Overview
This project controls a 2-DOF (degree of freedom) robotic arm that can move to specified x,y coordinates in its workspace. The system uses inverse kinematics to calculate the required joint angles and coordinates servo and motor movements to follow a predefined trajectory.
Features

Inverse Kinematics: Converts cartesian coordinates (x, y) to joint angles
Workspace Validation: Ensures target positions are within reachable area (-130 ≤ x ≤ 130, 70 ≤ y ≤ 139)
Synchronized Motion: Coordinates servo and motor movements with calculated timing
Trajectory Planning: Executes a series of waypoints sequentially

Dependencies

machine - MicroPython hardware control
lis3dh - Accelerometer library (imported but not used in current implementation)
servo - Servo motor control
encoder2 - Motor control with encoder feedback

Arm Configuration

Link 1 Length: 85.08 mm
Link 2 Length: 105.43 mm
Base Angle: 48.26°

Main Functions
calcPositions(x3, y3)
Calculates the joint angles required to reach target coordinates.
Parameters:

x3: Target x-coordinate
y3: Target y-coordinate

Returns:

(theta1D, theta2D): Joint angles in degrees, or None if unreachable

Usage
The code automatically:

Generates 12 waypoints along a diagonal path
Calculates required angles for each position
Moves through each waypoint with appropriate timing
Synchronizes servo positioning with motor movements

Coordinate System

Origin is at the base of the arm
X-axis: Horizontal movement (-130 to 130 mm)
Y-axis: Vertical reach (70 to 139 mm)
