from machine import Pin, PWM, I2C
import time
import machine
import lis3dh
import servo
import math
from encoder2 import Motor

ServoH = servo.Servo(5)
ServoV = servo.Servo(19)

ServoH.write_angle(0)
ServoV.write_angle(150)


theta0D = 48.26
theta0R = math.radians(theta0D)

L1 = 85.08
L2 = 105.43

#x2 = 0
y2 = L1*math.sin(theta0R)


#theta2D = 0
#theta2R = 0

#theta1R = 0
#theta1D = 0



            
mH = Motor(14, 27, 21, 22)
mV = Motor(12, 13, 33, 32)

def calcPositions (x3, y3):
    
    if -130 <= x3 <= 130 and 70 <= y3 <= 139:
        #y3 = y2+L2*math.sin(theta2R)
        theta2R = math.asin((y3-y2)/L2)
        theta2D = math.degrees(theta2R)
        #T1R = math.cos(theta1R)
        #x3 = L1*math.cos(theta0R)*math.cos(theta1R)+L2*math.cos(theta2R)*math.cos(theta1R)
        theta1R = math.acos(x3 / (L1 * math.cos(theta0R) + L2 * math.cos(theta2R)))
        theta1D = math.degrees(theta1R)
        
        #motorHPosition = theta1D * 32/3
        #motorVPosition = theta2D * 32/3
        #t2Final = 150-theta2D
        #print(theta1D, t2Final, motorHPosition, motorVPosition)
        return theta1D, theta2D
        #ServoH.write_angle(theta1D)
        #ServoV.write_angle(150-theta2D)
   


coords = []
for i in range(12):
    xC = -130 + i*25
    yC = 70 + i*6
    newC = (xC, yC)
    coords.append(newC)
    

angles = []

for (x, y) in coords:
    values = calcPositions(x, y)
    if values:  # only add valid ones
        angles.append(values)


for i, r in enumerate(angles):
    print(f"Input {coords[i]} → Output {r}")
    

#def turnMotor(angle1, angle2):
    

for i in range(len(angles)):
    servoH_target, servoV_target = angles[i]

    # Access previous row if it exists
    
    # Do your servo actions
    #stands for motorHMoveTime/motorVMoveTime
    
    
    
    ServoH.write_angle(servoH_target)
    ServoV.write_angle(150 - servoV_target)
    
    if i > 0:
        prev_servoH, prev_servoV= angles[i - 1]
        mHMT = abs(0.35 * (servoH_target - prev_servoH)/180)
        mVMT = abs(0.35 * (servoV_target - prev_servoV)/180)
        print(mHMT)
        print(mVMT)
        mH.setSpeed(10)
        time.sleep(mHMT)
        mH.stop()
        mV.setSpeed(10)
        time.sleep(mVMT)
        mV.stop()
    time.sleep(1)
    
    
    
    # move_servos(servo1_target, servo2_target)
    
    # Move motors to target positions"""

#this does a 180 rotation
#mH.setSpeed(10)
#time.sleep(0.35)
#mH.stop()


    
#ServoH.write_angle(0)
#ServoV.write_angle(150)

#calcPositions(130, 70)
