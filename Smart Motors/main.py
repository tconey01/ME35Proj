from machine import Pin, PWM, I2C
import time
import math

class Count(object):
    def __init__(self, A, B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0
        self.A.irq(self.cb, self.A.IRQ_FALLING | self.A.IRQ_RISING)
        self.B.irq(self.cb, self.B.IRQ_FALLING | self.B.IRQ_RISING)
    
    def cb(self, msg):
        other, inc = (self.B, 1) if msg == self.A else (self.A, -1)
        self.counter += -inc if msg.value() != other.value() else inc 
        
    def value(self):
        return self.counter
    
class Motor(object):
    def __init__(self, m1, m2, A, B):
        self.enc = Count(A, B)
        self.M1 = PWM(Pin(m1), freq=1000, duty_u16=0)
        self.M2 = PWM(Pin(m2), freq=1000, duty_u16=0)
        self.stop()
        
    def pos(self):
        return self.enc.value()
            
    def stop(self):
        self.M1.duty_u16(0) 
        self.M2.duty_u16(0) 
    
    def set_speed(self, speed):
        speed = max(0, min(100, speed))
        
        if speed > 0:
            self.M1.duty_u16(int(speed * 65535 / 100)) 
            self.M2.duty_u16(0)
        else:
            self.stop()

class DualMotorController:
    def __init__(self, motor1, motor2):
        self.motor1 = motor1
        self.motor2 = motor2
        
    def stop(self):
        self.motor1.stop()
        self.motor2.stop()
        
    def set_speed(self, speed):
        self.motor1.set_speed(speed)
        self.motor2.set_speed(speed)
        
    def get_positions(self):
        return self.motor1.pos(), self.motor2.pos()

class LIS3DHTR:
    def __init__(self, i2c, addr=0x19):
        self.i2c = i2c
        self.addr = addr
        
        # Initialize sensor
        self.i2c.writeto_mem(self.addr, 0x20, b'\x47')  # Enable all axes, 50Hz
        self.i2c.writeto_mem(self.addr, 0x23, b'\x08')  # 2g range
        time.sleep(0.1)
        
    def read_raw_accel(self):
        data = self.i2c.readfrom_mem(self.addr, 0x28 | 0x80, 6)
        
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        
        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        if z > 32767: z -= 65536
        
        return x, y, z
    
    def get_accel(self):
        x_raw, y_raw, z_raw = self.read_raw_accel()
        
        x = x_raw / 16384.0
        y = y_raw / 16384.0
        z = z_raw / 16384.0
        
        return x, y, z
    
    def get_wrist_tilt(self):
        """Returns wrist tilt angle in degrees.
        0 = neutral, positive = up, negative = down"""
        x, y, z = self.get_accel()
        
        if abs(z) < 0.1:
            angle = 90 if x > 0 else -90
        else:
            angle = math.atan2(x, z) * 180 / math.pi
            
        return angle

class WristThrottleController:
    def __init__(self, dual_motors, accel):
        self.motors = dual_motors
        self.accel = accel
        
        # Calibration state
        self.calibration_points = []
        self.calibration_step = 0
        self.is_calibrated = False
        self.playback_active = False
        
        # Button setup
        self.btn_train = Pin(34, Pin.IN, Pin.PULL_UP)
        self.btn_play = Pin(35, Pin.IN, Pin.PULL_UP)
        self.last_train_state = 1
        self.last_play_state = 1
        
        # Calibration positions (angle description, motor speed)
        self.calibration_positions = [
            ("WRIST DOWN (point down ~45°)", 0),
            ("WRIST SLIGHTLY DOWN (point down ~20°)", 25),
            ("WRIST NEUTRAL (horizontal)", 50),
            ("WRIST SLIGHTLY UP (point up ~20°)", 75),
            ("WRIST UP (point up ~45°)", 100)
        ]
        
        # Smoothing
        self.angle_history = []
        self.history_size = 5
    
    def get_smoothed_angle(self):
        current_angle = self.accel.get_wrist_tilt()
        
        self.angle_history.append(current_angle)
        if len(self.angle_history) > self.history_size:
            self.angle_history.pop(0)
        
        return sum(self.angle_history) / len(self.angle_history) if self.angle_history else current_angle
    
    def check_buttons(self):
        # Training button
        train_state = self.btn_train.value()
        if train_state == 0 and self.last_train_state == 1:
            time.sleep(0.05)  # debounce
            if self.btn_train.value() == 0:
                self.handle_train_button()
        self.last_train_state = train_state
        
        # Playback button
        play_state = self.btn_play.value()
        if play_state == 0 and self.last_play_state == 1:
            time.sleep(0.05)  # debounce
            if self.btn_play.value() == 0:
                self.handle_play_button()
        self.last_play_state = play_state
    
    def handle_train_button(self):
        if self.playback_active:
            print("Stop playback first!")
            return
            
        if self.calibration_step == 0:
            print("\nStarting dual motor calibration...")
            print("Hold sensor in fist, tilt wrist for different throttle positions")
            print("Calibrating both motors together\n")
            
            position_name, motor_speed = self.calibration_positions[0]
            print(f"Position 1: {position_name}")
            print(f"Both motors at {motor_speed}% - Press D34 when ready")
            
            self.calibration_points = []
            self.calibration_step = 1
            
        elif self.calibration_step <= len(self.calibration_positions):
            angle = self.get_smoothed_angle()
            _, expected_speed = self.calibration_positions[self.calibration_step - 1]
            
            self.calibration_points.append((angle, expected_speed))
            print(f"Recorded: {angle:.1f}° -> {expected_speed}%")
            
            if self.calibration_step < len(self.calibration_positions):
                position_name, motor_speed = self.calibration_positions[self.calibration_step]
                print(f"\nPosition {self.calibration_step + 1}: {position_name}")
                print(f"Both motors at {motor_speed}% - Press D34 when ready")
                self.calibration_step += 1
            else:
                self.calibration_step = 0
                self.is_calibrated = True
                self.sort_calibration_data()
                
                print("\nCalibration complete!")
                print("Wrist angle to throttle mapping:")
                for angle, speed in self.calibration_points:
                    bars = "#" * int(speed/10) + "-" * (10 - int(speed/10))
                    print(f"  {angle:+6.1f}° -> [{bars}] {speed:3.0f}%")
                
                print("\nReady! Press D35 to start throttle control")
    
    def sort_calibration_data(self):
        self.calibration_points.sort(key=lambda x: x[0])
    
    def angle_to_speed(self, current_angle):
        if not self.calibration_points:
            return 0
        
        # Below/above range
        if current_angle <= self.calibration_points[0][0]:
            return self.calibration_points[0][1]
        elif current_angle >= self.calibration_points[-1][0]:
            return self.calibration_points[-1][1]
        else:
            # Interpolate
            for i in range(len(self.calibration_points) - 1):
                angle1, speed1 = self.calibration_points[i]
                angle2, speed2 = self.calibration_points[i + 1]
                
                if angle1 <= current_angle <= angle2:
                    if angle2 - angle1 != 0:
                        ratio = (current_angle - angle1) / (angle2 - angle1)
                        return speed1 + ratio * (speed2 - speed1)
                    else:
                        return speed1
        return 0
    
    def handle_play_button(self):
        if not self.is_calibrated:
            print("Must calibrate first! Press D34 to start")
            return
        
        if not self.playback_active:
            self.playback_active = True
            print("\nDual motor throttle control active")
            print("Tilt wrist: down=slow, level=medium, up=fast")
            print("Press D35 again to stop")
        else:
            self.playback_active = False
            self.motors.stop()
            print("\nThrottle control stopped")
            
            pos1, pos2 = self.motors.get_positions()
            print(f"Final positions - Motor 1: {pos1}, Motor 2: {pos2}")
    
    def update_playback(self):
        if not self.playback_active:
            return
        
        current_angle = self.get_smoothed_angle()
        motor_speed = self.angle_to_speed(current_angle)
        self.motors.set_speed(motor_speed)
        
        # Status display
        bars = "#" * int(motor_speed/10) + "-" * (10 - int(motor_speed/10))
        
        if current_angle < -10:
            direction = "DOWN"
        elif current_angle > 10:
            direction = "UP"
        else:
            direction = "LEVEL"
        
        pos1, pos2 = self.motors.get_positions()
        print(f"Angle: {current_angle:+6.1f}° {direction:>5} | [{bars}] {motor_speed:3.0f}% | Enc: {pos1:5d},{pos2:5d}", end="\r")
    
    def run(self):
        print("\nDual Motor Wrist Tilt Controller")
        print("Control both motors by tilting your wrist")
        print("\nD34 = Calibrate wrist angles")
        print("D35 = Start/stop throttle control")
        print("\nMotor 1: pins 13,12 + encoders 32,39")
        print("Motor 2: pins 27,14 + encoders 25,33")
        print("\nPress D34 to begin...")
        
        while True:
            self.check_buttons()
            self.update_playback()
            time.sleep(0.02)

# Initialize system
print("Starting dual motor controller...")

# Setup I2C and accelerometer
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
accel = None

for addr in [0x19, 0x18]:
    try:
        accel = LIS3DHTR(i2c, addr=addr)
        print(f"Found accelerometer at {hex(addr)}")
        break
    except:
        continue

if accel is None:
    print("ERROR: Accelerometer not found!")
    print("Check connections: SCL->22, SDA->21, VCC->3.3V, GND->GND")
    raise Exception("Accelerometer not found")

# Setup motors
print("Initializing motors...")
motor1 = Motor(13, 12, 32, 39)  # Motor 1
motor2 = Motor(27, 14, 25, 33)  # Motor 2
dual_motors = DualMotorController(motor1, motor2)

print("Motors initialized")
print("System ready")

# Run controller
controller = WristThrottleController(dual_motors, accel)
controller.run()
