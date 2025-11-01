from BLE_CEEO import Yell
from machine import Pin, PWM
import time
import random
from servo import Servo

class Motor:
    def __init__(self, m1_pin, m2_pin):
        self.M1 = PWM(Pin(m1_pin), freq=1000, duty_u16=0)
        self.M2 = PWM(Pin(m2_pin), freq=1000, duty_u16=0)
        self.stop()
        
    def stop(self):
        """Stop the motor"""
        self.M1.duty_u16(0) 
        self.M2.duty_u16(0)
        
    def spin(self, speed):
        """Spin motor at given speed (0-100)"""
        duty = int(speed * 65535 / 100)
        self.M1.duty_u16(duty) 
        self.M2.duty_u16(0)

def servo_shake(servo, ef_level, duration=0.5):
    """
    Shake servo side to side based on EF level
    Higher EF = faster, wider shaking
    """
    shake_params = {
        0: (75, 105, 0.15),   # EF0: gentle
        1: (70, 110, 0.12),   # EF1: mild
        2: (65, 115, 0.10),   # EF2: moderate
        3: (60, 120, 0.08),   # EF3: strong
        4: (55, 125, 0.06),   # EF4: violent
        5: (50, 130, 0.04)    # EF5: extreme
    }
    
    left_angle, right_angle, delay = shake_params[ef_level]
    
    # Shake back and forth
    start = time.time()
    while time.time() - start < duration:
        servo.write_angle(left_angle)
        time.sleep(delay)
        servo.write_angle(right_angle)
        time.sleep(delay)
    
    # Return to center
    servo.write_angle(90)

def random_tornado():
    """Generate random tornado parameters"""
    ef_level = random.randint(0, 5)
    ef_ranges = {
        0: (30, 42),   # EF0
        1: (43, 55),   # EF1
        2: (56, 68),   # EF2
        3: (69, 81),   # EF3
        4: (82, 91),   # EF4
        5: (92, 100)   # EF5
    }
    min_speed, max_speed = ef_ranges[ef_level]
    speed = random.randint(min_speed, max_speed)
    ef_name = f"EF{ef_level}"
    return speed, ef_level, ef_name

def run_tornado(p, motor, servo):
    """Run a single tornado event"""
    # Generate random tornado
    speed, ef_level, ef_name = random_tornado()
    print(f"\n🌪️  {ef_name} Tornado Detected!")
    print(f"   Wind Speed: {speed}%")
    
    # Send SIMPLE start message - just the EF level
    start_msg = f"START:{ef_level}\n"
    p.send(start_msg)
    print(f"   Sent: {start_msg.strip()}")
    time.sleep(0.3)  
    
    # Start motor
    motor.spin(speed)
    
    # Run for 5 seconds with servo shaking
    start_time = time.time()
    while time.time() - start_time < 5:
        servo_shake(servo, ef_level, duration=0.5)
        time.sleep(0.1)
    
    # Stop everything
    motor.stop()
    servo.write_angle(90)
    
    stop_msg = f"STOP:{ef_level}\n"
    p.send(stop_msg)
    print(f"   Sent: {stop_msg.strip()}")
    print(f"   {ef_name} stopped!\n")

def callback(data):
    """Handle incoming data from listener"""
    try:
        print(f"Received from listener: {data.decode()}")
    except:
        print(f"Received: {data}")

def peripheral(name): 
    # Setup hardware
    motor = Motor(m1_pin=13, m2_pin=12)
    button = Pin(35, Pin.IN, Pin.PULL_UP)
    servo = Servo(19)
    
    try:
        # Initialize BLE
        p = Yell(name, interval_us=30000, verbose=True)
        
        print('🌪️  Tornado Simulator Starting...')
        print('Waiting for listener connection...')
        
        if p.connect_up(timeout=10000):
            p.callback = callback
            print('✅ Connected to Listener!')
            time.sleep(1)
            
            servo.write_angle(90)  # Center servo at start
            
            # AUTO-START: Run first tornado automatically
            print('\n🚨 AUTO-STARTING TORNADO!\n')
            time.sleep(1)
            run_tornado(p, motor, servo)
            
            # Wait for button presses for additional tornados
            print('Press button for another tornado...\n')
            
            while p.is_connected:
                # Check for button press for manual tornados
                if button.value() == 0:  # Button pressed
                    run_tornado(p, motor, servo)
                    time.sleep(0.5)  # Debounce
                
                time.sleep(0.1)
                
        else:
            print('❌ Failed to connect to listener')
            
        print('Lost connection to listener')
        
    except Exception as e:
        print('Error:', e)
    finally:
        motor.stop()
        servo.write_angle(90)
        p.disconnect()
        print('Closing up')
         
# Run the peripheral
peripheral('Jevon')
