import machine
import time
import ustruct
from machine import Pin, PWM, I2C

# Motor pins (Cytron Robo ESP32)
LEFT_MOTOR_PIN1 = Pin(27, Pin.OUT)
LEFT_MOTOR_PIN2 = Pin(14, Pin.OUT)
RIGHT_MOTOR_PIN1 = Pin(13, Pin.OUT)
RIGHT_MOTOR_PIN2 = Pin(12, Pin.OUT)

# Encoder pins
LEFT_ENCODER_PIN = Pin(25, Pin.IN, Pin.PULL_UP)  # Grove 4
RIGHT_ENCODER_PIN = Pin(32, Pin.IN, Pin.PULL_UP)  # Grove 6

# Buzzer pin
BUZZER_PIN = PWM(Pin(23), freq=1000)

# I2C for Grove V3.0 color sensor (VEML6040)
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# Motor speed control - Slower turning for longer search
NORMAL_SPEED_ON_TIME = 25   # milliseconds motor is on (keep current speed)
NORMAL_SPEED_OFF_TIME = 25  # milliseconds motor is off (keep current speed)
TURN_SPEED_ON_TIME = 25     # milliseconds motor is on (slower turning)
TURN_SPEED_OFF_TIME = 50    # milliseconds motor is off (much slower turning)

# Encoder variables
left_encoder_count = 0
right_encoder_count = 0

# State machine states
FOLLOWING_LINE = 0
SEARCHING_RIGHT = 1
SEARCHING_LEFT = 2
COLOR_DETECTED = 3
CLEARING_CHECKPOINT = 4
RETURNING_TO_CENTER = 5  # Add this new line

current_state = FOLLOWING_LINE

# Search variables
search_start_left = 0
search_start_right = 0
SEARCH_COUNTS = 700  # Increased for even longer search duration

# Checkpoint clearing variables
checkpoint_clear_start_time = 0
CHECKPOINT_CLEAR_DURATION = 1000  # 1 second to clear checkpoint area

# VEML6040 I2C address and registers
VEML6040_ADDR = 0x10
VEML6040_CONF = 0x00
VEML6040_R_DATA = 0x08
VEML6040_G_DATA = 0x09
VEML6040_B_DATA = 0x0A
VEML6040_W_DATA = 0x0B

# Integration time constants
IT_40MS = (0b000 << 4)   # 40ms
IT_80MS = (0b001 << 4)   # 80ms
IT_160MS = (0b010 << 4)  # 160ms
IT_320MS = (0b011 << 4)  # 320ms
IT_640MS = (0b100 << 4)  # 640ms
IT_1280MS = (0b101 << 4) # 1280ms

class VEML6040:
    def __init__(self, i2c, addr=VEML6040_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.init_sensor()
    
    def init_sensor(self):
        # Configure sensor: 40ms integration time (faster), auto mode, enable sensor
        # Configuration: IT_40MS + AF_AUTO(0x00) + SD_ENABLE(0x00) = IT_40MS
        config = IT_40MS  # Use 40ms for faster measurements
        self.i2c.writeto_mem(self.addr, VEML6040_CONF, ustruct.pack('<H', config))
        time.sleep_ms(50)  # Shorter delay since we're using faster integration
        print("VEML6040 configured with 40ms integration time")
    
    def set_integration_time(self, integration_time):
        """Set custom integration time"""
        config = integration_time  # AF_AUTO and SD_ENABLE are 0x00
        self.i2c.writeto_mem(self.addr, VEML6040_CONF, ustruct.pack('<H', config))
        time.sleep_ms(50)
    
    def read_reg(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        return ustruct.unpack('<H', data)[0]
    
    def get_red(self):
        return self.read_reg(VEML6040_R_DATA)
    
    def get_green(self):
        return self.read_reg(VEML6040_G_DATA)
    
    def get_blue(self):
        return self.read_reg(VEML6040_B_DATA)
    
    def get_white(self):
        return self.read_reg(VEML6040_W_DATA)

# Initialize color sensor
try:
    color_sensor = VEML6040(i2c)
    print("VEML6040 color sensor initialized")
except Exception as e:
    print(f"Error initializing color sensor: {e}")

# Encoder interrupt handlers
def left_encoder_handler(pin):
    global left_encoder_count
    left_encoder_count += 1

def right_encoder_handler(pin):
    global right_encoder_count
    right_encoder_count += 1

# Attach interrupts
LEFT_ENCODER_PIN.irq(trigger=Pin.IRQ_RISING, handler=left_encoder_handler)
RIGHT_ENCODER_PIN.irq(trigger=Pin.IRQ_RISING, handler=right_encoder_handler)

def read_color():
    try:
        red = color_sensor.get_red()
        green = color_sensor.get_green()
        blue = color_sensor.get_blue()
        white = color_sensor.get_white()
        
        # Debug output
        print(f"R: {red} G: {green} B: {blue} W: {white}")
        
        # Color detection thresholds (adjust these based on your environment)
        # Very low white light indicates black tape
        if white < 300:
            return "BLACK"
        
        # High white light with balanced RGB indicates white background
        if white > 1000:
            return "WHITE"
        
        # Color checkpoint detection - look for dominant colors
        # Adjust these thresholds based on your colored tape
        if green > red * 1.5 and green > blue * 1.5 and green > 300:
            return "GREEN"
        
        if red > green * 1.5 and red > blue * 1.5 and red > 300:
            return "RED"
        
        if blue > red * 1.5 and blue > green * 1.5 and blue > 300:
            return "BLUE"
        
        # Default to white if unclear
        return "WHITE"
    
    except Exception as e:
        print(f"Color sensor error: {e}")
        return "WHITE"

def move_forward():
    # Simple pulsed control for TT motors
    # Left motor forward, right motor forward (both wheels roll forward)
    LEFT_MOTOR_PIN1.on()
    LEFT_MOTOR_PIN2.off()
    RIGHT_MOTOR_PIN1.off()  # Changed: right motor should also use PIN1 for forward
    RIGHT_MOTOR_PIN2.on() # Changed: PIN2 off for forward
    
    time.sleep_ms(NORMAL_SPEED_ON_TIME)
    
    # Brief pause to slow down
    LEFT_MOTOR_PIN1.off()
    LEFT_MOTOR_PIN2.off()
    RIGHT_MOTOR_PIN1.off()
    RIGHT_MOTOR_PIN2.off()
    
    time.sleep_ms(NORMAL_SPEED_OFF_TIME)

def stop_motors():
    LEFT_MOTOR_PIN1.off()
    LEFT_MOTOR_PIN2.off()
    RIGHT_MOTOR_PIN1.off()
    RIGHT_MOTOR_PIN2.off()
    
def start_return_to_center():
    global current_state, search_start_left, search_start_right
    print("Right search failed. Returning to center...")
    current_state = RETURNING_TO_CENTER
    # Reset counters for return journey
    search_start_left = left_encoder_count
    search_start_right = right_encoder_count

def continue_return_to_center():
    left_counts = left_encoder_count - search_start_left
    
    if left_counts >= SEARCH_COUNTS * 1.5:
        # Completed return to center, now start left search
        start_search_left()
    else:
        # Turn left to return to center (opposite of right search)
        LEFT_MOTOR_PIN1.off()
        LEFT_MOTOR_PIN2.on()
        RIGHT_MOTOR_PIN1.off()
        RIGHT_MOTOR_PIN2.off()
        
        time.sleep_ms(40)
        
        # Brief pause
        LEFT_MOTOR_PIN2.off()
        time.sleep_ms(TURN_SPEED_OFF_TIME)

def start_search_right():
    global current_state, search_start_left, search_start_right
    print("Lost line! Starting right search...")
    current_state = SEARCHING_RIGHT
    search_start_left = left_encoder_count
    search_start_right = right_encoder_count

def continue_search_right():
    left_counts = left_encoder_count - search_start_left
    
    if left_counts >= SEARCH_COUNTS:
        # Completed right search, return to center first
        start_return_to_center()  # Changed this line
    else:
        # Turn right: left wheel forward, right wheel off
        LEFT_MOTOR_PIN1.on()
        LEFT_MOTOR_PIN2.off()
        RIGHT_MOTOR_PIN1.off()
        RIGHT_MOTOR_PIN2.off()
        
        time.sleep_ms(TURN_SPEED_ON_TIME)
        
        # Brief pause
        LEFT_MOTOR_PIN1.off()
        time.sleep_ms(TURN_SPEED_OFF_TIME)

def start_search_left():
    global current_state, search_start_left, search_start_right
    print("Right search complete. Starting left search...")
    current_state = SEARCHING_LEFT
    search_start_left = left_encoder_count
    search_start_right = right_encoder_count

def continue_search_left():
    global current_state
    right_counts = right_encoder_count - search_start_right
    
    if right_counts >= SEARCH_COUNTS * 2:  # 60 degrees each way from center
        # Completed full search pattern, return to line following
        print("Search pattern complete. Resuming forward movement...")
        current_state = FOLLOWING_LINE
        stop_motors()
    else:
        # Turn left: right wheel forward, left wheel off
        LEFT_MOTOR_PIN1.off()
        LEFT_MOTOR_PIN2.off()
        RIGHT_MOTOR_PIN1.off()  # Changed: use PIN1 for forward motion
        RIGHT_MOTOR_PIN2.on() # Changed: PIN2 off
        
        time.sleep_ms(TURN_SPEED_ON_TIME)
        
        # Brief pause
        RIGHT_MOTOR_PIN2.off() # Changed: turn off PIN1 instead of PIN2
        time.sleep_ms(TURN_SPEED_OFF_TIME)

def handle_color_checkpoint(color):
    global current_state, checkpoint_clear_start_time
    print(f"Color checkpoint detected: {color}")
    
    stop_motors()
    
    # Play buzzer pattern based on color
    if color == "GREEN":
        play_buzzer(1000, 200)  # High tone
    elif color == "RED":
        play_buzzer(500, 200)   # Mid tone
    elif color == "BLUE":
        play_buzzer(250, 200)   # Low tone
    
    # Start clearing the checkpoint area
    checkpoint_clear_start_time = time.ticks_ms()
    current_state = CLEARING_CHECKPOINT
    print("Moving forward to clear checkpoint...")

def play_buzzer(frequency, duration_ms):
    BUZZER_PIN.freq(frequency)
    BUZZER_PIN.duty(512)  # 50% duty cycle
    time.sleep_ms(duration_ms)
    BUZZER_PIN.duty(0)    # Turn off buzzer
    time.sleep_ms(50)     # Small gap after buzzer

def main():
    global current_state
    
    print("Robot initialized. Starting line following...")
    time.sleep(1)
    
    while True:
        try:
            detected_color = read_color()
            
            if current_state == FOLLOWING_LINE:
                if detected_color == "BLACK":
                    move_forward()
                elif detected_color == "WHITE":
                    start_search_right()
                elif detected_color in ["GREEN", "RED", "BLUE"]:
                    handle_color_checkpoint(detected_color)
            
            elif current_state == SEARCHING_RIGHT:
                if detected_color == "BLACK":
                    stop_motors()
                    current_state = FOLLOWING_LINE
                    print("Line found! Resuming line following.")
                else:
                    continue_search_right()
            
            elif current_state == SEARCHING_LEFT:
                if detected_color == "BLACK":
                    stop_motors()
                    current_state = FOLLOWING_LINE
                    print("Line found! Resuming line following.")
                else:
                    continue_search_left()
                    
            elif current_state == RETURNING_TO_CENTER:
                if detected_color == "BLACK":
                    stop_motors()
                    current_state = FOLLOWING_LINE
                    print("Line found while returning! Resuming line following.")
                else:
                    continue_return_to_center()
            
            elif current_state == COLOR_DETECTED:
                # Brief pause after color detection (this state is no longer used)
                time.sleep_ms(500)
                current_state = FOLLOWING_LINE
            
            elif current_state == CLEARING_CHECKPOINT:
                # Move forward to clear the checkpoint area, ignore color readings
                # Left motor forward, right motor forward (both wheels roll forward)
                LEFT_MOTOR_PIN1.on()
                LEFT_MOTOR_PIN2.off()
                RIGHT_MOTOR_PIN1.on()  # Changed: both motors use PIN1 for forward
                RIGHT_MOTOR_PIN2.off() # Changed: PIN2 off for forward
                
                time.sleep_ms(NORMAL_SPEED_ON_TIME)
                
                # Brief pause to slow down
                LEFT_MOTOR_PIN1.off()
                LEFT_MOTOR_PIN2.off()
                RIGHT_MOTOR_PIN1.off()
                RIGHT_MOTOR_PIN2.off()
                
                time.sleep_ms(NORMAL_SPEED_OFF_TIME)
                
                if time.ticks_diff(time.ticks_ms(), checkpoint_clear_start_time) >= CHECKPOINT_CLEAR_DURATION:
                    current_state = FOLLOWING_LINE
                    print("Checkpoint cleared. Resuming normal line following.")
            
            time.sleep_ms(10)  # Shorter delay since we have pulsed control
            
        except KeyboardInterrupt:
            print("Stopping robot...")
            stop_motors()
            break
        except Exception as e:
            print(f"Error in main loop: {e}")
            stop_motors()
            time.sleep_ms(100)

# Run the main function
if __name__ == "__main__":
    main()
