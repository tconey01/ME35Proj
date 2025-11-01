from BLE_CEEO import Listen
from machine import Pin
import time

# LED + BUZZER SETUP
led_pins = [21, 26, 25, 33, 32]
leds = [Pin(pin, Pin.OUT) for pin in led_pins]
BUZZER_PIN = 27
buzzer = Pin(BUZZER_PIN, Pin.OUT)
def all_leds(state):
    """Turn all LEDs on (1) or off (0)."""
    for led in leds:
        led.value(state)
def beep(count=3, on_time=0.15, off_time=0.15):
    """Audible buzzer beeps."""
    for _ in range(count):
        buzzer.value(1)
        time.sleep(on_time)
        buzzer.value(0)
        time.sleep(off_time)
def tornado_alarm_blink(ef_level):
    """
    Rapid LED + buzzer flashing pattern based on EF level.
    Higher EF = faster blinking and more beeps.
    """
    blink_delays = {
        0: 0.15,  # EF0
        1: 0.13,  # EF1
        2: 0.11,  # EF2
        3: 0.09,  # EF3
        4: 0.07,  # EF4
        5: 0.05   # EF5
    }
    delay = blink_delays.get(ef_level, 0.10)
    # Quick flashing + beep bursts
    for _ in range(3):
        all_leds(1)
        buzzer.value(1)
        time.sleep(delay)
        all_leds(0)
        buzzer.value(0)
        time.sleep(delay)
    time.sleep(0.2)
  
# MESSAGE HANDLING
tornado_active = False
current_ef_level = 0
message_buffer = ""
def handle_message(message):
    """Process BLE messages like START:3 or STOP:2."""
    global tornado_active, current_ef_level
    try:
        message = message.strip()
        print(f"Processing: '{message}'")
        if message.startswith('START:'):
            ef_level = int(message.split(':')[1])
            tornado_active = True
            current_ef_level = ef_level
            print(f"\n:tornado: EF{ef_level} TORNADO ALERT!")
            print(f":warning:  LED + BUZZER ALARM ACTIVATED!")
            beep(count=ef_level + 1, on_time=0.1, off_time=0.05)  # Audible cue
        elif message.startswith('STOP:'):
            ef_level = int(message.split(':')[1])
            tornado_active = False
            all_leds(0)
            buzzer.value(0)
            print(f":white_check_mark: EF{ef_level} passed. All clear.\n")
        else:
            print(f"Unknown message: {message}")
    except Exception as e:
        print(f"Error handling message: {e}")
def callback(data):
    """Parse incoming BLE data and handle complete messages."""
    global message_buffer
    try:
        decoded = bytes(data).decode('utf-8') if isinstance(data, (bytes, memoryview)) else str(data)
        print(f"Received chunk: '{decoded}'")
        message_buffer += decoded
        while '\n' in message_buffer:
            message, message_buffer = message_buffer.split('\n', 1)
            if message:
                handle_message(message)
    except Exception as e:
        print(f"Callback error: {e}")
        import sys
        sys.print_exception(e)

# MAIN LOOP
def central(name):
    global tornado_active, current_ef_level, message_buffer
    # LED + buzzer test
    print("Testing LEDs and buzzer...")
    for _ in range(2):
        all_leds(1)
        buzzer.value(1)
        time.sleep(0.3)
        all_leds(0)
        buzzer.value(0)
        time.sleep(0.3)
    print("Hardware test complete\n")
    try:
        p = Listen(name, verbose=True)
        print(":tornado: Tornado Listener Starting...")
        print("Searching for Yeller device...")
        if p.connect_up(timeout=10000):
            p.callback = callback
            print(":white_check_mark: Connected to Yeller!")
            print("Waiting for tornado alerts...\n")
            all_leds(0)
            message_buffer = ""
            while p.is_connected:
                if tornado_active:
                    tornado_alarm_blink(current_ef_level)
                else:
                    time.sleep(0.1)
        else:
            print(":x: Failed to connect to Yeller")
        print("Connection lost")
    except Exception as e:
        print("Error:", e)
        import sys
        sys.print_exception(e)
    finally:
        all_leds(0)
        buzzer.value(0)
        p.disconnect()
        print("System shut down")

# RUN CENTRAL
central('Jevon')
