from BLE_CEEO import Yell, Listen
import time
from encoder import Motor

last_message = None

def callback(msg):
    global last_message
    try:
        if isinstance(msg, bytes):
            msg = msg.decode().strip()
        print("Received:", msg)
        last_message = msg
    except Exception as e:
        print("Error decoding message:", e)

def peripheral(name, motor):
    global last_message
    p = None
    try:
        p = Yell(name, interval_us=30000, verbose=True)
        if p.connect_up():
            p.callback = callback
            print("Connected")
            
            last_heartbeat = time.time()
            
            while p.is_connected:
                # Send heartbeat every 1 second to keep connection alive
                if time.time() - last_heartbeat > 1.0:
                    p.send("alive")
                    last_heartbeat = time.time()
                
                if last_message:
                    if last_message == "catch L":
                        print("Moving motor left")
                        motor.setSpeed(0, 15)
                        time.sleep(0.2)
                        motor.stop()
                        p.send("done L")  # Send acknowledgment
                    elif last_message == "catch R":
                        print("Moving motor right")
                        motor.setSpeed(1, 15)
                        time.sleep(0.2)
                        motor.stop()
                        p.send("done R")  # Send acknowledgment
                    last_message = None
                    
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("Stopping...")
    except Exception as e:
        print("Error:", e)
    finally:
        if p:
            p.disconnect()
        motor.stop()
        
motor = Motor(12, 13, 32, 39)
peripheral('Natalie', motor)
