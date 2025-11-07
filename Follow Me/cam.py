import cv2
import numpy as np
import asyncio
from bleak import BleakClient, BleakScanner
import time

# Nordic UART Service (NUS) UUIDs
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
# ===== TRACKING CONFIGURATION =====
MIN_AREA = 500  # Minimum pixel area to track
COLOR_THRESHOLD = 50  # Threshold for red color detection
# Boundary limits
LEFT_BOUNDARY = 950   # Maximum left distance in pixels
RIGHT_BOUNDARY = 950  # Maximum right distance in pixels
CATCH_THRESHOLD = 500 # When to send "catch" command
# BLE Configuration
DEVICE_ADDRESS = "A716D38F-F9D9-A1BA-A23D-292158F4C6FC"  # Your ESP32 MAC address
SEND_INTERVAL = 0.1  # Send data every 100ms to avoid overwhelming ESP32
class TrackingSystem:
    """Combined tracking and BLE communication system"""
    
    def __init__(self, device_address):
        self.device_address = device_address
        self.client = None
        self.connected = False
        self.last_sent_time = 0
        self.last_message = ""
        
    async def connect_ble(self):
        """Connect to ESP32 via BLE"""
        try:
            print(f"Connecting to ESP32 at {self.device_address}...")
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            
            if self.client.is_connected:
                self.connected = True
                print(f"✓ Connected to ESP32")
                
                # Start notifications
                await self.client.start_notify(UART_TX_CHAR_UUID, self.handle_esp32_response)
                return True
            else:
                print("Failed to connect")
                return False
                
        except Exception as e:
            print(f"BLE connection error: {e}")
            self.connected = False
            return False
    
    def handle_esp32_response(self, sender, data):
        """Handle responses from ESP32"""
        try:
            message = data.decode('utf-8').strip()
            if message:  # Only print non-empty messages
                print(f"ESP32: {message}")
        except Exception as e:
            print(f"Decode error: {e}")
    
    async def send_to_esp32(self, message):
        """Send message to ESP32 with proper formatting"""
        if not self.connected or not self.client:
            return False
        
        # Throttle sending to avoid overwhelming ESP32
        current_time = time.time()
        if current_time - self.last_sent_time < SEND_INTERVAL:
            return False  # Skip this message if sending too fast
        
        # Don't send duplicate messages
        if message == self.last_message:
            return False
        
        try:
            # Format message properly for ESP32
            # Ensure it ends with newline for proper parsing
            formatted_message = f"{message}\n"
            data = formatted_message.encode('utf-8')
            
            # Send the data
            await self.client.write_gatt_char(UART_RX_CHAR_UUID, data)
            
            # Debug: Show exactly what was sent
            print(f"Sent to ESP32: {message}")
            
            self.last_sent_time = current_time
            self.last_message = message
            return True
            
        except Exception as e:
            print(f"Send error: {e}")
            self.connected = False
            return False
    
    async def run_tracking(self):
        """Main tracking loop"""
        # Initialize camera
        cam = cv2.VideoCapture(1)
        if not cam.isOpened():
            print("Error: Could not open camera")
            return
        
        # Get camera dimensions
        FRAME_WIDTH = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        FRAME_HEIGHT = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        CENTER_X = FRAME_WIDTH // 2
        
        print(f"\n=== Tracking Started ===")
        print(f"Camera: {FRAME_WIDTH}x{FRAME_HEIGHT}")
        print(f"Center: {CENTER_X}px")
        print(f"Boundaries: ±{CATCH_THRESHOLD}px")
        print("Press 'q' to quit")
        print("-" * 30)
        
        try:
            while True:
                ret, frame = cam.read()
                if not ret:
                    break
                
                # ===== DETECT RED OBJECTS =====
                b, g, r = cv2.split(frame)
                red_filtered = cv2.subtract(r, g)
                red_filtered = cv2.subtract(red_filtered, b)
                
                _, red_mask = cv2.threshold(red_filtered, COLOR_THRESHOLD, 255, cv2.THRESH_BINARY)
                
                kernel = np.ones((5, 5), np.uint8)
                red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
                
                contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    biggest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(biggest_contour)
                    
                    if area > MIN_AREA:
                        # Draw green outline
                        cv2.drawContours(frame, [biggest_contour], -1, (0, 255, 0), 3)
                        
                        # Calculate center
                        M = cv2.moments(biggest_contour)
                        if M["m00"] != 0:
                            object_x = int(M["m10"] / M["m00"])
                            object_y = int(M["m01"] / M["m00"])
                            
                            # Distance from center
                            distance_from_center = object_x - CENTER_X
                            
                            # ===== DETERMINE POSITION =====
                            message = ""
                            
                            if distance_from_center < 0:
                                # LEFT of center
                                pixel_distance = abs(distance_from_center)
                                
                                if pixel_distance >= CATCH_THRESHOLD:
                                    message = "catch L"
                                    color = (0, 0, 200)  # Dark red
                                else:
                                    message = f"L{pixel_distance}"
                                    color = (0, 0, 255)  # Red
                                
                                # Print to terminal
                                print(f"Terminal: {message}")
                                
                            elif distance_from_center > 0:
                                # RIGHT of center
                                pixel_distance = distance_from_center
                                
                                if pixel_distance >= CATCH_THRESHOLD:
                                    message = "catch R"
                                    color = (200, 0, 0)  # Dark blue
                                else:
                                    message = f"R{pixel_distance}"
                                    color = (255, 0, 0)  # Blue
                                
                                # Print to terminal
                                print(f"Terminal: {message}")
                                
                            else:
                                # CENTER
                                message = "C0"
                                color = (0, 255, 0)  # Green
                                print(f"Terminal: {message}")
                            
                            # Send to ESP32
                            if self.connected and message:
                                await self.send_to_esp32(message)
                            
                            # ===== VISUALS =====
                            cv2.circle(frame, (object_x, object_y), 7, (255, 255, 255), -1)
                            cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (255, 255, 0), 1)
                            cv2.line(frame, (CENTER_X, object_y), (object_x, object_y), color, 3)
                            
                            # Boundary lines
                            left_b = CENTER_X - CATCH_THRESHOLD
                            right_b = CENTER_X + CATCH_THRESHOLD
                            cv2.line(frame, (left_b, 0), (left_b, FRAME_HEIGHT), (0, 165, 255), 2)
                            cv2.line(frame, (right_b, 0), (right_b, FRAME_HEIGHT), (0, 165, 255), 2)
                            
                            # Display message
                            cv2.putText(frame, message, (10, 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                
                # BLE Status
                status = "BLE: Connected" if self.connected else "BLE: Not Connected"
                status_color = (0, 255, 0) if self.connected else (0, 0, 255)
                cv2.putText(frame, status, (10, FRAME_HEIGHT - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
                
                cv2.imshow("ESP32 Tracker", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # Small async delay
                await asyncio.sleep(0.01)
                
        finally:
            cam.release()
            cv2.destroyAllWindows()
    
    async def disconnect(self):
        """Clean disconnect from ESP32"""
        if self.client and self.connected:
            try:
                await self.client.stop_notify(UART_TX_CHAR_UUID)
                await self.client.disconnect()
                print("Disconnected from ESP32")
            except Exception as e:
                print(f"Disconnect error: {e}")
async def main():
    """Main program entry point"""
    
    # Create tracking system
    tracker = TrackingSystem(DEVICE_ADDRESS)
    
    # Connect to ESP32
    connected = await tracker.connect_ble()
    
    if not connected:
        print("Warning: Running without BLE connection")
        print("Check that ESP32 is powered on and in range")
    
    try:
        # Run the tracking
        await tracker.run_tracking()
        
    finally:
        # Clean disconnect
        await tracker.disconnect()
        print("Program ended")
if __name__ == "__main__":
    # Run the async main function
    asyncio.run(main())
