from machine import Pin
from machine import PWM
import time

class Count(object):
    def __init__(self,A,B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0
        self.A.irq(self.cb,self.A.IRQ_FALLING|self.A.IRQ_RISING)
        self.B.irq(self.cb,self.B.IRQ_FALLING|self.B.IRQ_RISING)
        
    def cb(self,msg):
        other,inc = (self.B,1) if msg == self.A else (self.A,-1)
        self.counter += -inc if msg.value()!=other.value() else inc 
        
    def value(self):
        return self.counter
    
    def reset(self):
        self.counter = 0
    
class Motor(Count):
    def __init__(self,m1,m2, A, B, counts_per_rev=3829):
        self.enc = Count(A,B)
        self.M1 = PWM(m1, freq=50, duty_u16=0)
        self.M2 = PWM(m2, freq=50, duty_u16=0)
        self.counts_per_rev = counts_per_rev
        self.stop()
        
    def pos(self):
        return self.enc.value()
    
    def angle(self):
        counts = self.enc.value()
        degrees = (counts / self.counts_per_rev) * 360
        return degrees
            
    def stop(self):
        self.M1.duty_u16(0) 
        self.M2.duty_u16(0)
        
    def start(self, direction=0, speed=50):
        if direction:
            self.M1.duty_u16(int(speed*65535/100)) 
            self.M2.duty_u16(0)
        else:
            self.M1.duty_u16(0)
            self.M2.duty_u16(int(speed*65535/100))
    
    def move_to_angle(self, target_degrees, speed=50, overshoot_comp=15):
        """Move to target angle with overshoot compensation"""
        target_counts = int((target_degrees / 360) * self.counts_per_rev)
        current = self.pos()
        
        distance_degrees = abs(target_degrees - self.angle())
        actual_comp = min(overshoot_comp, distance_degrees * 0.8)
        
        if target_counts > current:
            direction = 1
            stop_target = target_counts - int((actual_comp / 360) * self.counts_per_rev)
        else:
            direction = 0
            stop_target = target_counts + int((actual_comp / 360) * self.counts_per_rev)
        
        print(f"Moving from {self.angle():.1f}° to {target_degrees}°")
        self.start(direction, speed)
        
        while True:
            current = self.pos()
            if direction == 1 and current >= stop_target:
                break
            elif direction == 0 and current <= stop_target:
                break
            time.sleep(0.01)
        
        self.stop()
        time.sleep(0.1)
        print(f"Stopped at {self.angle():.1f}°")
    
    def reset_position(self):
        self.enc.reset()

# Setup motor
Motor1 = Motor(13, 12, 16, 17, counts_per_rev=3829)


input("Press Enter when ready")


Motor1.move_to_angle(120, speed=80, overshoot_comp=20)

print("\n=== LAUNCH COMPLETE ===")
