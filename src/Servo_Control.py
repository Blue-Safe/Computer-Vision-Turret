import time
import math
import serial

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=0)


def sendData(x,y,laser=0):
    x = max(0, min(180,int(x)))
    y = max(0, min(180,int(y)))
    ser.write(bytes([0xFF,x,y,laser]))



def drawCircle(center, radius_deg, step_deg, delay):
    
    cx, cy = center
    for theta in range(0, 360, step_deg):
        
            
            
            rad = math.radians(theta)
            x = cx + radius_deg * math.cos(rad)
            y = cy + radius_deg * math.sin(rad)

            # clamp to safe servo range

            x = max(0, min(180, x))
            y = max(0, min(180, y))

            sendData(x,y,1)
            time.sleep(delay)

