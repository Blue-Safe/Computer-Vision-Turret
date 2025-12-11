import adafruit_servokit as sk
import time
from gpiozero import OutputDevice
import math

kit = sk.ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(1000,2000)
kit.servo[1].set_pulse_width_range(1000,2000)

laser = OutputDevice(17,active_high=True,initial_value=False)

def setPos(x,y):
    kit.servo[0].angle = x
    kit.servo[1].angle = y

def basic():
    while True:
        laser.off()

        kit.servo[0].angle = 90
        kit.servo[1].angle = 90

        time.sleep(1)

        kit.servo[0].angle = 0
        kit.servo[1].angle = 0

        time.sleep(1)

        kit.servo[0].angle = 180
        kit.servo[1].angle = 180

        

        time.sleep(2)

def leftRight():

    # Go home

    kit.servo[0].angle = 90
    kit.servo[1].angle = 90

    time.sleep(1)

    for i in range(90,181):
        kit.servo[0].angle = i
        laser.on()
        time.sleep(.05)
        laser.off()
    time.sleep(3)
    for i in range(180,0,-1):
        kit.servo[0].angle = i
        laser.on()
        time.sleep(.05)
        laser.off()
    time.sleep(3)
    kit.servo[0].angle = 90

def drawCircle(center, radius_deg, step_deg, delay):
    
    cx, cy = center
    while True:
        for theta in range(0, 360, step_deg):
            laser.on()
            rad = math.radians(theta)
            x = cx + radius_deg * math.cos(rad)
            y = cy + radius_deg * math.sin(rad)

            # clamp to safe servo range

            x = max(0, min(180, x))
            y = max(0, min(180, y))

            setPos(x, y)
            laser.off()
            time.sleep(delay)

drawCircle((90,90),45,2,.02)
print("done")