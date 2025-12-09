import adafruit_servokit as sk
import time
from gpiozero import OutputDevice
import math

kit = sk.ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(1000,2000)
kit.servo[1].set_pulse_width_range(1000,2000)

laser = OutputDevice(17,active_high=False,initial_value=False)

def setPos(x,y):
    kit.servo[0].angle = x
    kit.servo[1].angle = y

def basic():
    while True:
        laser.on()

        kit.servo[0].angle = 90
        kit.servo[1].angle = 90

        time.sleep(1)

        kit.servo[0].angle = 0
        kit.servo[1].angle = 0

        time.sleep(1)

        kit.servo[0].angle = 180
        kit.servo[1].angle = 180

        laser.off()

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


def circle(r):
    # for theta in range(0,361):

    #     x = r * math.cos(theta)
    #     y = r * math.sin(theta)

    #     kit.servo[0].angle = x
    #     kit.servo[1].angle = y
    #     time.sleep(.5)

 
    setPos(90,90)
    time.sleep(2)
    setPos(0,90)
    time.sleep(2)
    setPos(45,135)
    time.sleep(2)
    setPos(90,180)
    time.sleep(2)
    setPos(135,135)
    time.sleep(2)
    setPos(180,90)
    time.sleep(2)
    setPos(135,45)
    time.sleep(2)
    setPos(90,0)
    time.sleep(2)
    setPos(45,45)
    time.sleep(2)
    setPos(0,90)
    time.sleep(2)
    setPos(90,90)
    time.sleep(2)
    
laser.on()
circle(1)
laser.off()
