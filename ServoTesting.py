import adafruit_servokit as sk
import time

kit = sk.ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(1000,2000)
kit.servo[1].set_pulse_width_range(1000,2000)


while True:
    kit.servo[0].angle = 90
    kit.servo[1].angle = 90

    time.sleep(1)

    kit.servo[0].angle = 0
    kit.servo[1].angle = 0

    time.sleep(1)

    kit.servo[0].angle = 180
    kit.servo[1].angle = 180

    time.sleep(1)