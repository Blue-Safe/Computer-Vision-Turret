import pigpio, time, os

pi = pigpio.pi()
y = 3
x = 5
pi.set_mode(y, pigpio.OUTPUT)
pi.set_mode(x, pigpio.OUTPUT)

def setPWM(pin,time):
    pi.set_servo_pulsewidth(pin,time)

while True:
    os.system("clear")
    pin = input("Axis: ")
    if pin.lower() == "x":
        pin = 3
    elif pin.lower() == "y":
        pin = 5
    else:
        print("Invalid; 'x' or 'y' ")
        time.sleep(2)
        continue
    print()
    angle = int(input("Angle: "))
    if angle == "0":
        angle = 1000
    elif angle == "90":
        angle = 1500
    elif angle == "180":
        angle = 2000
    else:
        print("Invalid; '0','90','180'")
        time.sleep(2)
        continue

    setPWM(pin,angle)
    time.sleep(1)
    print()
    input("Hit enter to leave")




