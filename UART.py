import serial, time

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=0)
time.sleep(0.2)
tf = .4

def send(x,y,laser=0):
    x = max(0, min(180,int(x)))
    y = max(0, min(180,int(y)))
    ser.write(bytes([0xFF,x,y,laser]))

send(90,90,1)
