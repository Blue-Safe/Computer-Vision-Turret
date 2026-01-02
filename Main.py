import cv2
import numpy as np
from ServoTesting import *
import time


samples = 5
nudgeFactor = 5


def dataProcess(ret,frame,state=0):        

    # State 0 finds all blobs, and makes the largest the target

    if state == 0:

        grayScale = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(grayScale,(15,15), 0)

        thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,31,5)
        
        mask = thresh

        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        MIN_AREA = 500

    # The second state looks just for the laser. It will detect other objects aswell,

    elif state == 1:
    
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        lower1, upper1 = (0,  60, 120), (179, 255, 255)
        lower2, upper2 = (170,60, 120), (179,255, 255)
        WL, WU = (0,0,200), (179,10,255)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask3 = cv2.inRange(hsv, WL, WU)

        mask = cv2.bitwise_or(mask1,mask2)
        mask = cv2.bitwise_or(mask,mask3)

        res = cv2.bitwise_and(frame, frame, mask=mask)

        


        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        MIN_AREA = 100

    

    # Meaningfull objects

    MO = []

    for object in contours:
        if cv2.contourArea(object) > MIN_AREA:
            MO.append(object)

    visualize = frame.copy()
    cv2.drawContours(visualize,MO,-1,(0,255,0),2)


    targetCenter = None

    if MO:
        target = max(MO,key=cv2.contourArea)
        
        x,y,w,h = cv2.boundingRect(target)
        
        cx,cy = x + w//2, y+ h//2

        targetCenter = (cx,cy)
    
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,0,255),2)
        cv2.circle(frame, (cx,cy), 5, (0,255,0), -1)

    return (frame,targetCenter)

def findDirection(laser,target):
    LX, LY = laser
    TX, TY = target

    # I'm going to account for 8 possible moves. The turret will be nudged in one of eight 
    # Cardinal Directions. Rather than 8 if statents, we'll instead return -1,0, or 1 for the
    # x and y direction. That will be left/down, stay, or right/up.
    x = 0
    y = 0
    if TX > LX:
        x = -1
    elif TX < LX:
        x = 1
    if TY > LY:
        y = -1
    elif TY < LY:
        y = 1

    return x,y

def average(list):
    sumX = 0
    sumY = 0
    for i in range(samples):
        sumX += list[i][0]
        sumY += list[i][1]
    return sumX/samples, sumY/samples


def main():
    
    servoPos = 90,90
    laserSamples = []
    targetSamples = []
    for i in range(samples):
        laserSamples.append((0,0))
        targetSamples.append((0,0))
    
    sendData(90,90,1)
    time.sleep(2)

    
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Could not open Camera")
        return
    
    print("Camera opened")

    state = 1
    
    # Start forever loop
    counter = 0
    while True:

        # grab the frame

        ret, frame = cap.read()
        if not ret:
            print("failed to grab frame")
            break
        

        # Pass off frame processing to helper. Returns display with target, and the coordinates for that target.
        # Switches between laser (0) and target (1) states.
        display,targetCenter = dataProcess(ret,frame,state)

        
        if targetCenter != None:
            

            # Seperates work into which state we are in
            if state == 1:
                print("Laser: ",targetCenter)
                # If we just got data about the laser, we want to store it, then find the direction of the target.
                laserSamples[counter] = targetCenter
                laserAvg = average(laserSamples)
                targetAvg = average(targetSamples)
                x,y = findDirection(laserAvg,targetAvg)

                # Then we pass off the direction change to the pico to move servos.
                servoPos = max(0,min(180,(servoPos[0] + (x*nudgeFactor)))),max(0,min(180,servoPos[1] + (y*nudgeFactor)))
                sendData(servoPos[0],servoPos[1],1)

                # toggle state & increase counter
                state = 0
                if counter >= samples-1:
                    counter=0
                else:
                    counter+=1
            elif state == 0:
                print("target: ",targetCenter)
                # When we get target data, we just want to store it.
                targetSamples[counter] = targetCenter

                # Toggle state
                state = 1

        cv2.imshow('Webcam',display)
        time.sleep(1)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
    

    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    main()
