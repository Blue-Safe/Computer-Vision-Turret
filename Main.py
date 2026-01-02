import cv2
import numpy as np
# from ServoTesting import *
import time





def dataProcess(ret,frame,state=0):        


    if state == 0:

        grayScale = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(grayScale,(15,15), 0)

        thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,31,5)
        
        mask = thresh

        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        MIN_AREA = 500

    elif state == 1:
        while True:
            hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

            lower = np.array([0,255,255])
            upper = np.array([10,255,255])

            mask = cv2.inRange(hsv, lower, upper)

            res = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow('frame',frame)
            
            cv2.imshow('mask',mask)


            cv2.imshow('res',res)


            k = cv2.waitKey(5) & 0xFF

            if k == 27:

                break

    

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

def simpleCam(cap):
    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed to grab frame")
            break
        
        display,targetCenter = dataProcess(ret,frame)

        cv2.imshow('Webcam',display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

def calibration(cap,x,y): 
    
    homex,homey = 0,0
    setPos(x,y)
    laser.on()
    time.sleep(2)
    averageList = []
    while True:
        ret, frame = cap.read() 
        if not ret: 
            print("Failed to grab frame")
            break
        

        display,targetCenter = dataProcess(ret,frame)

        
        if len(averageList) < 50:
            if targetCenter != None:
                averageList.append(targetCenter)
                print(targetCenter)
        else:
            for i in range(0,len(averageList)):
                homex += averageList[i][0]
                homey += averageList[i][1]

            homex = homex/len(averageList)
            homey = homey/len(averageList)

            return homex,homey

def tracker(cap,home):
    """
    The tracker works by a P loop. It alternates between two states, Laser and Target.
    
    
    :param cap: Description
    :param home: Description
    """

    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed to grab frame")
            break
        
        display,targetCenter = dataProcess(ret,frame)

        
            


        cv2.imshow('Webcam',display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break


def laser(cap):
    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed to grab frame")
            break
        
        display,targetCenter = dataProcess(ret,frame,1)

        cv2.imshow('Webcam',display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        
def main():
    debug = True
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Could not open Camera")
        return
    
    print("Camera opened")


    laser(cap)
    

    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    main()
