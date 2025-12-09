import cv2
import numpy as np


def dataProcess(ret,frame):        

    grayScale = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(grayScale,(15,15), 0)

    thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,31,5)
    
    mask = thresh

    contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    MIN_AREA = 800

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

    return (visualize,targetCenter)



def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Could not open Camera")
        return
    
    print("Camera opened")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed to grab frame")
            break
        
        display,targetCenter = dataProcess(ret,frame)

        cv2.imshow('Webcam',display)

        if cv2.waitKey(1) * 0xFF == ord('q'):
            break
    

        

    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    main()
