import cv2 as cv

import numpy as np

cap = cv.VideoCapture(0)

while(1):

    # Take each frame

    _, frame = cap.read()

    # Convert BGR to HSV

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    
    lower1, upper1 = (0,  60, 120), (255, 255, 255)
    lower2, upper2 = (170,60, 120), (255,255, 255)

    mask1 = cv.inRange(hsv, lower1, upper1)
    mask2 = cv.inRange(hsv, lower2, upper2)

    mask = cv.bitwise_or(mask1, mask2)

    res  = cv.bitwise_and(frame, frame, mask=mask)

    cv.imshow('frame',frame)

    cv.imshow('res',res)

    k = cv.waitKey(5) & 0xFF

    if k == 27:

        break

        cv.destroyAllWindows()