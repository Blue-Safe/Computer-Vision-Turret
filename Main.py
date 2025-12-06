import cv2
import numpy as np
import matplotlib.pyplot as plt


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


        grayScale = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(grayScale,(15,15), 0)

        cv2.imshow("Webcam",blur)

        if cv2.waitKey(1) * 0xFF == ord('q'):
            break

        

    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    main()
