import cv2

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
        cv2.imshow("Webcam",frame)

        if cv2.waitKey(1) * 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    main()
