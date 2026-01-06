import cv2
import numpy as np
import time
import math
from enum import Enum, auto
from ServoTesting import sendData


# Tunable Variables 
NUDGE_DEG = .5
DEADBAND_PX = 10
LASER_AREA  = 5000
TARGET_AREA = 33600
LOCKED_PX = math.sqrt(TARGET_AREA/math.pi)
MAX_ANGLE = 170
MIN_ANGLE = 10
MAX_ANGLE_CHANGE = 20
JUMP_CONSTANT = 75
LOST_COUNT = 30


# Invert if turret tracks in the wrong direction
INVERT_X = True
INVERT_Y = False

# Turn on/off debugging tools
Debug = True
UI = True
AllMasks = False

class State(Enum):
    TRACK = auto()
    IDLE = auto()
    LOCKED = auto()



def largest_center_from_mask(mask, min_area,max_area):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    good = []
    for c in contours:
        if cv2.contourArea(c) >= min_area and cv2.contourArea(c) <= max_area:
            good.append(c)
    
    if not good:
        return None, None
    
    c = max(good, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    cx, cy = x + w // 2, y + h // 2
    return (cx, cy), c


def detect_laser(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Laser appears bright red and white hot to the webcam. I use three ranges, as red wraps around the scale and
    # If we don't account for the white center, it'll be a donot shape rather than our whole laser.
    RL1, RU1 = (0, 60, 120), (10, 255, 255)
    RL2, RU2 = (170, 60, 120), (179, 255, 255)
    WL, WU = (0, 0, 200), (179, 10, 255)

    m1 = cv2.inRange(hsv, RL1, RU1)
    m2 = cv2.inRange(hsv, RL2, RU2)
    m3 = cv2.inRange(hsv, WL, WU)


    # To get each pixal we picked out, we have to or all our maskes together. We have to embedded the call
    # Inside because it only takes two arguments
    mask = cv2.bitwise_or(cv2.bitwise_or(m1, m2), m3)

    # Help clean specks
    mask = cv2.medianBlur(mask, 5)

    # Pass off to our helper to get the largest blobs center.
    center, contour = largest_center_from_mask(mask, LASER_AREA-5000,LASER_AREA+5000)

    # Return the center cord, the masked frame incase we want to use it, and the actual object itself. 
    return center, mask, contour


def detect_target(frame_bgr, laser_mask=None):
    # The goal of this helper is to find the largest object in the frame while not picking up the laser.
    # Thats why we pass laser_maks through, so we automatically block out where it is from the target mask.
    
    # Prepare the frame and use thresh anaylsis to detect targets.
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (15, 15), 0)

    thresh = cv2.adaptiveThreshold( blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,31, 5)

    # This is where we block out where the laser is in the frame, or atleast where it was when we last located it.
    if laser_mask is not None:
        inv_laser = cv2.bitwise_not(laser_mask)
        thresh = cv2.bitwise_and(thresh, inv_laser)

    # Pass off largest blob detection to helper function.

    center, contour = largest_center_from_mask(thresh, TARGET_AREA-1000, TARGET_AREA+1000)
    return center, thresh, contour

# Make sure we are sending valid angles to the pico
def clamp(val, low, high):
    return max(low, min(high, val))

# To combat parralax and over forms of distorition, rather than trying to move to exactly where the target is,
# I instead decided to just figure out the direction the target is in and nudge the laser towards it.
# This helper method returns -1,0,or 1 based on if we will need to add or subtract from the current servo angle.

def sign_step(error, deadband, step_deg):
    if abs(error) <= deadband:
        return 0
    if error > 0:
        return step_deg
    else:
        return -step_deg


def move_servos(target_center,laser_center,servo_x,servo_y,speed):
        
        tx,ty = target_center
        lx,ly = laser_center

        # calculate error from laser to target
        dx = tx - lx
        dy = ty - ly

        # Convert the pixel error to something meaningful to the servos
        step_x = sign_step(dx, DEADBAND_PX, NUDGE_DEG)
        step_y = sign_step(dy, DEADBAND_PX, NUDGE_DEG)

        
        
        # If Servo moves in the wrong direction flip axis's
        if INVERT_X:
            step_x = -step_x
        if INVERT_Y:
            step_y = -step_y

        # Calculate next angle Proportionally

        Xfactor = abs(dx) // JUMP_CONSTANT
        Yfactor = abs(dy) // JUMP_CONSTANT

        # Clamp to max servo jump

        Xfactor = min(Xfactor,MAX_ANGLE_CHANGE)
        Yfactor = min(Yfactor,MAX_ANGLE_CHANGE)

        # Need to adjust magnitude without messing with sign

        if step_x != abs(step_x):
            Xfactor = Xfactor*-1
        if step_y != abs(step_y):
            Yfactor = Yfactor *-1

        # Clamp new angle to safe range
        servo_x = clamp(servo_x + step_x + Xfactor, 0, 180)
        servo_y = clamp(servo_y + step_y + Yfactor, 0, 180)

        

        # If servo angles exceed the expected view of the camera, put it back in the center.
        if (servo_x >= MAX_ANGLE or servo_x<= MIN_ANGLE):
            servo_x = 90
            servo_y = 90
        elif (servo_y >= MAX_ANGLE or servo_y<= MIN_ANGLE):
            servo_x = 90
            servo_y = 90
        

        # Send data over to pico
        sendData(servo_x, servo_y, 1)

        return servo_x,servo_y

def main():

    # Initializing code. Starts up required variables and locates laser home position.

    state = State.IDLE

    servo_x, servo_y = 90, 90
    sendData(servo_x, servo_y, 1)
    time.sleep(1.0)

    confidence = 0


    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open Camera")
        return

    # Get home position for the laser

    ret, frame = cap.read()
    laser_center,laser_mask, laser_contour = detect_laser(frame)
    homePos = laser_center

    last_laser  = None
    last_target = None

    lx,ly = 0,0
    tx,ty = 0,0

    lost_counter = 0

    while True:

        # Read in new frame each loop

        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect Laser and Target.

        laser_center, laser_mask, laser_contour = detect_laser(frame)
        target_center, target_mask, target_contour = detect_target(frame, laser_mask=laser_mask)
        
        # Store current data 

        if laser_center == None:
            laser_center = last_laser
        else:
            last_laser = laser_center
            lx,ly = laser_center

        if target_center != None:
            last_target = target_center
            tx,ty = target_center

        # calculate error from laser to target
        dx = tx - lx
        dy = ty - ly

        if state is State.IDLE:
            # In the Idle state, we do the following:
            # Start drifting home (incase it comes back in frame).
            # Wait for a target to come into frame.
            # React to target by switching States.

            speed = 1

            target_center = homePos

            servo_x,servo_y = move_servos(target_center,laser_center,servo_x,servo_y,speed)

            if target_contour != None:
                state = State.TRACK
                lost_counter = 0

        elif state is State.TRACK:
            # In the tracking state, we do the following:
            # Ensure target is still seen
            # Check if we are locked yet
            # Move towards target

            if target_contour != None:

                lost_counter = max(0,lost_counter-1)

                speed = 1

                if abs(dx) <LOCKED_PX or abs(dy) <LOCKED_PX:
                    state = State.LOCKED
                
                servo_x,servo_y = move_servos(target_center,laser_center,servo_x,servo_y,speed)
            else:
                lost_counter += 1
                if lost_counter >= LOST_COUNT:
                    state = State.IDLE
            
        elif state is State.LOCKED:
            # In the Locked state, we do the following:
            # Ensure we are within locked range 
            # Move closer to center precisly or don't move at all

            speed = 1

            if abs(dx) < LOCKED_PX and abs(dy) < LOCKED_PX:
                if abs(dx)< DEADBAND_PX or abs(dy) <DEADBAND_PX:
                    confidence = 100
                else:
                    servo_x,servo_y = move_servos(target_center,laser_center,servo_x,servo_y,speed)
            else:
                state = State.TRACK
        
        if UI:
            vis = frame.copy()

            # Draw detections onto the frame for debugging
            if laser_contour is not None:
                cv2.drawContours(vis, [laser_contour], -1, (0, 255, 255), 2)
            if target_contour is not None:
                cv2.drawContours(vis, [target_contour], -1, (0, 255, 0), 2)

            # UI
            cv2.circle(vis, (lx, ly), 5, (0, 255, 255), -1)
            cv2.circle(vis, (tx, ty), 5, (0, 255, 0), -1)
            cv2.line(vis, (lx, ly), (tx, ty), (255, 255, 255), 1)
            cv2.putText(vis, f"dx={dx} dy={dy}  servo=({servo_x},{servo_y})    State={state}",(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

            cv2.imshow("Webcam", vis)


            if AllMasks:
                cv2.imshow("laser_mask", laser_mask)
                cv2.imshow("target_mask", target_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
