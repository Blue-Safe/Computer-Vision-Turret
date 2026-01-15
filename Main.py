import cv2
import numpy as np
import time
import math
from enum import Enum, auto
from ServoTesting import sendData


# Tunable Variables 
NUDGE_DEG = .25
DEADBAND_PX = 5
LASER_AREA  = 5000
TARGET_AREA = 8000
LOCKED_PX = TARGET_AREA
MAX_ANGLE = 170
MIN_ANGLE = 10
MAX_ANGLE_JUMP = 30
JUMP_CONSTANT = 75
LOST_COUNT = 15
PTD_CONSTANT = 0
VEL_CAP = 60
T_LEAD = .22
LOCKED_TIME =.2
CONFIDENCE_THRESHOLD = 80



# Invert if turret tracks in the wrong direction
INVERT_X = True
INVERT_Y = False

# Turn on/off debugging tools
Debug = True
UI = True
frameRate = True
AllMasks = False


class State(Enum):
    TRACK = auto()
    IDLE = auto()
    LOCKED = auto()
    INIT = auto()



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
    # print(cv2.contourArea(c))
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
    
    # Convert color range to hsv for color tracking

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Filter for colors within the expected color range of the target.

    BL,BU = (0,0,0),(179,255,50)


    mask = cv2.inRange(hsv,BL,BU)

    

    # This is where we block out where the laser is in the frame, or atleast where it was when we last located it.
    if laser_mask is not None:
        inv_laser = cv2.bitwise_not(laser_mask)
        mask = cv2.bitwise_and(mask, inv_laser)

    # Pass off largest blob detection to helper function.

    center, contour = largest_center_from_mask(mask, TARGET_AREA-7000, TARGET_AREA+7000)
    
    return center, mask, contour

# Make sure we are sending valid angles to the pico
def clamp(val, low, high):
    return max(low, min(high, val))

# To combat parralax and over forms of distorition, rather than trying to move to exactly where the target is,
# I instead decided to just figure out the direction the target is in and nudge the laser towards it.
# This helper method returns -step_deg,0,or step_deg based on if we will need to add or subtract from the current servo angle.

def sign_step(error, deadband, step_deg):
    if abs(error) <= deadband:
        return 0
    if error > 0:
        return step_deg
    else:
        return -step_deg


def move_servos(error, servos, gain):
    dx, dy = error
    servo_x, servo_y = servos

    
    if abs(dx) <= DEADBAND_PX:
        X_degree = 0.0
    else:
        X_degree = (dx / PTD_CONSTANT) * gain

    if abs(dy) <= DEADBAND_PX:
        Y_degree = 0.0
    else:
        Y_degree = (dy / PTD_CONSTANT) * gain

    # Clamp jump
    X_degree = clamp(X_degree, -MAX_ANGLE_JUMP, MAX_ANGLE_JUMP)
    Y_degree = clamp(Y_degree, -MAX_ANGLE_JUMP, MAX_ANGLE_JUMP)

    servo_x = clamp(servo_x + X_degree, 0, 180)
    servo_y = clamp(servo_y + Y_degree, 0, 180)

    sendData(servo_x, servo_y, 1)
    return servo_x, servo_y


def main():

    # Initializing code. Starts up required variables and locates laser home position.

    global PTD_CONSTANT
    global MAX_ANGLE_JUMP
    global homePos
    global last_target
    global last_laser
    lock_in_time = 0
    last_target = None
    homePos = 0,0
    target_center = 0,0
    tx,ty = target_center
    state = State.INIT

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("Could not open Camera")
        return

    lastfps = 0
    prev_t = time.time()
    accuracy = 0.0
    t_lock = 0.0
    time_confidence = 0.0
    confidence = 0.0
    tau = 0.7


    while True:

        
        
        now = time.time()
        dt = now - prev_t
        prev_t = now
        dt = max(.01, min(dt, 0.1))

        target_center = None
        target_contour = None
        target_mask = None

        laser_center = None
        laser_contour = None
        laser_mask = None

        lx,ly = 0,0

        

        

        
            


        if state is not State.INIT:

            # Read in new frame each loop

            cap.grab()
            ret, frame = cap.retrieve()

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

            if target_center != (0,0) and target_center is not None:
                tx,ty = target_center
            else:
                target_center = homePos
                tx, ty = target_center
            # calculate error from laser to target
            dx = tx - lx
            dy = ty - ly


        


        if state is State.INIT:
            # In the setup state, we do the following:
            # Store Homeposition
            # Small calibration to determine rough pixel to degree constant
            # Go to idle State
            # Get home position for the laser

            servo_x,servo_y = 80,90
            sendData(servo_x,servo_y,1)
            time.sleep(1)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            leftPos = laser_center

            servo_x, servo_y = 90, 90
            sendData(servo_x, servo_y, 1)
            time.sleep(1)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            homePos = laser_center

            

            

            servo_x = 100
            sendData(servo_x,servo_y,1)
            time.sleep(1)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            RightPos = laser_center
            
            

            


            L_pixel_delta = leftPos[0] - homePos[0]
            R_pixel_delta = RightPos[0] - homePos[0]


            
            

            PTD_CONSTANT = ((abs(L_pixel_delta) + abs(R_pixel_delta)) / 2) / 10

            print(PTD_CONSTANT)

            servo_x = 90
            sendData(servo_x,servo_y,1)
            time.sleep(1)

            
        
            state = State.IDLE
            MAX_ANGLE_JUMP = 3
            
        elif state is State.IDLE:
            # In the Idle state, we do the following:
            # Start drifting home (rather than immediatly incase it comes back in frame).
            # Wait for a target to come into frame.
            # React to target by switching States.

            gain = .1
            lost_counter = 0

            target_center = homePos

            (servo_x,servo_y) = move_servos((dx,dy),(servo_x,servo_y),gain)

            if target_contour is not None:
                state = State.TRACK
                lost_counter = 0
                MAX_ANGLE_JUMP = 13
                last_target = target_center
                locked = False
                Lock_start = time.time()
            if servo_x > MAX_ANGLE or servo_x < MIN_ANGLE or servo_y > MAX_ANGLE or servo_y < MIN_ANGLE:
                sendData(90,90,1)
                servo_x, servo_y = 90,90
                time.sleep(.4)

        elif state is State.TRACK:
            # In the tracking state, we do the following:
            # Ensure target is still seen
            # Check if we are locked yet
            # Move towards target

            if target_contour is not None:

                lost_counter = max(0,lost_counter-1)

                gain = .25 

                # Get targets Velocity
                if last_target is not None:
                    vx = (target_center[0] - last_target[0]) / dt
                    vy = (target_center[1] - last_target[1]) / dt
                    velocity = (vx, vy)
                    if abs(velocity[0]) < VEL_CAP:
                        velocity = (0, velocity[1])
                    if abs(velocity[1]) < VEL_CAP:
                        velocity = (velocity[0], 0)

                    ptx = tx + vx * T_LEAD
                    pty = ty + vy * T_LEAD
                    
                    # calculate error from laser to target
                    dx = ptx - lx 
                    dy = pty - ly 

                    

                else:
                    vx,vy = 0,0
                    velocity = vx,vy


                if dx == 0:
                    dx = .0001
                if dy == 0:
                    dy = .0001
                
                err = math.hypot(dx, dy)

                # Accuracy score 0-1
                accuracy = clamp(1.0 - (err / LOCKED_PX), 0.0, 1.0)

                # Time-on-target score 0-1
                if err < LOCKED_PX:
                    t_lock += dt
                else:
                    t_lock = 0.0

                if t_lock >= LOCKED_TIME and not locked and confidence >= CONFIDENCE_THRESHOLD:
                    lock_end = time.time()
                    locked = True
                    lock_in_time = lock_end-Lock_start
                time_confidence = 1.0 - math.exp(-t_lock / tau)

                confidence = 100.0 * accuracy * time_confidence
                confidence = 0.8*confidence + 0.2*confidence

                if math.hypot(dx,dy) > DEADBAND_PX:
                    
                    # If Servo moves in the wrong direction flip axis's
                    if INVERT_X:
                        dx = -dx
                        vx = -vx
                    if INVERT_Y:
                        dy = -dy
                        vy = -vy
                    (servo_x,servo_y) = move_servos((dx,dy),(servo_x,servo_y),gain)
                


            else:
                lost_counter += 1
                if lost_counter >= LOST_COUNT:
                    state = State.IDLE
                    accuracy = 0.0
                    t_lock = 0.0
                    time_confidence = 0.0
                    confidence = 0.0

            if servo_x > MAX_ANGLE or servo_x < MIN_ANGLE or servo_y > MAX_ANGLE or servo_y < MIN_ANGLE:
                print("LIMIT RESET TRIGGERED")
                sendData(90,90,1)
                servo_x, servo_y = 90,90
                time.sleep(.4)
            accuracy = 1/math.hypot(dx,dy) 
        
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
            cv2.putText(vis, f"servo=({round(servo_x)},{round(servo_y)})    State={state}   "     ,(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis, f"FPS={round(lastfps)}    Confidence={round(confidence,2)} Lock in time={round(lock_in_time,2)}"     ,(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            if not ret or vis is None or vis.size == 0:
                continue
            else:
                cv2.imshow("Webcam", vis)


                if AllMasks:
                    if target_mask is not None and target_mask.size > 0 and laser_mask is not None and laser_mask.size > 0:
                        cv2.imshow("target_mask", target_mask)
                        cv2.imshow("laser_mask", laser_mask)

            
        lastfps = 1.0 / dt
        last_target = target_center
        


        

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
