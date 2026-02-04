import cv2
import numpy as np
import time
import math
import csv
from pathlib import Path
from enum import Enum, auto
from dataclasses import dataclass
from ServoTesting import sendData

# Tunable Variables 
DEADBAND_PX = 7
LASER_AREA  = 5000
TARGET_AREA = 8000
LOCKED_PX = math.sqrt(TARGET_AREA / math.pi) / 2
MAX_ANGLE = 170
MIN_ANGLE = 10
MAX_ANGLE_JUMP = 30
LOST_COUNT = 15
PTD_CONSTANT = 0
VEL_THRESHOLD = 30
T_LEAD = .22
LOCKED_TIME =.5
CONFIDENCE_THRESHOLD = 80
TAU = 0.7
WIDTH = 640
HEIGHT = 480
DIVISIONS = 10

# Invert if turret tracks in the wrong direction
INVERT_X = True
INVERT_Y = False

# Turn on/off debugging tools
Debug = False
UI = True
frameRate = True
AllMasks = False

# Establish States

class State(Enum):
    TRACK = auto()
    IDLE = auto()
    INIT = auto()
    TESTONE = auto()
    TESTTWO = auto()


# Define Turret object. This organizes the variables I edit in many
# spots, without having the headache of globals

@dataclass
class Turret:
    state: State = State.INIT
    servo_x: float = 90.0
    servo_y: float = 90.0
    home_pos: tuple[int,int] = (0,0)
    target_pos: tuple[int,int] = (0,0)
    last_target: tuple[int,int] = (0,0)
    laser_pos: tuple[int,int] = (0,0)

    lost_counter: int = 0
    locked: bool = False
    lock_start: float = 0.0
    lock_in_time: float = 0.0
    t_lock: float = 0.0
    confidence: float = 0.0
    accuracy: float = 0.0

class CSVLogger:

    def __init__(self):
        self.f = None
        self.w = None
        self.t0 = None

    def start(self, filename: str):
        Path("logs").mkdir(exist_ok=True)
        self.f = open(Path("logs") / filename, "w", newline="")
        self.w = csv.writer(self.f)
        self.t0 = time.perf_counter()

        self.w.writerow([
            "t", "dt", "fps", "state",
            "laser_x", "laser_y",
            "target_x", "target_y",
            "dx", "dy", "err_px",
            "vx", "vy", "velocity",
            "servo_x", "servo_y",
            "locked", "t_lock",
            "lost_counter"
        ])



    def log_frame(self, t: Turret, dt, dx, dy, err_px, vx=0,vy=0):
        if self.w is None:
            return

        rel_t = time.perf_counter() - self.t0

        self.w.writerow([
            round(rel_t, 4), round(dt, 4), round(1.0/dt), t.state.name,
            int(t.laser_pos[0]), round(1.0/dt), int(t.laser_pos[1]),
            int(t.target_pos[0]), int(t.target_pos[1]),
            round(dx, 3), round(dy, 3),
            round(vx,3), round(vy,3),round(math.hypot(vx+.00001,vy+.00001)),
            round(err_px, 3),round(t.servo_x, 3), round(t.servo_y, 3),
            int(t.locked), round(t.t_lock, 4)
        ])

    def stop(self):
        if self.f:
            self.f.close()
        self.f = None
        self.w = None
        self.t0 = None

    @property
    def enabled(self):
        return self.w is not None

class Dummytarget:

    pos: tuple[int,int] = (0,0)
    velocity: tuple[float,float] = (0.0,0.0)
    area: float = TARGET_AREA 

    def __init__(self,pos,velocity):
        self.pos = pos
        self.velocity = velocity

    def move(self,dt):
        # Determine next position based off of the current velocity and time passed.

        x,y = self.pos
        dx,dy = self.velocity[0]*dt,self.velocity[1]*dt
        self.pos = x+dx,y+dy

def save_trials_csv(time_trials: dict, filename: str):
    Path("logs").mkdir(exist_ok=True)
    with open(Path("logs")/filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial_id","goal_x","goal_y","lock_in_time_s"])
        for i, ((x,y), tlock) in enumerate(time_trials.items()):
            w.writerow([i, x, y, tlock])


def largest_center_from_mask(mask, min_area,max_area):
    # Helper function that finds all contours within the frame, then orgainzes them
    # and returns the position and contour object that is the largest within the range 
    # Passed through the parameters

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = []
    for c in contours:
        if cv2.contourArea(c) >= min_area and cv2.contourArea(c) <= max_area:
            valid.append(c)
    
    if not valid:
        return None, None
    
    c = max(valid, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    cx, cy = x + w // 2, y + h // 2
    if Debug:
        print(f"Biggest Blob size: {cv2.contourArea(c)}")
    return (cx, cy), c

def detect_laser(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Laser appears bright red and white hot to the webcam. I use three ranges, as red wraps around the scale and
    # If we don't account for the white center, it'll be a donut shape rather than our whole laser.
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
    # Range is geneours because the webcame is noisy.
    center, contour = largest_center_from_mask(mask, LASER_AREA-5000,LASER_AREA+5000)

    # Return the center cord, the masked frame incase we want to display it, and the actual object itself. 
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

    center, contour = largest_center_from_mask(mask, TARGET_AREA-3000, TARGET_AREA+3000)
    
    return center, mask, contour

def clamp(val, low, high):
    return max(low, min(high, val))

def move_servos(t:Turret,dx,dy,gain):
    # Moves the servos based on their current position, the error, and gain.

    # If Servo moves in the wrong direction flip axis's
    if INVERT_X:
        dx = -dx
    if INVERT_Y:
        dy = -dy

    # pixels to degrees
    X_degree = (dx / PTD_CONSTANT) * gain
    Y_degree = (dy / PTD_CONSTANT) * gain

    # Clamp angle change and new servo position
    X_degree = clamp(X_degree, -MAX_ANGLE_JUMP, MAX_ANGLE_JUMP)
    Y_degree = clamp(Y_degree, -MAX_ANGLE_JUMP, MAX_ANGLE_JUMP)

    t.servo_x = clamp(t.servo_x + X_degree, 0, 180)
    t.servo_y = clamp(t.servo_y + Y_degree, 0, 180)

    sendData(t.servo_x, t.servo_y, 1)

def demo_track(test):

    # Initializing code. Starts up required variables and locates laser home position.

    global PTD_CONSTANT
    global MAX_ANGLE_JUMP
    turret = Turret()
    logger = CSVLogger()
    
    # Grab webcam data, define resolution, and only let 1 frame sit in buffer
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("Could not open Camera")
        return

    lastfps = 0
    prev_t = time.perf_counter()
    
    while True:
        
        target_contour = None
        laser_contour = None
        target_mask = None
        laser_mask = None
        target_center = None
        laser_center = None
        ret = False
        frame = None

        now = time.perf_counter()
        dt = now - prev_t
        prev_t = now

        # Due to my lower framerate,if there is a significant stutter dt would become larger
        # Then what we would want it to be. Clamp to a reasoble time
        dt = clamp(dt,.01,.1) 
        
        if turret.state is not State.INIT:

            # Read in new frame each loop

            cap.grab()
            ret, frame = cap.retrieve()

            if not ret:
                break

            # Detect Laser and Target.

            laser_center, laser_mask, laser_contour = detect_laser(frame)
            target_center, target_mask, target_contour = detect_target(frame, laser_mask=laser_mask)
            
            # Store current data. If no laser or target is picked up, assume last position.
             

            if laser_center != None:
                turret.laser_pos = laser_center

            if target_center != None:
                turret.target_pos = target_center

            # Calculate error

            if laser_center != None and target_center != None:
                dx = target_center[0] - laser_center[0]
                dy = target_center[1] - laser_center[1]
            else:
                dx,dy = 0,0

            err_px = math.hypot(dx, dy)
            
                
        if turret.state is State.INIT:
            # In the setup state, we find the PTD_CONSTANT,

           
            sendData(80,90,1)
            time.sleep(.5)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            leftPos = laser_center

            sendData(90,90, 1)
            time.sleep(.5)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            homePos = laser_center
            turret.home_pos = homePos

            sendData(100,90,1)
            time.sleep(.5)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            RightPos = laser_center
          
            L_pixel_delta = leftPos[0] - homePos[0]
            R_pixel_delta = RightPos[0] - homePos[0]

            PTD_CONSTANT = ((abs(L_pixel_delta) + abs(R_pixel_delta)) / 2) / 10

            if Debug:
                print(f"PTD_CONSTANT: {PTD_CONSTANT}")

            
            sendData(90,90,1)
            time.sleep(.5)
            
            # Switch states
            turret.state = State.IDLE
            MAX_ANGLE_JUMP = 3
            gain = .1
        
        elif turret.state is State.IDLE:
            # In the Idle state, we first check if we see a target or not. 
            # If we don't, we track the home position slowly, in case the target comes back on frame.
            # Then we ensure we do a check to ensure we are still within the expected range.

            gain = .1
            
            if target_contour is not None:
                turret.state = State.TRACK
                turret.lost_counter = 0
                MAX_ANGLE_JUMP = 13
                turret.last_target = target_center
                turret.locked = False
                turret.t_lock = 0.0
                turret.confidence = 0.0
                turret.accuracy = 0.0
                turret.lock_in_time = 0.0
                turret.lock_start = time.perf_counter()
            else:
                turret.target_pos = turret.home_pos
                dx = turret.target_pos[0] - turret.laser_pos[0]
                dy = turret.target_pos[1] - turret.laser_pos[1]

                move_servos(turret,dx,dy,gain)

                if turret.servo_x > MAX_ANGLE or turret.servo_x < MIN_ANGLE or turret.servo_y > MAX_ANGLE or turret.servo_y < MIN_ANGLE:
                    sendData(90,90,1)
                    turret.servo_x, turret.servo_y = 90,90
                    time.sleep(.4)
        
        elif turret.state is State.TESTTWO:
            # In the tracking state,

            gain = .25

            if target_contour is not None:
                
                # when we see the target, start decreasing the counter till we hit zero.
                turret.lost_counter = max(0,turret.lost_counter-1)

                # Get targets Velocity
                last_target = turret.last_target
                vx = (target_center[0] - last_target[0]) / dt
                vy = (target_center[1] - last_target[1]) / dt
                
                if abs(vx) < VEL_THRESHOLD:
                    vx = 0
                if abs(vy) < VEL_THRESHOLD:
                    vy = 0

                ptx = target_center[0] + vx * T_LEAD
                pty = target_center[1] + vy * T_LEAD
                
                # calculate error from laser to target. Add .001 so it is unlikely to be zero
                dx = ptx - turret.laser_pos[0]+.001 
                dy = pty - turret.laser_pos[1]+.001
                
                err = math.hypot(dx, dy)

                # Accuracy score 0-1
                turret.accuracy = clamp(1.0 - (err / LOCKED_PX), 0.0, 1.0)

                # Time-on-target score 0-1
                if err < LOCKED_PX:
                    turret.t_lock += dt
                else:
                    turret.t_lock = 0.0

                if turret.t_lock >= LOCKED_TIME and not turret.locked and turret.confidence >= CONFIDENCE_THRESHOLD:
                    lock_end = time.perf_counter()
                    turret.locked = True
                    turret.lock_in_time = lock_end-turret.lock_start
                time_confidence = 1.0 - math.exp(-turret.t_lock / TAU)

                turret.confidence = 100.0 * turret.accuracy * time_confidence
                
                if err > DEADBAND_PX:
                    
                    move_servos(turret,dx,dy,gain)

            else:
                turret.lost_counter += 1
                if turret.lost_counter >= LOST_COUNT:
                    turret.state = State.IDLE
                    turret.accuracy = 0.0
                    turret.t_lock = 0.0
                    time_confidence = 0.0
                    turret.confidence = 0.0

            if turret.servo_x > MAX_ANGLE or turret.servo_x < MIN_ANGLE or turret.servo_y > MAX_ANGLE or turret.servo_y < MIN_ANGLE:
                print("LIMIT RESET TRIGGERED")
                sendData(90,90,1)
                turret.servo_x, turret.servo_y = 90,90
                time.sleep(.4)
             
        if UI and frame is not None:

            vis = frame.copy()

            # Draw detections onto the frame for debugging
            if laser_contour is not None:
                cv2.drawContours(vis, [laser_contour], -1, (0, 255, 255), 2)
            if target_contour is not None:
                cv2.drawContours(vis, [target_contour], -1, (0, 255, 0), 2)

            # UI
            cv2.circle(vis, (turret.laser_pos), 5, (0, 255, 255), -1)
            cv2.circle(vis, (turret.target_pos), 5, (0, 255, 0), -1)
            cv2.line(vis, (turret.laser_pos), (turret.target_pos), (255, 255, 255), 1)
            cv2.putText(vis, f"servo=({round(turret.servo_x)},{round(turret.servo_y)})    State={turret.state}   "     ,(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(vis, f"FPS={round(lastfps)}    Confidence={round(turret.confidence,2)} Lock in time={round(turret.lock_in_time,2)}"     ,(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            if not ret or vis is None or vis.size == 0:
                continue
            else:
                cv2.imshow("Webcam", vis)


                if AllMasks:
                    if target_mask is not None and target_mask.size > 0 and laser_mask is not None and laser_mask.size > 0:
                        cv2.imshow("target_mask", target_mask)
                        cv2.imshow("laser_mask", laser_mask)

            
        lastfps = 1.0 / dt
        if target_center is not None:
            turret.last_target = target_center

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord('l') and not logger.enabled:
            logger.start(f"run_{int(time.perf_counter())}.csv")
            print("LOGGING ON")

        if key == ord('k') and logger.enabled:
            logger.stop()
            print("LOGGING OFF")

    cap.release()
    logger.stop()
    cv2.destroyAllWindows()

def test_track(test):

    # Initializing code. Starts up required variables and locates laser home position.

    global PTD_CONSTANT
    global MAX_ANGLE_JUMP
    turret = Turret()
    logger = CSVLogger()
    
    # Grab webcam data, define resolution, and only let 1 frame sit in buffer
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("Could not open Camera")
        return

    lastfps = 0
    prev_t = time.perf_counter()
    
    while True:
        
        target_contour = None
        laser_contour = None
        target_mask = None
        laser_mask = None
        target_center = None
        laser_center = None
        ret = False
        frame = None

        now = time.perf_counter()
        dt = now - prev_t
        prev_t = now

        # Due to my lower framerate,if there is a significant stutter dt would become larger
        # Then what we would want it to be. Clamp to a reasoble time
        dt = clamp(dt,.01,.1) 
        
        if turret.state is not State.INIT:

            if test == 1:
                # Read in new frame each loop

                cap.grab()
                ret, frame = cap.retrieve()

                if not ret:
                    break

                # Detect Laser and Target.

                laser_center, laser_mask, laser_contour = detect_laser(frame)
                
                # Store current data. If no laser or target is picked up, assume last position.
                

                if laser_center != None:
                    turret.laser_pos = laser_center

            elif test == 2:
                # Read in new frame each loop

                cap.grab()
                ret, frame = cap.retrieve()

                if not ret:
                    break

                # Detect Laser and Target.

                laser_center, laser_mask, laser_contour = detect_laser(frame)
                target_center, target_mask, target_contour = detect_target(frame, laser_mask=laser_mask)
                
                # Store current data. If no laser or target is picked up, assume last position.
                

                if laser_center != None:
                    turret.laser_pos = laser_center

                if target_center != None:
                    turret.target_pos = target_center

                # Calculate error

                if laser_center != None and target_center != None:
                    dx = target_center[0] - laser_center[0]
                    dy = target_center[1] - laser_center[1]
                else:
                    dx,dy = 0,0

                err_px = math.hypot(dx, dy)

                
        if turret.state is State.INIT:
            # In the setup state, we find the PTD_CONSTANT,

           
            sendData(80,90,1)
            time.sleep(.5)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            leftPos = laser_center

            sendData(90,90, 1)
            time.sleep(.5)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            homePos = laser_center
            turret.home_pos = homePos

            sendData(100,90,1)
            time.sleep(.5)

            ret, frame = cap.read()
            laser_center,laser_mask, laser_contour = detect_laser(frame)
            RightPos = laser_center
          
            L_pixel_delta = leftPos[0] - homePos[0]
            R_pixel_delta = RightPos[0] - homePos[0]

            PTD_CONSTANT = ((abs(L_pixel_delta) + abs(R_pixel_delta)) / 2) / 10

            if Debug:
                print(f"PTD_CONSTANT: {PTD_CONSTANT}")

            
            sendData(90,90,1)
            time.sleep(.5)

            # Make list of test points

            test_points = []
            time_trials = {}

            x_step = WIDTH/DIVISIONS
            y_step = HEIGHT/DIVISIONS

            for i in range(DIVISIONS):
                for j in range(DIVISIONS):
                    x_cord = x_step*i 
                    y_cord = y_step*j 
                    if x_cord == 0 or x_cord >= WIDTH:
                        continue
                    if y_cord == 0 or y_cord >= HEIGHT:
                        continue
                    test_points.append((x_cord,y_cord))
            
            # Switch states
            prev_state = turret.state
            if test == 1:
                turret.state = State.TESTONE
            elif test == 2:
                turret.state = State.TESTTWO
            else:
                break
            
            MAX_ANGLE_JUMP = 4
            gain = .1
        
        elif turret.state is State.IDLE:
            # In the Idle state, we first check if we see a target or not. 
            # If we don't, we track the home position slowly, in case the target comes back on frame.
            # Then we ensure we do a check to ensure we are still within the expected range.

            gain = .1
            
            if target_contour is not None:
                turret.state = State.TESTTWO
                turret.lost_counter = 0
                MAX_ANGLE_JUMP = 3
                turret.last_target = target_center
                turret.locked = False
                turret.t_lock = 0.0
                turret.confidence = 0.0
                turret.accuracy = 0.0
                turret.lock_in_time = 0.0
                turret.lock_start = time.perf_counter()
            else:
                turret.target_pos = turret.home_pos
                dx = turret.target_pos[0] - turret.laser_pos[0]
                dy = turret.target_pos[1] - turret.laser_pos[1]

                move_servos(turret,dx,dy,gain)

                if turret.servo_x > MAX_ANGLE or turret.servo_x < MIN_ANGLE or turret.servo_y > MAX_ANGLE or turret.servo_y < MIN_ANGLE:
                    sendData(90,90,1)
                    turret.servo_x, turret.servo_y = 90,90
                    time.sleep(.4)
        
        elif turret.state is State.TESTTWO:

            if prev_state == State.INIT:
                prev_state = turret.state
                logger.start(f"run_{int(time.perf_counter())}.csv")

            gain = .25

            if target_contour is not None:
                
                # when we see the target, start decreasing the counter till we hit zero.
                turret.lost_counter = max(0,turret.lost_counter-1)

                # Get targets Velocity
                last_target = turret.last_target
                vx = (target_center[0] - last_target[0]) / dt
                vy = (target_center[1] - last_target[1]) / dt
                
                if abs(vx) < VEL_THRESHOLD:
                    vx = 0
                if abs(vy) < VEL_THRESHOLD:
                    vy = 0

                ptx = target_center[0] + vx * T_LEAD
                pty = target_center[1] + vy * T_LEAD
                
                # calculate error from laser to target. Add .001 so it is unlikely to be zero
                dx = ptx - turret.laser_pos[0]+.001 
                dy = pty - turret.laser_pos[1]+.001
                
                err = math.hypot(dx, dy)

                # Accuracy score 0-1
                turret.accuracy = clamp(1.0 - (err / LOCKED_PX), 0.0, 1.0)

                # update t_lock
                if err < LOCKED_PX:
                    turret.t_lock += dt
                else:
                    turret.t_lock = 0.0

                # update confidence before lock check
                turret.accuracy = clamp(1.0 - (err / LOCKED_PX), 0.0, 1.0)
                time_confidence = 1.0 - math.exp(-turret.t_lock / TAU)
                turret.confidence = 100.0 * turret.accuracy * time_confidence

                # lock decision uses current confidence
                if (turret.t_lock >= LOCKED_TIME
                    and not turret.locked
                    and turret.confidence >= CONFIDENCE_THRESHOLD):
                    turret.locked = True
                    turret.lock_in_time = time.perf_counter() - turret.lock_start
                
                if err > DEADBAND_PX:
                    
                    move_servos(turret,dx,dy,gain)

            else:
                turret.lost_counter += 1
                if turret.lost_counter >= LOST_COUNT:
                    turret.state = State.IDLE
                    turret.accuracy = 0.0
                    turret.t_lock = 0.0
                    time_confidence = 0.0
                    turret.confidence = 0.0
            
            logger.log_frame(turret,dt,dx,dy,err,vx,vy)

            if turret.servo_x > MAX_ANGLE or turret.servo_x < MIN_ANGLE or turret.servo_y > MAX_ANGLE or turret.servo_y < MIN_ANGLE:
                print("LIMIT RESET TRIGGERED")
                sendData(90,90,1)
                turret.servo_x, turret.servo_y = 90,90
                time.sleep(.4)
        
        elif turret.state is State.TESTONE:

            # Run a series of test to determine LOCK IN TIME. Run through a list of coordinates based 
            # off of screen resolution, then average results.

            # If this is the first trial, test_trials will be empty. Generate target object and start process
            if prev_state == State.INIT:
                prev_state = turret.state
                test_number = 0
                target = Dummytarget(test_points[test_number],(0.0,0.0))
                turret.lock_start = time.perf_counter()
                logger.start(f"run_{int(time.perf_counter())}.csv")
                print("LOGGING ON")

            # If the turret is locked, reset it and move onto the next target.
            if turret.locked:
                turret.locked = False
                if turret.lock_in_time > LOCKED_TIME:
                    time_trials[test_points[test_number]] = turret.lock_in_time
                turret.lock_in_time = 0
                sendData(90, 90, 1)
                turret.servo_x, turret.servo_y = 90, 90
                time.sleep(.5)
                turret.t_lock = 0
                turret.lock_start = time.perf_counter()

                if test_number + 1 < len(test_points):
                    test_number += 1
                    target.pos = test_points[test_number]
                    turret.target_pos = target.pos
                    
                else:
                    logger.stop()
                    save_trials_csv(time_trials, f"trials_{int(time.perf_counter())}.csv")
                    print("LOGGING OFF")
                    break
            
            gain = .25
                
            # Generate data on artifical target

            target_center = target.pos
            print(target.pos)

            # Calculate error

            if laser_center != None and target_center != None:
                dx = target_center[0] - laser_center[0]
                dy = target_center[1] - laser_center[1]
            else:
                dx,dy = 0,0

            err_px = math.hypot(dx, dy)
            

            # Get targets Velocity
            last_target = turret.last_target
            vx = (target_center[0] - last_target[0]) / dt
            vy = (target_center[1] - last_target[1]) / dt
            
            if abs(vx) < VEL_THRESHOLD:
                vx = 0
            if abs(vy) < VEL_THRESHOLD:
                vy = 0

            ptx = target_center[0] + vx * T_LEAD
            pty = target_center[1] + vy * T_LEAD
            
            # calculate error from laser to target. Add .001 so it is unlikely to be zero
            dx = ptx - turret.laser_pos[0]+.001 
            dy = pty - turret.laser_pos[1]+.001
            
            err = math.hypot(dx, dy)

            # Time-on-target score 0-1
            if err < LOCKED_PX:
                turret.t_lock += dt
            else:
                turret.t_lock = 0.0

            if turret.t_lock >= LOCKED_TIME and not turret.locked:
                lock_end = time.perf_counter()
                turret.locked = True
                turret.lock_in_time = lock_end-turret.lock_start

            
            if err > DEADBAND_PX:
                
                move_servos(turret,dx,dy,gain)

            if turret.servo_x > MAX_ANGLE or turret.servo_x < MIN_ANGLE or turret.servo_y > MAX_ANGLE or turret.servo_y < MIN_ANGLE:
                print("LIMIT RESET TRIGGERED")
                sendData(90,90,1)
                turret.servo_x, turret.servo_y = 90,90
                time.sleep(.4)
            
            target.move(dt)
            if turret.lock_in_time < LOCKED_TIME:
                pass
            else:
                logger.log_frame(turret, dt, dx, dy, err_px)
        
        if test == 1:
            if UI and frame is not None:

                vis = frame.copy()

                # Draw detections onto the frame for debugging
                if laser_contour is not None:
                    cv2.drawContours(vis, [laser_contour], -1, (0, 255, 255), 2)
                if target_contour is not None:
                    cv2.drawContours(vis, [target_contour], -1, (0, 255, 0), 2)

                # UI
                cv2.circle(vis, (turret.laser_pos), 5, (0, 255, 255), -1)
                # cv2.circle(vis, (turret.target_pos), 5, (0, 255, 0), -1)
                # cv2.line(vis, (turret.laser_pos), (turret.target_pos), (255, 255, 255), 1)
                cv2.putText(vis, f"servo=({round(turret.servo_x)},{round(turret.servo_y)})    Target Position= {target_center}"     ,(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(vis, f"FPS={round(lastfps)}   Lock in time={round(turret.lock_in_time,2)}"     ,(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                if not ret or vis is None or vis.size == 0:
                    continue
                else:
                    cv2.imshow("Webcam", vis)


                    if AllMasks:
                        if target_mask is not None and target_mask.size > 0 and laser_mask is not None and laser_mask.size > 0:
                            cv2.imshow("target_mask", target_mask)
                            cv2.imshow("laser_mask", laser_mask)
        elif test == 2:
            if UI and frame is not None:

                vis = frame.copy()

                # Draw detections onto the frame for debugging
                if laser_contour is not None:
                    cv2.drawContours(vis, [laser_contour], -1, (0, 255, 255), 2)
                if target_contour is not None:
                    cv2.drawContours(vis, [target_contour], -1, (0, 255, 0), 2)

                # UI
                cv2.circle(vis, (turret.laser_pos), 5, (0, 255, 255), -1)
                cv2.circle(vis, (turret.target_pos), 5, (0, 255, 0), -1)
                cv2.line(vis, (turret.laser_pos), (turret.target_pos), (255, 255, 255), 1)
                cv2.putText(vis, f"servo=({round(turret.servo_x)},{round(turret.servo_y)})      TargetVelocity={math.hypot((turret.target_pos[0]-turret.last_target[0])/dt,(turret.target_pos[1]-turret.last_target[1])/dt)}"     ,(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(vis, f"FPS={round(lastfps)}"     ,(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                if not ret or vis is None or vis.size == 0:
                    continue
                else:
                    cv2.imshow("Webcam", vis)


                    if AllMasks:
                        if target_mask is not None and target_mask.size > 0 and laser_mask is not None and laser_mask.size > 0:
                            cv2.imshow("target_mask", target_mask)
                            cv2.imshow("laser_mask", laser_mask)


        lastfps = 1.0 / dt
        if target_center is not None:
            turret.last_target = target_center
        
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord('l') and not logger.enabled:
            logger.start(f"run_{int(time.perf_counter())}.csv")
            print("LOGGING ON")

        if key == ord('k') and logger.enabled:
            logger.stop()
            print("LOGGING OFF")
            if test == 2:
                break


    cap.release()
    logger.stop()
    cv2.destroyAllWindows()

def main():

    test_track(2)

if __name__ == "__main__":
    main()