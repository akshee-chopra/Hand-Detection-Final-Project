import time as t
import djitellopy as tello
import cv2
import numpy as np
import mediapipe as mp

# Initialize and connect to my_drone
my_drone = tello.Tello()
my_drone.connect()
print("battery:", my_drone.get_battery())

# Turn on video streaming
my_drone.streamon()
my_drone.takeoff()
my_drone.get_frame_read()

# mediapipe stuff - a Google library meant to help detect hands 
mpHands = mp.solutions.hands
hands = mpHands.Hands(min_tracking_confidence=0.8, min_detection_confidence=0.8)
mpDraw = mp.solutions.drawing_utils
i = 0
y6 = 0 #labeling certain coordinates that are provided from MediaPipe
y8 = 0
x12 = 0
y12 = 0
# px12 = 0
# py12 = 0
dcx = 480  # h = 720, w = 960, c = 3
dcy = 360
fist = False #intializing fist to false originally 


# PID Gains

#these are PID values that I tested with my drone specifically The P stands for Proportional, the I stands for Integral, and the D stands for Derivative. This is the basic structure that makes up PID. The R stands for role, which is an axis of the drone. I wanted to control the movements along this axis, so I have values for it in my PID controller. I tested these values through trial-and-error to see what worked best for my drone specifically. 


RP = .05
RI = 0.0
RD = .05


# Define Constants
SCREEN_WIDTH = 960  # org diff
SCREEN_HEIGHT = 720  # org diff
TARGET_SIZE = 16000
MAX_EFFORT = 100
MAX_PITCH_EFFORT = 30
DT = 1

# Initialize some variables for later use
#Here, I kept track of error because error is how a PID controller actually tells whether it is moving the drone in the proper direction or not. The point of the PID controller is to minimize error for roll. 

prev_roll_error = 0
roll_integral = 0
autonomous_control = False
then = t.time()
time = []
roll_error_arr = []

# Display live fed from front facing camera
while i < 75:
    print("iter:", i)
    # Get current time
    now = t.time()

    # Get and resize image from my_drone
    img = my_drone.get_frame_read().frame
    img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT))

    cv2.imshow("Image", img)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    #marking the center of the drone's webcam 
    cv2.circle(img, (dcx, dcy), 5, (0, 0, 255), 2)
    cv2.putText(img, "CEN", (dcx, dcy), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)

    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            for id, lm in enumerate(handLms.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                if id == 6:
                    y6 = cy
                if id == 8:
                    y8 = cy
                if id == 12:
                    x12 = cx
                    y12 = cy
                if y8 > y6:
                    fist = True
                    #based on the points provided by MediaPipe, if a certain placement was happening in the hand coordinates, then the hand was assumed to be in a fist. 
                else:
                    fist = False
            cv2.circle(img, (x12, y12), 2, (255, 0, 255), cv2.FILLED)
            cv2.putText(img, "12", (x12, y12), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)


    cv2.imshow("Image", img)
    cv2.waitKey(1)

    # Definitions of error for each possible effort
    roll_error = dcx - x12  # left right

    # Keep track of time and error for plotting
    time.append(now - then)
    roll_error_arr.append(roll_error)

    # Iterate integral term
    roll_integral += roll_error * DT


    # Calculate difference in error since last time interval
    roll_derivative = (roll_error - prev_roll_error) / DT


    # PID to update effort values
    roll_effort = int((RP * roll_error) + (RI * roll_integral) - (RD * roll_derivative))

    roll_effort = int(np.clip(roll_effort, -MAX_EFFORT, MAX_EFFORT))


    # Send effort values to my_drone
    if fist and roll_effort != 24 and roll_effort <= 15 and roll_error != 0:
        print("MOVING", i)
        r = roll_effort * -1
        my_drone.send_rc_control(r, 0, 0, 0)
        t.sleep(1)
    else:
        my_drone.send_rc_control(0,0,0,0)
    # Update prev error values for next loop
    prev_roll_error = roll_error
    i = i + 1
my_drone.land()
my_drone.streamoff()
