#The goal of this project is to use computer vision to take control of a 
#   drone in flight and autonomously land and charge it

#This python program connects to an Arduino which handles the communication to the drone
#Please see the gitHub documentation for further information

#Credit:
#Rotation matrix to euler angles math: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#Inspiration and PID loop framework from @partomatl: https://github.com/partomatl
#Signal filtering from @partomatl


#Imports
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import serial
from scipy.signal import butter, lfilter
import keyboard


#Position Targets
xTarget = 0
yTarget = 0
zTarget = 50.0 #Note the zTarget is written over when the drone is first detected
angleTarget = 0

#Define a zone that the drone can be detected in
XYtransitionBoundry = 10
ZtransitionMax = 80
ZtransitionMin = 35

#Height where the prop wash of the drone causes instability and the PID
#   error loops falls apart. At this height the drone drops with dropThrottle
groundEffectHeight = 20
dropThrottle = 1575

#Keeps track of frames the drone is detected in
detectionCount = 0
detectionThreshold = 8
detectionLostThreshold = 70

#These are landing phases
phase0Timer = 8 #Hover
phase2Timer = 0.3 #Drop through prop wash zone
phase3Timer = 1 #Cut throttle and wait
phase4Timer = 3 #Slide backwards to dock
phase5Timer = 2 #Stop throttle

#Offset based on how the tag is placed on the drone
angleOffset = 315

#Velocity of decent
velocity = 0.15

#Define Aruco tag
id_to_find  = 42
marker_size  = 3 #This is in cm

#USB port for the Arduino serial
usb_port = 'COM15'

# define middle PPM values that make the drone hover
throttle_middle = 1690  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# motors off PPM value for throttle
throttle_off = 1000

#Set throttle to safe value while things settle
throttle = throttle_off

#Control channel selects computer or PPM control (from rc reciever on arduino pin 2)
#   1000 for user control (PPM) or 2000 for computer control 
control = 1000


#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

#----------------------------------------------------------------------------------


# Calculates the shortest error "distance" between the actual and target angle
def getAngleError(actual, target, oldError):
    
    difference = target - actual
    if (difference > 180):
        actual += 360
    if (difference < -180):
        actual -= 360
    difference = target - actual

    return difference

#Checks if the drone is inside the transition boundry so computer can take control from the user safely
def ifInTransitionBoundry():
    return (zDrone > ZtransitionMin and zDrone < ZtransitionMax and yDrone > -XYtransitionBoundry
            and yDrone < XYtransitionBoundry and xDrone > -XYtransitionBoundry and xDrone < XYtransitionBoundry)


# record values for filtering
timeRecord = []
xRecord = []
yRecord = []
zRecord = []
angleRecord = []

# define a clamping function
def clamp(n, minimum, maximum):
    return max(min(maximum, n), minimum)

# define a Butterworth filter to filter the measurements
# call y = lfilter(b, a, data) to filter the data signal
order = 2  # second order filter
fs = 60  # sampling frequency is around 60 Hz
nyq = 0.5 * fs
lowcut = 2  # cutoff frequency at 2 Hz
low = lowcut / nyq
b, a = butter(order, low, btype='low')

# initialize drone variables
xDrone, yDrone, zDrone, angleDrone = 0, 0, 0, 0

# initialize PID controller
xError, yError, zError, angleError = 0, 0, 0, 0
xErrorI, yErrorI, zErrorI, angleErrorI = 0, 0, 0, 0
xErrorD, yErrorD, zErrorD, angleErrorD = 0, 0, 0, 0
xError_old, yError_old, zError_old, angleError_old = 0, 0, 0, 0

# define PID gains
KPx, KPy, KPz, KPangle = 4, 4, 2, -3
KIx, KIy, KIz, KIangle = 0.02, 0.02, 0.04, 0
KDx, KDy, KDz, KDangle = 250, 250, 220, 10

# count frames without detecting the drone
framesWithoutDrone = 0

# open serial connection with Arduino
# baudrate of 115200
arduino = serial.Serial(usb_port, 115200, timeout=.01)

# wait a bit for the connection to settle
print ("Connecting Arduino")
time.sleep(2)

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_PS_EYE.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_PS_EYE.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters  = aruco.DetectorParameters_create()

#--- Capture the webcam video
cap = cv2.VideoCapture(0)

#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap.set(cv2.CAP_PROP_FPS, 120)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# wait a bit for the connection to settle
print ("Starting video capture")
time.sleep(2)

#Time values used for timing tasks
now = time.perf_counter()
last = now
phase0Start = 0
phase2Start = 0
phase3Start = 0
phase4Start = 0
phase5Start = 0

#Tracks the phase of landing
landPhase = 0

#Tracks loops so debugging values can print out every 50 frames
printcount = 0

#Sets who has control 
autoControl = 0

print ("Ready")

while True:

    #Records time of current loop
    now = time.perf_counter()

    
    #Calculates approximate refresh rate
    fps = 1/(now-last)

    last = now

    #Printcount is used to debug by printing every 50 frames
    #   if content is printed every frame, the fps drops significantly
    printcount +=1
    if (printcount > 50):
        #print (fps)
        printcount = 0
    
    # drone detected or not
    droneDetected = False

    #Read the camera frame
    ret, frame = cap.read()

    #Flip Image
    frame = cv2.flip(frame, -1)

    #Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    
    #If drone marker is detected
    if ids is not None and ids[0] == id_to_find:
        droneDetected = True
        detectionCount += 1
        framesWithoutDrone = 0
        

        #rotation and position of each marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference frame over it
        #   Uncomment these two lines to see the detection on screen
        #aruco.drawDetectedMarkers(frame, corners)
        #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
        
        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #Extract position values
        xDrone = tvec[0]
        yDrone = tvec[1]
        zDrone = tvec[2]
        angleDrone = -((math.degrees(yaw_marker) + angleOffset)%365 -180)

        #Uncomment these two lines to see the position on screen
        #str_position = "DRONE Position x=%4.0f  y=%4.0f  z=%4.0f yaw=%4.0f"%(xDrone, yDrone, zDrone, angleDrone)
        #cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    #Drone not detected so increase count of frames without detection
    else:
        framesWithoutDrone += 1

    #If user is flying and drone is detected in boundary, take over control
    if ((autoControl == 0) and (detectionCount > detectionThreshold) and ifInTransitionBoundry()):

        #Set target to current z location to smooth transition
        zTarget = zDrone
        autoControl = 1

        #Reset errors so transition is smoother
        xErrorI = 0
        yErrorI = 0
        zErrorI = 0
        angleErrorI = 0

        #tracks time in stabalize hover
        phase0Start = time.perf_counter()
        print ("Taking Control and stabilizing")  

    #If drone is in first two stages of landing and is lost, return control to user
    if ((autoControl == 1) and (landPhase == 0 or landPhase == 1) and (framesWithoutDrone >= detectionLostThreshold)):
        autoControl = 0
        detectionCount = 0
        landPhase = 0
        phase0Start = 0
        phase2Start = 0
        print ("Drone lost")

    #When stabalize timer is done, transition to landing phase 1
    if ((autoControl == 1) and ((now - phase0Start) > phase0Timer) and landPhase == 0 ):
        landPhase = 1
        print ("Decending") 
             

    #Slowly lower drone to land
    if (landPhase == 1):
        zTarget -= velocity

    #Once drone gets close to landing pad, drop throttle to get through ground effect zone
    if ((landPhase == 1) and (zDrone < groundEffectHeight)):
        landPhase = 2
        phase2Start = time.perf_counter()

    #After a short time, kill throttle to land fully (motor controls are done later)
    if ((landPhase == 2) and ((now - phase2Start) > phase2Timer)):
        landPhase = 3
        phase3Start = time.perf_counter()

    #After a short time, set drone to slide backwards to dock (motor controls are done later)
    if ((landPhase == 3) and ((now - phase3Start) > phase3Timer)):
        landPhase = 4
        phase4Start = time.perf_counter()
        
    #After a short time, kill throtte to stop docking (motor controls are done later)
    if ((landPhase == 4) and ((now - phase4Start) > phase4Timer)):
        landPhase = 5
        phase5Start = time.perf_counter()

    #Docking done, give control back to user for takeoff
    if ((landPhase == 5) and ((now - phase5Start) > phase5Timer)):
        landPhase = 6
        print ("Drone now charging")
        print ("Charge done when lights turn off")
        print ("Toggle switch SWB and apply throttle to takeoff")
        print ("Press 'r' on the keyboard to reset the landing pad")
        print ("Press 'Esc' on the keyboard to quit the program")

    # record the position and orientation
    xRecord.append(xDrone)
    yRecord.append(yDrone)
    zRecord.append(zDrone)
    angleRecord.append(angleDrone)

    # filter x
    xFiltered = lfilter(b, a, xRecord)
    xDroneFiltered = xFiltered[-1]

    # filter y
    yFiltered = lfilter(b, a, yRecord)
    yDroneFiltered = yFiltered[-1]

    # filter z
    zFiltered = lfilter(b, a, zRecord)
    zDroneFiltered = zFiltered[-1]

    # filter angle
    angleFiltered = lfilter(b, a, angleRecord)
    angleDroneFiltered = angleFiltered[-1]

    # implement a PID controller

    # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
    xError_old = xError
    yError_old = yError
    zError_old = zError
    angleError_old = angleError

    # compute errors in position (cm) and angle (degrees) (P)
    # compute errors wrt filtered measurements

    xError = -(xTarget-xDroneFiltered)
    yError = yTarget-yDroneFiltered
    zError = int(zTarget)-zDroneFiltered
    angleError = getAngleError(angleDroneFiltered, angleTarget, angleError_old)


    # compute integral (sum) of errors (I)
    xErrorI += xError
    yErrorI += yError
    zErrorI += zError
    # ErrorI is clamped to prevent spooling
    # Note that zErrorI is handled in a non symmetric manner because it is a non symmetric response
    zErrorI = clamp(zErrorI, -(50/KIz), (100/KIz))
    xErrorI = clamp(xErrorI, -(50/KIx), (50/KIx))
    yErrorI = clamp(yErrorI, -(50/KIy), (50/KIy))
    angleErrorI += angleError

    # compute derivative (variation) of errors (D)
    xErrorD = xError-xError_old
    yErrorD = yError-yError_old
    zErrorD = zError-zError_old
    angleErrorD = angleError-angleError_old

    #User control
    if (autoControl == 0):
        control = 1000
        
        # create the command to send to Arduino to tell arduino to user user inputs
        command = "%i,%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle, control)

    #Computer control   
    else:
        control = 2000
        
        # compute commands
        #   note I and D is weighted with the fps because the fps may vary computer to computer
        xCommand = KPx*xError + KIx*xErrorI*fps/60 + KDx*xErrorD*fps/60
        yCommand = KPy*yError + KIy*yErrorI*fps/60 + KDy*yErrorD*fps/60
        zCommand = KPz*zError + KIz*zErrorI*fps/60 + KDz*zErrorD*fps/60
        angleCommand = KPangle*angleError + KIangle*angleErrorI*fps/60 + KDangle*angleErrorD*fps/60

        # throttle command is zCommand
        # commands are relative to the middle PPM values
        throttleCommand = throttle_middle + zCommand

        # angleDrone to radians
        angleDroneRad = angleDrone * np.pi/180

        # project xCommand and yCommand on the axis of the drone
        # commands are relative to the middle PPM values
        elevatorCommand = elevator_middle + -np.sin(angleDroneRad)*xCommand + -np.cos(angleDroneRad)*yCommand
        aileronCommand = aileron_middle + np.cos(angleDroneRad)*xCommand + -np.sin(angleDroneRad)*yCommand

        # rudder command is angleCommand
        # commands are relative to the middle PPM values
        rudderCommand = rudder_middle + angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(clamp(rudderCommand, 1000, 2000))

        #Going through ground effect zone requires lower throttle than what the PID loop will apply
        if (landPhase == 2):
            throttleCommand = dropThrottle

        #Cut throttle once landed
        if (landPhase == 3):
            throttleCommand = 1000

        #Slide backwards to dock
        if (landPhase == 4):
            throttleCommand = 1300
            elevatorCommand = 1300

        #Cut throttle once docked
        if (landPhase == 5):
            throttleCommand = 1000

        #Give control back to that user
        if (landPhase == 6):
            control = 1000
      
        # create the command to send to Arduino
        command = "%i,%i,%i,%i,%i" % (throttleCommand, aileronCommand, elevatorCommand, rudderCommand, control)

        # print the projected commands
        #    Note this will slow the program down significantly, only use for debugging purposes
        #print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(throttleCommand, aileronCommand, elevatorCommand, rudderCommand))

    # send command to Arduino via serial port
    command = command + "\n"
    arduino.write(command.encode())
 
    #--- Display the frame
    #    Note this will slow the program down significantly, only use for debugging purposes
    #cv2.imshow('frame', frame)
    
    # wait 1 ms for a key to be pressed
    #   NOTE: this line must be present for imshow to display the frame
    #key = cv2.waitKey(2)

    #Exit program is esc is pressed
    if keyboard.is_pressed('esc'): #ESC
        print("Exit!")
        break

    #Reset if r is pressed
    if (not(landPhase == 0) and keyboard.is_pressed('r')): #Reset
        print("Reset")
        print("Landing pad now ready for next landing")
        print("**************************************")
        print(" ")
        landPhase = 0
        autoControl = 0


time.sleep(0.5)
# send a neutral command to the drone on exit (throttle off)
i = 0
while(i < 10):
    command = "%i,%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle, 1000)
    command = command + "\n"
    arduino.write(command.encode())
    time.sleep(0.1)
    i += 1

# release video
cv2.destroyAllWindows()

# close the connection and reopen it
arduino.close()
arduino = serial.Serial(usb_port, 115200, timeout=.01)
arduino.close()
