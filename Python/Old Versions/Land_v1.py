##Playstation eye
##Attempting to land on small landing pad

"""
This demo calculates multiple things for different scenarios.
Here are the defined reference frames:
TAG:
                A y
                |
                |
                |tag center
                O---------> x
CAMERA:
                X--------> x
                | frame center
                |
                |
                V y
F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis
The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)
We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import serial
from scipy.signal import butter, lfilter
import keyboard


#***********************************************

#Position Targets
xTarget = [0]
yTarget = [0]
zTarget = 50.0
angleTarget = [0]

XYtransitionBoundry = 10
ZtransitionMax = 80
ZtransitionMin = 35
groundEffectHeight = 25

detectionCount = 0
detectionThreshold = 8
detectionLostThreshold = 70

phase1Time = 8
landedShutdownTime = 0.3

angleOffset = 315

dropThrottle = 1500

yTrim = 0
xTrim = 0

velocity = 0.2

#--- Define Tag
id_to_find  = 42
marker_size  = 3 #- [cm]

# specify the USB port for the Arduino
usb_port = 'COM15'

# define middle PPM values that make the drone hover
throttle_middle = 1690  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000

throttle = throttle_off

#Control channel selects computer or PPM control (from rc reciever on arduino pin 2)
control = 1000

#***********************************************

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

# Calculates the shortest error "distance" between the actual and target angle
def getAngleError(actual, target, oldError):
    
    difference = target - actual
    if (difference > 180):
        actual += 360
    if (difference < -180):
        actual -= 360
    difference = target - actual

    return difference

def ifInTransitionBoundry():
    return (zDrone > ZtransitionMin and zDrone < ZtransitionMax and yDrone > -XYtransitionBoundry
            and yDrone < XYtransitionBoundry and xDrone > -XYtransitionBoundry and xDrone < XYtransitionBoundry)
    
ii = 0

# record everything for filtering
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
fs = 60  # sampling frequency is around 30 Hz
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

# count loops
loopsCount = 0

# open serial connection with Arduino
# baudrate of 115200
arduino = serial.Serial(usb_port, 115200, timeout=.01)

# wait a bit for the connection to settle
time.sleep(1)

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


#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FPS, 120)
#cap.set(cv2.CAP_PROP_SETTINGS, 1)
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(cap.get(cv2.CAP_PROP_FPS))

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# wait a bit for the connection to settle
time.sleep(2)

now = time.perf_counter()
last = now
startStabilize = 0
landed = 0

landPhase = 0

key = 22

printcount = 0

autoControl = 0


print ("Starting")

while True:

    now = time.perf_counter()
    fps = 1/(now-last)
    last = now
    printcount +=1
    if (printcount > 40):
        #print (fps)
        printcount = 0
    
    # drone detected or not
    droneDetected = False

    #-- Read the camera frame
    ret, frame = cap.read()

    #Flip Image
    frame = cv2.flip(frame, -1)

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    #corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    
    if ids is not None and ids[0] == id_to_find:
        droneDetected = True
        detectionCount += 1

        framesWithoutDrone = 0
        
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)
        #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)


        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        xDrone = tvec[0]
        yDrone = tvec[1]
        zDrone = tvec[2]
        angleDrone = -((math.degrees(yaw_marker) + angleOffset)%365 -180)
        
        #str_position = "DRONE Position x=%4.0f  y=%4.0f  z=%4.0f yaw=%4.0f"%(xDrone, yDrone, zDrone, angleDrone)
        #cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


    else:
        framesWithoutDrone += 1

    if ((autoControl == 0) and (detectionCount > detectionThreshold) and ifInTransitionBoundry()):
        zTarget = zDrone
        autoControl = 1
        #print ("ZTarget: ")
        #print (zTarget)
        xErrorI = 0
        yErrorI = 0
        zErrorI = 0
        angleErrorI = 0

        startStabilize = time.perf_counter()

    if ((autoControl == 1) and (framesWithoutDrone >= detectionLostThreshold)):
        autoControl = 0
        detectionCount = 0
        landPhase = 0
        startStabilize = 0
        landed = 0
        print ("Drone lost")

    if ((autoControl == 1) and ((now - startStabilize) > phase1Time) and landPhase == 0 ):
        landPhase = 1
        print ("landing phase 1")       

    if (landPhase == 1):
        zTarget -= velocity
        #print (zTarget)

    if ((landPhase == 1) and (zDrone < groundEffectHeight)):
        landPhase = 2
        print ("landing phase 2")
        landed = time.perf_counter()

    if ((landPhase == 2) and ((now - landed) > landedShutdownTime)):
        landPhase = 3

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
    if ii == len(xTarget):
        ii = 0

    xError = -(xTarget[ii]-xDroneFiltered)
    yError = yTarget[ii]-yDroneFiltered
    zError = int(zTarget)-zDroneFiltered
    angleError = getAngleError(angleDroneFiltered, angleTarget[ii], angleError_old)

    ii += 1

    # compute integral (sum) of errors (I)
    xErrorI += xError
    yErrorI += yError
    zErrorI += zError
    zErrorI = clamp(zErrorI, -(50/KIz), (100/KIz))
    xErrorI = clamp(xErrorI, -(50/KIx), (50/KIx))
    yErrorI = clamp(yErrorI, -(50/KIy), (50/KIy))
    angleErrorI += angleError

    # compute derivative (variation) of errors (D)
    xErrorD = xError-xError_old
    yErrorD = yError-yError_old
    zErrorD = zError-zError_old
    angleErrorD = angleError-angleError_old


    if (autoControl == 0):
        control = 1000
        
        # create the command to send to Arduino
        command = "%i,%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle, control)
        
    else:
        control = 2000

        # abort if we lost the drone for about 200 secs
        if framesWithoutDrone >= 6000:
            k = 0
            print("Lost the drone! Abort!")
            while (k < 10):
                command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle, 1000)
                command = command + "\n"
                arduino.write(command.encode())
                time.sleep(0.1)
                k += 1
            break
        
        # compute commands
        xCommand = KPx*xError + KIx*xErrorI*fps/60 + KDx*xErrorD*fps/60 + xTrim
        yCommand = KPy*yError + KIy*yErrorI*fps/60 + KDy*yErrorD*fps/60 + yTrim
        zCommand = KPz*zError + KIz*zErrorI*fps/60 + KDz*zErrorD*fps/60
        angleCommand = KPangle*angleError + KIangle*angleErrorI*fps/60 + KDangle*angleErrorD*fps/60

        # throttle command is zCommand
        # commands are relative to the middle PPM values
        throttleCommand = throttle_middle + zCommand

        #print(" zError={:.0f} zErrorI={:.0f} zErrorD={:.0f} ".format(zError, zErrorI, zErrorD))
        #print("ZCommand={:.1f} ".format(zCommand))

        # angleDrone to radians for projection
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

        if (landPhase == 2):
            throttleCommand = dropThrottle

        if (landPhase == 3):
            throttleCommand = 1000
            
      
        # create the command to send to Arduino
        command = "%i,%i,%i,%i,%i" % (throttleCommand, aileronCommand, elevatorCommand, rudderCommand, control)

        # print the projected commands
        #print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(throttleCommand, aileronCommand, elevatorCommand, rudderCommand))

    # send to Arduino via serial port
    command = command + "\n"
    arduino.write(command.encode())

    # if ESC is pressed, stop the program
    #if key == 27:
        #print("Exit!")
        #break

    loopsCount += 1
 
    #--- Display the frame
    #cv2.imshow('frame', frame)
    
    # wait 1 ms for a key to be pressed
    #key = cv2.waitKey(2)

    if keyboard.is_pressed('esc'): #ESC
        print("Exit!")
        break

    if (not(landPhase == 0) and keyboard.is_pressed('r')): #Reset
        print("Reset")
        landPhase = 0
        autoControl = 0


time.sleep(0.5)
# send a neutral command to the drone (throttle off)
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
