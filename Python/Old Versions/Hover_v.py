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


#***********************************************

#Position Targets
xTarget = [16]
yTarget = [6]
zTarget = [20]
angleTarget = [100]

#--- Define Tag
id_to_find  = 72
marker_size  = 3.2 #- [cm]

# specify the USB port for the Arduino
usb_port = 'COM5'

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

def getAngleDifference(a, b)
    difference = b - a
    while (differece < -180):
        difference += 360
    while (difference > 180):
        difference -= 360
    return difference

ii = 0

# record everything
timeRecord = []
xRecord = []
xFilteredRecord = []
yRecord = []
yFilteredRecord = []
zRecord = []
zFilteredRecord = []
angleRecord = []
angleFilteredRecord = []
xErrorRecord = []
yErrorRecord = []
zErrorRecord = []
angleErrorRecord = []
aileronRecord = []
elevatorRecord = []
throttleRecord = []
rudderRecord = []

# define a clamping function
def clamp(n, minimum, maximum):
    return max(min(maximum, n), minimum)

# define a Butterworth filter to filter the measurements
# call y = lfilter(b, a, data) to filter the data signal
order = 2  # second order filter
fs = 30  # sampling frequency is around 30 Hz
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

# first set of gains (office)
# KPx, KPy, KPz, KPangle = 10, 10, 9, -3
# KIx, KIy, KIz, KIangle = 0, 0, 0.15, 0
# KDx, KDy, KDz, KDangle = 160, 160, 115, 10

# # second set with filtering of measurements (30 fps)
KPx, KPy, KPz, KPangle = 3, 3, 4, -3
KIx, KIy, KIz, KIangle = 0, 0, 0.1, 0
KDx, KDy, KDz, KDangle = 120, 120, 80, 10

# third set with video recording (23 fps)
# KPx, KPy, KPz, KPangle = 3, 3, 4, -3
# KIx, KIy, KIz, KIangle = 0, 0, 0.1*30/23, 0
# KDx, KDy, KDz, KDangle = 120*23/30, 120*23/30, 80*23/30, 10*23/30

# third set for trajectory
# KPx, KPy, KPz, KPangle = 6, 6, 8, -3
# KIx, KIy, KIz, KIangle = 0, 0, 0.2, 0
# KDx, KDy, KDz, KDangle = 110, 110, 100, 10

# define middle PPM values that make the drone hover
throttle_middle = 1600  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000

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
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_Logitech.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_Logitech.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()


#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# wait a bit for the connection to settle
time.sleep(2)

count = 0.01

while True:

    #angleTarget = [int(count)]
    #count += 0.5
    #if (count >180):
       # count = -180

    #print("[Angle Target]: Angle={:.0f}".format(int(count)))

    # drone detected or not
    droneDetected = False

    #-- Read the camera frame
    ret, frame = cap.read()

    #Flip Image
    frame = cv2.flip(frame, -1)

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    
    if ids is not None and ids[0] == id_to_find:

        droneDetected = True

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
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)


        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        print (yaw_marker)
        print ("     ")


        xDrone = tvec[0]
        yDrone = tvec[1]
        zDrone = tvec[2]
        angleDrone = math.degrees(yaw_marker)

        str_position = "DRONE Position x=%4.0f  y=%4.0f  z=%4.0f yaw=%4.0f"%(xDrone, yDrone, zDrone, angleDrone)
        cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


    else:
        framesWithoutDrone += 1

    # abort if we lost the drone for about 200 secs
    if framesWithoutDrone >= 6000:
        k = 0
        print("Lost the drone! Abort!")
        while (k < 10):
            command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle)
            command = command + "\n"
            arduino.write(command.encode())
            time.sleep(0.1)
            k += 1
        break

    # wait 1 ms for a key to be pressed
    key = cv2.waitKey(20)

    # record the position and orientation
    xRecord.append(xDrone)
    yRecord.append(yDrone)
    zRecord.append(zDrone)
    angleRecord.append(angleDrone)

    # filter x
    xFiltered = lfilter(b, a, xRecord)
    xDroneFiltered = xFiltered[-1]
    xFilteredRecord.append(xDroneFiltered)

    # filter y
    yFiltered = lfilter(b, a, yRecord)
    yDroneFiltered = yFiltered[-1]
    yFilteredRecord.append(yDroneFiltered)

    # filter z
    zFiltered = lfilter(b, a, zRecord)
    zDroneFiltered = zFiltered[-1]
    zFilteredRecord.append(zDroneFiltered)

    # filter angle
    angleFiltered = lfilter(b, a, angleRecord)
    angleDroneFiltered = angleFiltered[-1]
    angleFilteredRecord.append(angleDroneFiltered)

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

    xError = xTarget[ii]-xDroneFiltered
    yError = yTarget[ii]-yDroneFiltered
    zError = zTarget[ii]-zDroneFiltered
    angleError = angleTarget[ii]-angleDroneFiltered

    ii += 1

    # compute integral (sum) of errors (I)
    # should design an anti windup for z
    xErrorI += xError
    yErrorI += yError
    zErrorI += zError
    angleErrorI += angleError

    # compute derivative (variation) of errors (D)
    xErrorD = xError-xError_old
    yErrorD = yError-yError_old
    zErrorD = zError-zError_old
    angleErrorD = angleError-angleError_old

    # compute commands
    xCommand = KPx*xError + KIx*xErrorI + KDx*xErrorD
    yCommand = KPy*yError + KIy*yErrorI + KDy*yErrorD
    zCommand = KPz*zError + KIz*zErrorI + KDz*zErrorD
    angleCommand = KPangle*angleError + KIangle*angleErrorI + KDangle*angleErrorD

    # print the X, Y, Z, angle commands
    #print("[COMMANDS]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))

    # throttle command is zCommand
    # commands are relative to the middle PPM values
    throttleCommand = throttle_middle + zCommand

    # angleDrone to radians for projection
    angleDroneRad = angleDrone * np.pi/180

    # project xCommand and yCommand on the axis of the drone
    # commands are relative to the middle PPM values
    elevatorCommand = elevator_middle + -np.sin(angleDroneRad)*xCommand + np.cos(angleDroneRad)*yCommand
    aileronCommand = aileron_middle + np.cos(angleDroneRad)*xCommand + np.sin(angleDroneRad)*yCommand

    # rudder command is angleCommand
    # commands are relative to the middle PPM values
    rudderCommand = rudder_middle + angleCommand

    # round and clamp the commands to [1000, 2000] us (limits for PPM values)
    throttleCommand = round(clamp(throttleCommand, 1000, 2000))
    aileronCommand = round(clamp(aileronCommand, 1000, 2000))
    elevatorCommand = round(clamp(elevatorCommand, 1000, 2000))
    rudderCommand = round(clamp(rudderCommand, 1000, 2000))

    throttleCommand = 1500
    elevatorCommand = 1500
    aileronCommand = 1500

    # create the command to send to Arduino
    command = "%i,%i,%i,%i" % (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

    print("[Location]: x={:.0f} y={:.0f} z={:.0f} angle={:.0f}".format(xDrone, yDrone, zDrone, angleDrone))
    # print the projected commands
    print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(throttleCommand, aileronCommand, elevatorCommand, rudderCommand))

    # send to Arduino via serial port
    command = command + "\n"
    arduino.write(command.encode())

    xErrorRecord.append(xError)
    yErrorRecord.append(yError)
    zErrorRecord.append(zError)
    angleErrorRecord.append(angleError)

    elevatorRecord.append(elevatorCommand)
    aileronRecord.append(aileronCommand)
    throttleRecord.append(throttleCommand)
    rudderRecord.append(rudderCommand)

    # if ESC is pressed, stop the program
    if key == 27:
        print("Exit!")
        break

    loopsCount += 1
 
    #--- Display the frame
    cv2.imshow('frame', frame)


time.sleep(0.5)
# send a neutral command to the drone (throttle off)
i = 0
while(i < 10):
    command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle)
    command = command + "\n"
    arduino.write(command.encode())
    time.sleep(0.1)
    i += 1

# release capture and close all the windows
cap.stop()

# release video
out.release()
cv2.destroyAllWindows()

# close the connection and reopen it
arduino.close()
arduino = serial.Serial(usb_port, 115200, timeout=.01)
arduino.close()
