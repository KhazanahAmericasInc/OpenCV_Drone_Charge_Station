import serial
import time

# define middle PPM values that make the drone hover
throttle_middle = 1600  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000

arduino=serial.Serial('COM5', 115200, timeout=.01)

# wait a bit for the connection to settle
time.sleep(2)

command = "%i,%i,%i,%i" % (1000, 1500, 1500, 1500) #Throttle, 
command = command + "\n"
arduino.write(command.encode())
print("Low throttle")

time.sleep(3)
command = "%i,%i,%i,%i" % (1200, 1500, 1500, 1500) #Throttle, 
command = command + "\n"
arduino.write(command.encode())
print("High throttle")

time.sleep(3)
command = "%i,%i,%i,%i" % (1000, 1500, 1500, 1500) #Throttle, 
command = command + "\n"
arduino.write(command.encode())
print("Low throttle")










