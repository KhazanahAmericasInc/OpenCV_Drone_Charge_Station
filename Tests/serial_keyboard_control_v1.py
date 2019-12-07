import serial
import time, keyboard

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n


throttle_rate=7
aileron_rate=500
elevator_rate=500
rudder_rate=400

motors_on = False;

# define middle PPM values that make the drone hover
throttle_middle = 1600  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000

arduino=serial.Serial('COM7', 115200, timeout=.01)

# wait a bit for the connection to settle
time.sleep(2)


throttle = throttle_off
aileron = aileron_middle
elevator = elevator_middle
rudder = rudder_middle

while (1):
    
    aileron = aileron_middle
    elevator = elevator_middle
    rudder = rudder_middle

    if keyboard.is_pressed('esc'): #ESC
        break
    
    if keyboard.is_pressed('w'): #throttle up
        throttle += throttle_rate

    elif keyboard.is_pressed('s'): #throttle down
        throttle -= throttle_rate

    if keyboard.is_pressed('a'): #yaw left
        rudder -= rudder_rate

    if keyboard.is_pressed('d'): #yaw right
        rudder += rudder_rate

    if keyboard.is_pressed('right'): #roll right
        aileron += aileron_rate

    if keyboard.is_pressed('left'): #roll left
        aileron -= aileron_rate
    
    if keyboard.is_pressed('up'): #pitch forward
        elevator += elevator_rate

    if keyboard.is_pressed('down'): #pitch back
        elevator -= elevator_rate

        
    throttle = clamp (throttle, throttle_off, 2000)
    aileron = clamp (aileron, 1000, 2000)
    elevator = clamp (elevator, 1000, 2000)
    rudder = clamp (rudder, 1000, 2000)
    

    if (throttle > throttle_off):
        motors_on = True

    if (motors_on):
        command = "%i,%i,%i,%i,%i" % (throttle, aileron, elevator, rudder,2000) 
        command = command + "\n"
        arduino.write(command.encode())
        print("%i,%i,%i,%i" % (throttle, aileron, elevator, rudder))
        if (throttle <= throttle_off):
            motors_on = False



command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle) 
command = command + "\n"
arduino.write(command.encode())







