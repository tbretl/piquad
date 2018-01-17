##Marie Beasley
##Last Updated: Jan 18th, 2018
##Flight 1
##Uses State Estimation for the position and the linear velocity, does not use this information for anything, just outputs to the data file
##Does not use the camera at all
##Has the planner function, but only for future use


#############################################################
## this code is set up so that it will directly control the #
## PWM of motors 1 and 3 in attempt to control the pitch    #
## angle of the quadrotor                                   #
#############################################################

#############################################################
#           Import the various necessary things             #
#############################################################
import logging
import sys
import time
import math
import numpy

from Adafruit_BNO055 import BNO055 #for the IMU


#needed for the motors
import os     #importing os library so as to communicate with the system
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1.5) #it is impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library





#############################################################
#           Planner and Controller Functions                #
#############################################################

def Planner(Quad,tcurrent):
    #determine the desired position to pass to the controller
    xgoal = 0
    ygoal = 0
    zgoal = 0.5
    #all the planner stuff goes here
    #r = 0.2
    #
    #for now, do it without a planner
    xdesired = xgoal
    ydesired = ygoal
    zdesired = zgoal
    Controller(Quad,xdesired,ydesired,zdesired,tcurrent)

def Controller(Quad,xdesired,ydesired,zdesired,tcurrent): #take in current state and goal position to determine the desired inputs
    maxsigma = 1000
    k_M = 1.13e-7 #using values from in lab quadrotors
    k_F = 5.46e-5 #using values from in lab quadrotors
    lx = 0.28575/2 #x spar length (m)
    ly = 0.3556/2 #y spar length (m)
    m = 1 #mass of the quadcopter (kg)
    g = 9.81 #acceleration due to gravity (m/s^2)
    K = numpy.matrix([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    state = numpy.matrix([[Quad.xpos], [Quad.ypos], [Quad.zpos], [Quad.yaw], [Quad.roll], [Quad.pitch], [Quad.xvel], [Quad.yvel], [Quad.zvel], [Quad.w1], [Quad.w2], [Quad.w3]])
    goalstate = numpy.matrix([[xdesired], [ydesired], [zdesired], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
    diff = state-goalstate
    u = K*diff
    #u = numpy.matrix([[s1],[0],[s3],[0]]) #just for now to check that the motors are running
    #check to ensure within max spinrate
    #
    #
    #

    pitchgoal = 0
    
    u1 = 0 #u[0]
    u2 = -0.4*(Quad.pitch-pitchgoal) - 0.4594*Quad.w2 #u[1]
    #-0.5515
    #-0.4594
    u3 = 0 #u[2]
    u4 = 0 #u[3]


    #Convert to spin rates
    W = numpy.array([[k_F*ly,-k_F*ly,k_F*ly,-k_F*ly], [k_F*lx,k_F*lx,-k_F*lx,-k_F*lx], [k_M, -k_M, -k_M, k_M], [-k_F, -k_F, -k_F, -k_F]])
    u = numpy.array([[u1],[u2],[u3],[u4]])

    spin = numpy.linalg.solve(W,u)


 
    
    
    MotorsControl(float(spin[0]),float(spin[1]),float(spin[2]),float(spin[3])) #call function to update the motor speeds - NEED TO HAVE THE INT CONVERSION




#############################################################
#                   Class Definitions                       #
#############################################################

#create the quadrotor class
class QuadState:
    def __init__(self,xpos,ypos,zpos,yaw,roll,pitch,xvel,yvel,zvel,w1,w2,w3):
        self.xpos = xpos
        self.ypos = ypos
        self.zpos = zpos
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch
        self.xvel = xvel
        self.yvel = yvel
        self.zvel = zvel
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3

#create the obstacle class
class ObstacleState:
    def __init__(self,xpos,ypos,zpos):
        self.xpos = xpos
        self.ypos = ypos
        self.zpos = zpos





#############################################################
#                    Motor Functions                        #
#############################################################

# Motor Configuration:
#   1:yellow(c)      2:orange
#                 .
#                 .
#   3:white(cc)       4:blue(c)
Motor1=6 #Connect Motor1 in this GPIO pin (yellow)
Motor2=13 #Connect Motor2 in this GPIO pin (orange)
Motor3=19 #Connect Motor3 in this GPIO pin (white)
Motor4=26 #Connect Motor4 in this GPIO pin (blue)
max_value = 2000 #max PWM
min_value = 700  #min PWM
pi = pigpio.pi()
def MotorsStart():
    pi.set_servo_pulsewidth(Motor1, 0)
    #pi.set_servo_pulsewidth(Motor2, 0)
    pi.set_servo_pulsewidth(Motor3, 0)
    #pi.set_servo_pulsewidth(Motor4, 0)
def MotorsCalibrate():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(Motor1, 0)
    #pi.set_servo_pulsewidth(Motor2, 0)
    pi.set_servo_pulsewidth(Motor3, 0)
    #pi.set_servo_pulsewidth(Motor4, 0)
    pi.set_servo_pulsewidth(Motor1, max_value)
    #pi.set_servo_pulsewidth(Motor2, max_value)
    pi.set_servo_pulsewidth(Motor3, max_value)
    #pi.set_servo_pulsewidth(Motor4, max_value)
    print("You will here two beeps, then wait for a gradual falling tone then press Enter")
    inp = raw_input()         
    pi.set_servo_pulsewidth(Motor1, min_value)
    #pi.set_servo_pulsewidth(Motor2, min_value)
    pi.set_servo_pulsewidth(Motor3, min_value)
    #pi.set_servo_pulsewidth(Motor4, min_value)
    print "Wierd eh! Special tone"
    time.sleep(7)
    print "Wait for it ...."
    time.sleep (5)
    print "Im working on it, ....."
    pi.set_servo_pulsewidth(Motor1, 0)
    #pi.set_servo_pulsewidth(Motor2, 0)
    pi.set_servo_pulsewidth(Motor3, 0)
    #pi.set_servo_pulsewidth(Motor4, 0)
    time.sleep(2)
    print "Arming ESC now..."
    pi.set_servo_pulsewidth(Motor1, min_value)
    #pi.set_servo_pulsewidth(Motor2, min_value)
    pi.set_servo_pulsewidth(Motor3, min_value)
    #pi.set_servo_pulsewidth(Motor4, min_value)
    time.sleep(1)
    print "Calibration Complete"
def MotorsArm(): #This is the arming procedure of an ESC
    print('Arming Motors - Should hear two beeps')
    pi.set_servo_pulsewidth(Motor1, max_value)
    #pi.set_servo_pulsewidth(Motor2, max_value)
    pi.set_servo_pulsewidth(Motor3, max_value)
    #pi.set_servo_pulsewidth(Motor4, max_value)
    time.sleep(1.5)
    pi.set_servo_pulsewidth(Motor1, min_value)
    #pi.set_servo_pulsewidth(Motor2, min_value)
    pi.set_servo_pulsewidth(Motor3, min_value)
    #pi.set_servo_pulsewidth(Motor4, min_value)
    time.sleep(1)
def MotorsControl(spin1,spin2,spin3,spin4): #update the PWM of the motors
    maxi = 1500
    mini = 1100
    speed1 = (spin1+1000)
    speed2 = 0 #spin2
    speed3 = (spin3+1000)
    speed4 = 0 #spin4
    if speed1 > maxi:
        speed1 = maxi
    if speed1 < mini:
        speed1 = mini
    if speed2 > maxi:
        speed2 = maxi
    if speed2 < mini:
        speed2 = mini
    if speed3 > maxi:
        speed3 = maxi
    if speed3 < mini:
        speed3 = mini
    if speed4 > maxi:
        speed4 = maxi
    if speed4 < mini:
        speed4 = mini
    #print('control')
    print(str(int(speed1)) + '\t' + str(int(speed2)) + '\t' + str(int(speed3)) + '\t' + str(int(speed4)))
    pi.set_servo_pulsewidth(Motor1, speed1)
    #pi.set_servo_pulsewidth(Motor2, speed2)
    pi.set_servo_pulsewidth(Motor3, speed3)
    #pi.set_servo_pulsewidth(Motor4, speed4)
def MotorsStop(): #This will stop every action your Pi is performing for all motors.
    pi.set_servo_pulsewidth(Motor1, 0)
    #pi.set_servo_pulsewidth(Motor2, 0)
    pi.set_servo_pulsewidth(Motor3, 0)
    #pi.set_servo_pulsewidth(Motor4, 0)
    pi.stop()
    os.system ("sudo killall pigpiod")



#############################################################
#                    Other Functions                        #
#############################################################
def DegtoRad(deg):
    return deg*math.pi/180






#############################################################
#                       Main Program                        #
#############################################################

# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)
# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

#x is forward, y is to the right, z is down
BNO055.BNO055.set_axis_remap(bno, 1, 0, 2, x_sign=0, y_sign=1, z_sign=0)

# Calibrate the ESC
# Need to calibrate if first run after pluggin turning on
print('Need to calibrate the ESC? Y or N')
inp = raw_input()
if inp == 'Y':
    MotorsCalibrate()
# Start the motors
MotorsStart()
MotorsArm()
print('Motors Armed')
MotorsControl(1150,0,1150,0)
time.sleep(5) #delay between starting motors and beginning flight
print('Begin Flight')

#create data file
timestr = time.strftime("%Y%m%d-%H%M%S") #so the filename has the date followed by the time
datafile = open(timestr + '.txt','w')
datafile.write('Time(sec) \t DeltaT \t Xpos(m) \t Ypos(m) \t Zpos(m) \t Yaw(rad) \t Roll(rad) \t Pitch(rad) \t Xvel(m/s) \t Yvel(m\s) \t Zvel (m\s) \t xgyro(rad/s) \t ygyro(rad/s) \t zgyro(rad/s) \n')
tstart = time.time()
told = time.time()

#initial pos and vel
xpos = 0
ypos = 0
zpos = 0
xvel = 0
yvel = 0
zvel = 0



#run until ctrl+C is entered into the keyboard
try:
    while True:
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        yaw, roll, pitch = bno.read_euler()
        yaw = DegtoRad(yaw) #convert to radians
        roll = DegtoRad(roll) #convert to radians
        pitch = DegtoRad(pitch) #convert to radians
        # Gyroscope data (in degrees per second):
        xgyro,ygyro,zgyro = bno.read_gyroscope()
        xgyro = DegtoRad(xgyro) #convert to radians/sec
        ygyro = DegtoRad(ygyro) #convert to radians/sec
        zgyro = DegtoRad(zgyro) #convert to radians/sec
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        xlinacc,ylinacc,zlinacc = bno.read_linear_acceleration()
        #get the curent time
        deltat = time.time()-told
        tcurrent = time.time()-tstart
        told = tcurrent+tstart
        #estimate the current position and velocity
        xpos = xpos + xvel*deltat + xlinacc*deltat*deltat
        ypos = ypos + yvel*deltat + ylinacc*deltat*deltat
        zpos = zpos + yvel*deltat + ylinacc*deltat*deltat
        xvel = xvel + xlinacc*deltat
        yvel = yvel + ylinacc*deltat
        zvel = zvel + zlinacc*deltat
        Quad = QuadState(xpos,ypos,zpos,yaw,roll,pitch,xvel,yvel,zvel,xgyro,ygyro,zgyro)
        #print(str(Quad.xpos) + '\t' + str(Quad.ypos) + '\t' + str(Quad.zpos))
        # Print everything to the data file.
        datafile.write(str(tcurrent) + '\t' + str(deltat) +  '\t' + str(Quad.xpos) +'\t' + str(Quad.ypos) + '\t' + str(Quad.zpos) + '\t' + str(Quad.yaw) + '\t' + str(Quad.roll) + '\t' + str(Quad.pitch) + '\t' + str(Quad.xvel) + '\t' + str(Quad.yvel) + '\t' + str(Quad.zvel) + '\t' + str(Quad.w1) + '\t' + str(Quad.w2) + '\t' + str(Quad.w3) + '\n')
        Planner(Quad,tcurrent) #call the planner
            
except KeyboardInterrupt:
    datafile.close()
    MotorsStop()
    print("\nEnd Flight")

    




