# Simple Adafruit BNO055 sensor reading example.  Will print the orientation
# and calibration data every second.
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import sys
import time

from Adafruit_BNO055 import BNO055

# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

#create data file
datafile = open('testfile.txt','w')
datafile.write('Time(sec) \t DeltaT \t Heading(deg) \t Roll(deg) \t Pitch(deg) \t xgyro(deg/s) \t ygyro(deg/s) \t zgyro(deg/s) \n')
tstart = time.time()
told = time.time()

while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Gyroscope data (in degrees per second):
    xgyro,ygyro,zgyro = bno.read_gyroscope()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    xlinacc,ylinacc,zlinacc = bno.read_linear_acceleration()
    #get the curent time
    deltat = time.time()-told
    tcurrent = time.time()-tstart
    told = tcurrent+tstart
    print(tcurrent)
    # Print everything to the data file.
    datafile.write(str(tcurrent) + '\t' + str(deltat) + '\t' + str(heading) + '\t' + str(roll) + '\t' + str(pitch) + '\t' + str(xgyro) + '\t' + str(ygyro) + '\t' + str(zgyro) + '\n')

    # Accelerometer data (in meters per second squared):
    #x,y,z = bno.read_accelerometer()

    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.
    # time.sleep(1)
datafile.close
