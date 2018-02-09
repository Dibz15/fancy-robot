'''
*	File: motorControllerDemo.py
*	Description:  This module is used to demo the motor control functions
*	Author(s):		Austin Dibble
*	Date Created:	1/18/17
'''

from MotorController import *
import Constants
import pigpio
import time
from SensorsController import *

#Initialize pigpio reference
pi = pigpio.pi()

#Start the sensors controller (serial, decoders, etc.)
sensors = SensorsController(pi)
sensors.start()

#Start the motor controller
motors = MotorController(pi,
                        Constants.leftMotorForward,         #These are the pin assignments
                        Constants.leftMotorReverse,
                        Constants.rightMotorForward,
                        Constants.rightMotorReverse,
                        decoder = sensors.getDecoder())

#Start motor driver stuff
motors.start()
#Halt motors, to start
motors.halt()

#Spin motors forward at 50% power
print("Forward 50%")
motors.forward(50)

#Go for 5 seconds, getting the average deltas for the decoders
for i in range(100):
    print("DAvg: " + str(sensors.getDecoder().getDeltaAverages()))
    time.sleep(0.05)

#Stop everything, clean up resources
sensors.stop()
motors.stop()
pi.stop()
