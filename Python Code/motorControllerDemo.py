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
#sensors = SensorsController(pi)
#sensors.start()

#Start the motor controller
motors = MotorController(pi)

#Start motor driver stuff
motors.start()
#Halt motors, to start
time.sleep(1)

motors.takeControl()

while not motors.manualControl:
    time.sleep(0.2)

#motors.forward(100, 100)

time.sleep(1)

motors.releaseControl()

time.sleep(1)

#Stop everything, clean up resources
#sensors.stop()
motors.stop()
pi.stop()
