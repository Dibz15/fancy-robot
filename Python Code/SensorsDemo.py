'''
*	File: SensorsDemo.py
*	Description:	This module is for demoing the function of the sensors systems.
*	Author(s):		Austin Dibble
*	Date Created:	02/01/18
'''

#Imports
from SensorsController import *
from MotorController import *
import pigpio
from SpeechController import *

#Initialize our pigpio reference
pi = pigpio.pi()

#Start the speech controller
speech = SpeechController()
speech.start()

#Say what we're doing here
speech.speak("Program starting.")

speech.speak("Booting sensors.")

#Boot sensors module
sensors = SensorsController(pi)
sensors.start()

#Start our motor controller
speech.speak("Preparing motors.")
motors = MotorController(pi,
                        Constants.leftMotorForward,
                        Constants.leftMotorReverse,
                        Constants.rightMotorForward,
                        Constants.rightMotorReverse,
                        decoder = sensors.getDecoder())
motors.start()

#Read out battery voltage
for i in range(3):
    speech.sayVoltage(sensors.getBatteryVoltage())
    #2 seconds between each read
    time.sleep(2)

speech.speak("Shutdown sequence commencing.")

#Close up our resources
sensors.stop()
motors.stop()
speech.stop()
pi.stop()
