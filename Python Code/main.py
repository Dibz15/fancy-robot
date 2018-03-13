'''
*	File: main.py
*	Description:  This file holds the Main thread which will pass the motor, speech, sensors, CV,
and PanTilt controllers to an instantiated aiController class. It loops through the update function
of the aiController class, which in turn calls the update function of the current aiState which
is on top of the stack.
*
*	Author(s):		Michael Lee
*	Date Created:	1/28/17
'''

import pigpio
import time
import Constants

from MotorController import *
from SensorsController import *
from SpeechController import *
from aiController import *
from aiState import *
from PanTiltControl import *
from threading import Thread
import imp
from OpenCV.RoboCV import *
from BreakHandler import *

#Main control for the robot here...
#needs to keep track of CV, IR, ultrasonic, etc...

#RoboCVRef = imp.load_source('RoboCV', './OpenCV stuff/RoboCV.py')
rCV = RoboCV()

#initializing MotorControllor, Sensors, Speach, PanTiltControl, & aiController
pi = pigpio.pi()
if not pi.connected:
    print("Pigpio not initialized.")
    exit()

speech = SpeechController()
speech.start()
#speech.speak("Initializing Modules")
sensors = SensorsController(pi)
motors = MotorController(pi, decoder = sensors.getDecoder())
panTilt = PanTiltControl(Constants.panPin, Constants.tiltPin, pi)
ai = aiController(pi, speech, motors, panTilt, rCV, sensors)
print("finished initializing ai controller")
#speech.speak("Starting Modules")

breakHandler = BreakHandler(motors, sensors, panTilt, rCV, ai, speech, pi)

#starting controller operations
sensors.start()
motors.start()
panTilt.start()
rCV.start()
time.sleep(.5)
ai.start()

rCV.setCurrentVisionFunction("LineFollower")

#loops aiController's update function, which calls the update function of the current aiState robot is in
while not ai.stopped:
    time.sleep(1.0 / 20.0)
    ai.update()


#speech.speak("Shutting down all modules")
#shutting down all modules
ai.stop()
motors.stop()
sensors.stop()
panTilt.stop()
speech.stop()
rCV.stop()
pi.stop()
