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

speech.speak("Initializing Modules")

sensors = SensorsController(pi)
motors = MotorController(pi, decoder = sensors.getDecoder())
panTilt = PanTiltControl(Constants.panPin, Constants.tiltPin, pi)
ai = aiController(pi, speech, motors, panTilt, rCV, sensors)
print("finished initializing ai controller")
#speech.speak("Starting Modules")

sensors.start()
motors.start()
panTilt.start()
rCV.start()
time.sleep(.5)

ai.start()


rCV.setCurrentVisionFunction("LineFollower")

'''
print("Waiting for line image in main")
while rCV.getCurrentVisionFunctionValue("modified") is None:
    time.sleep(0.1)
    continue
'''

#implement while update loop here
for i in range(100):
    time.sleep(1.0 / 20.0)
    ai.update()


#speech.speak("Shutting down all modules")

motors.stop()
sensors.stop()
panTilt.stop()
rCV.stop()
ai.stop()
speech.stop()
pi.stop()
