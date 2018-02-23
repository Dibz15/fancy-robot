import pigpio
import time
import Constants
from OpenCV.RoboCV import *
from MotorController import *
from SensorsController import *
from SpeechController import *
from aiController import *
from aiState import *
from PanTiltControl import *
from threading import Thread
import imp
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
ai.start()


#implement while update loop here
for i in range(5):
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
