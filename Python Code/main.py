import pigpio
import time
import Constants
#from RoboCV import *
from MotorController import *
from SensorsController import *
from SpeechController import *
from aiController import *
from aiState import *
import PanTiltControl
from threading import Thread
import imp
#Main control for the robot here...
#needs to keep track of CV, IR, ultrasonic, etc...


RoboCVRef = imp.load_source('RoboCV', './OpenCV stuff/RoboCV.py')
rCV = RoboCVRef.RoboCV()

#initializing MotorControllor, Sensors, Speach, PanTiltControl, & aiController
pi = pigpio.pi()
speech = SpeechController()
speech.start()

speech.speak("Initializing Modules")

sensors = SensorsController(pi)
motors = MotorController(pi, Constants.leftMotorForward, Constants.leftMotorReverse, Constants.rightMotorForward, Constants.rightMotorReverse, decoder = sensors.getDecoder())
panTilt = PanTiltControl(pi, Constants.panPin, Constants.tiltPin)
ai = aiController(pi)

speech.speak("Starting Modules")

sensors.start()
motors.start()
panTilt.start()
rCV.start()
ai.start()


#implement while update loop here


speech.speak("Shutting down all modules")

motors.stop()
sensors.stop()
panTilt.stop()
rCV.stop()
ai.stop()
speech.stop()
pi.stop()
