
import pigpio
import time
from SpeechController import *
from SensorsController import *
from MotorController import *
pi = pigpio.pi()

speech = SpeechController(voiceType = 1 , speechRate = 150)

speech.start()


sensors = SensorsController(pi)
motor = MotorController(pi, Constants.leftMotorForward, Constants.leftMotorReverse, Constants.rightMotorForward, Constants.rightMotorReverse, decoder = sensors.getDecoder())

motor.stop()
speech.speak("I have stopped motor functions, Mister wizard")
speech.speak("Your welcome")
speech.stop()
