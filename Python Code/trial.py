
import pigpio
import time
from SpeechController import *
from SensorsController import *
from MotorController import *
pi = pigpio.pi()

speech = SpeechController(voiceType = 1 , speechRate = 150)

speech.start()

speech.speak("I am returning to base station")
speech.stop()
