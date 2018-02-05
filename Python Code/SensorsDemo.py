from SensorsController import *
from MotorController import *
import pigpio
from SpeechController import *


pi = pigpio.pi()

speech = SpeechController()
speech.start()

speech.speak("Program starting.")

speech.speak("Booting sensors.")
sensors = SensorsController(pi)
sensors.start()

speech.speak("Preparing motors.")
motors = MotorController(pi,
                        Constants.leftMotorForward,
                        Constants.leftMotorReverse,
                        Constants.rightMotorForward,
                        Constants.rightMotorReverse,
                        decoder = sensors.getDecoder())
motors.start()


for i in range(3):
    speech.sayVoltage(sensors.getBatteryVoltage())
    time.sleep(2)


speech.speak("Shutdown sequence commencing.")


sensors.stop()
motors.stop()
speech.stop()
pi.stop()
