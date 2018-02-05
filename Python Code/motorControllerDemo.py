from MotorController import *
import Constants
import pigpio
import time
from SensorsController import *

pi = pigpio.pi()

sensors = SensorsController(pi)
sensors.start()

motors = MotorController(pi,
                        Constants.leftMotorForward,
                        Constants.leftMotorReverse,
                        Constants.rightMotorForward,
                        Constants.rightMotorReverse,
                        decoder = sensors.getDecoder())

motors.start()
motors.stop()

print("Forward 100%")
motors.forward(50)


for i in range(100):
    print("DAvg: " + str(sensors.getDecoder().getDeltaAverages()))
    time.sleep(0.05)

sensors.stop()
motors.stop()
pi.stop()
