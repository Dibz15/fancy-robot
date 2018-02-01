from SensorsController import *
from MotorController import *
import pigpio

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


time.sleep(10)

sensors.stop()
motors.stop()
