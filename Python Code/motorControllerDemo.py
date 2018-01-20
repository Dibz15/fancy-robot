from MotorController import *
import Constants
import RPi.GPIO as GPIO
import pigpio
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

pi = pigpio.pi()

controller = MotorController(pi, Constants.leftMotorForward, Constants.leftMotorReverse, Constants.rightMotorForward, Constants.rightMotorReverse)

controller.start()

print("Forward 100%")
controller.forward(100)
time.sleep(12)


controller.stop()
pi.stop()
GPIO.cleanup()
