from collections import deque
import numpy as np
import imutils
import cv2
from skimage import exposure
from cvlib import *
from RoboCV import *
#import Constants
import os
import RPi.GPIO as GPIO
import pigpio
#import PanTiltControl

import imp

Constants = imp.load_source('Constants', '../Constants.py')
fooP = imp.load_source('PanTiltControl', '../PanTiltControl.py')
fooM = imp.load_source('MotorController', '../MotorController.py')

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()

if not pi.connected:
	exit()

ptControl = fooP.PanTiltControl(Constants.panPin, Constants.tiltPin, pi)
ptControl.start()
mController = fooM.MotorController(pi, Constants.leftMotorForward, Constants.leftMotorReverse, Constants.rightMotorForward, Constants.rightMotorReverse)
mController.start()

'''
rCV = RoboCV()
rCV.start()
rCV.setCurrentVisionFunction("LineFollower")

while True:
	# show the frame
	cv2.imshow("Frame", rCV.getUnmodifiedFrame())
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
'''

mController.stop()

print("Right")
mController.rightTurn(100)
time.sleep(3)

print("Left")
mController.leftTurn(100)
time.sleep(3)

print("Reverse")
mController.reverse(100)
time.sleep(3)

print("Forward")
mController.forward(100)
time.sleep(3)


#rCV.stop()
mController.stop()
ptControl.stop()
pi.stop()
GPIO.cleanup()
