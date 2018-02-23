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

c = 0
while (c < 70):
	c += 5
	ptControl.setServo(Constants.tiltPin, c)
	time.sleep(0.1)

rCV = RoboCV()
rCV.start()
rCV.setCurrentVisionFunction("LineFollower")

while rCV.getCurrentVisionFunctionValue("lineImage") is None:
	time.sleep(0.1)
	continue

startTime = time.time()

hasLooked = False
looking = False
lookingRight = False
lookingLeft = False
timeStartedLeft = -1
timeStartedRight = -1

while time.time() - startTime < 10:
	# show the frames
	time.sleep( 1.0 / 10.0 )

	#cv2.imshow("Frame", rCV.getUnmodifiedFrame())
	#cv2.imshow("Line Image", rCV.getCurrentVisionFunctionValue("lineImage"))
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

	if rCV.getCurrentVisionFunctionValue("missingContours") < 3 and not looking:
		direction = rCV.getCurrentVisionFunctionValue("direction")
		print("Direction " + str(direction))

		looking = False
		hasLooked = False

		maxPwr = 80.0
		minPwr = 30.0
		maxDirection = 500.0

		if direction < 0:
			#right
			dirRight = abs(direction)
			capped = min(maxDirection, dirRight)
			leftPwr = minPwr + (capped / maxDirection * (maxPwr - minPwr))
			rightPwr = minPwr#minPwr - (capped / maxDirection * (minPwr))
			mController.forwardPower(leftPwr, rightPwr)
			print("Right: " + str(leftPwr))

		elif direction > 0:
			#left
			dirLeft = abs(direction)
			capped = min(maxDirection, dirLeft)
			rightPwr = minPwr + (capped / maxDirection * (maxPwr - minPwr))
			leftPwr = minPwr#minPwr - (capped / maxDirection * (minPwr))
			mController.forwardPower(leftPwr, rightPwr)
			print("Left: " + str(rightPwr))

		elif direction == 0:
			print("Forward: " + str(minPwr))
			mController.forward(minPwr)

	elif looking:
		#integrate algorithm to look
		if rCV.getCurrentVisionFunctionValue("missingContours") < 3:
			looking = False
			lookingLeft = False
			lookingRight = False
			timeStartedLeft = -1
			timeStartedRight = -1

		if lookingLeft is False and lookingRight is False:
			lookingLeft = True
			timeStartedLeft = time.time()
		elif lookingLeft is True and lookingRight is False:
			if(time.time() - timeStartedLeft > 0.5):
				lookingLeft = False
				timeStartedLeft = -1
				lookingRight = True
				timeStartedRight = time.time()
		elif lookingLeft is False and lookingRight is True:
			if time.time() - timeStartedRight > 0.6:
				lookingRight = False
				timeStartedRight = -1
				looking = False
				hasLooked = True

		if lookingLeft:
			mController.leftTurn(60)
		elif lookingRight:
			mController.rightTurn(50)
		else:
			mController.stop()

	else:
		mController.stop()
		if not hasLooked:
			looking = True
		continue


rCV.stop()
mController.stop()
ptControl.stop()
pi.stop()
GPIO.cleanup()
