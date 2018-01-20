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

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()

if not pi.connected:
	exit()

rCV = RoboCV()
rCV.start()
rCV.setCurrentVisionFunction("LineFollower")

while rCV.getCurrentVisionFunctionValue("lineImage") is None:
	time.sleep(0.1)
	continue

startTime = time.time()


while True:
	# show the frames
	time.sleep( 1.0 / 10.0 )

	cv2.imshow("Frame", rCV.getCurrentVisionFunctionValue("modified"))
	cv2.imshow("Line Image", rCV.getCurrentVisionFunctionValue("lineImage"))
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

	if rCV.getCurrentVisionFunctionValue("missingContours") < 3:
		direction = rCV.getCurrentVisionFunctionValue("direction")
		print("Direction " + str(direction))


rCV.stop()
pi.stop()
GPIO.cleanup()
