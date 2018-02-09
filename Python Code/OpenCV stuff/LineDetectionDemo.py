'''
*	File: LineDetectionDemo.py
*	Description:	Demo showing off the line detection algorithm
*	Author(s):		Austin Dibble
*	Date Created:	1/18/18
'''
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

#Import Constant.py module from different directory
Constants = imp.load_source('Constants', '../Constants.py')

#Turn off RPi GPIO warnings
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Initialize pigpio library object
pi = pigpio.pi()

#Exit if there was an error with pigpio
if not pi.connected:
	exit()

#Start RoboCV module
rCV = RoboCV()
rCV.start()

#Set our current image processing to line following
rCV.setCurrentVisionFunction("LineFollower")

#Wait until we grab an image
while rCV.getCurrentVisionFunctionValue("lineImage") is None:
	time.sleep(0.1)
	continue

#Keep track of our start time (in case we want to limit our run-time...)
startTime = time.time()

#Keep doing this until the user exits
while True:
	# ~10fps
	time.sleep( 1.0 / 10.0 )

	#Show our modified frame (with direction and angle information overlayed)
	cv2.imshow("Frame", rCV.getCurrentVisionFunctionValue("modified"))
	#Show our binary color-detection image
	cv2.imshow("Line Image", rCV.getCurrentVisionFunctionValue("lineImage"))
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

	#As long as we aren't missing too many contours, we can trust the direction info
	if rCV.getCurrentVisionFunctionValue("missingContours") < 3:
		direction = rCV.getCurrentVisionFunctionValue("direction")
		#Print out our direction information
		print("Direction " + str(direction))


#Clean up our resources
rCV.stop()
pi.stop()
GPIO.cleanup()
