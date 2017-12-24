from collections import deque
import numpy as np
import imutils
import cv2
from skimage import exposure
from cvlib import *
from RoboCV import *
import os

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

rCV.stop()
