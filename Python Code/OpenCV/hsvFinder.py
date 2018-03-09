from collections import deque
import numpy as np
import imutils
import cv2
from skimage import exposure
from cvlib import *
from RoboCV import *
import os
import time

rCV = RoboCV()
rCV.start()
rCV.setCurrentVisionFunction("HSVFinder")

while rCV.getCurrentVisionFunctionValue("binarized") is None:
    time.sleep(1.0 / 15.0)
    print("Waiting for binarized image")
    continue

while True:
	# show the frame
    #print("Showing frame")
    time.sleep(1.0 / 15.0)
    cv2.imshow("Frame", rCV.getCurrentVisionFunctionValue( "resized" ))
    cv2.imshow("Binarized", rCV.getCurrentVisionFunctionValue( "binarized" ))
    key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

rCV.stop()
