#

import numpy as np
import argparse
import imutils
import cv2
from skimage import exposure
from cvlib import *

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image file")

args = vars(ap.parse_args())


frame = cv2.imread(args["image"], 0)

if not frame is None:

    while True:
        #resize the frame, blur it, convert it to HSV
        frame = imutils.resize(frame, height=400)
        ratio = frame.shape[0] / 400
        orig = frame.copy()

        filtered = cv2.bilateralFilter(frame.copy(), 11, 17, 17)
        edges = auto_canny(filtered, cv2.getTrackbarPos('Sigma', 'window') / 100)

        screenCnt = get_rect(edges)
        warp = warp_perspective(orig, screenCnt, ratio, 600, 400)

        thresh = None
        if warp is not None:
            cv2.imshow("Warp", warp)
            #warp = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
            warp = exposure.rescale_intensity(warp, out_range = (0, 255))
            (ret, thresh) = cv2.threshold(warp, 127, 255, cv2.THRESH_BINARY)
            cv2.imshow("threshold", thresh)

        #print(mouseX, mouseY)
        #pgui.moveTo(mouseX, mouseY, 0.0)

        #show our frame
        cv2.imshow("Frame", frame)
        #cv2.imshow("Mask", colorMask)
        cv2.imshow("Edges", edges)

        if not thresh is None:
            cv2.imwrite("binarized.jpg", thresh)

        key = cv2.waitKey(1) & 0xFF

        #if 'q' is pressed, stop the loop
        if key == ord("q"):
            break

cv2.destroyAllWindows()
