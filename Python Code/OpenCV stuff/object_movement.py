#Import packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import pyautogui as pgui
from skimage import exposure
import glob
from cvlib import *
import os

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help = "path to the (optional video file)")
ap.add_argument("-b", "--buffer", type=int, default=32,
                help="max buffer size")
args = vars(ap.parse_args())

#Create trackbars
cv2.namedWindow('window')

cv2.createTrackbar('H_MAX', 'window', 0, 180, nothing) #H
cv2.createTrackbar('H_MIN', 'window', 0, 180, nothing)

cv2.createTrackbar('S_MAX', 'window', 0, 255, nothing) #S
cv2.createTrackbar('S_MIN', 'window', 0, 255, nothing)

cv2.createTrackbar('V_MAX', 'window', 0, 255, nothing) #V
cv2.createTrackbar('V_MIN', 'window', 0, 255, nothing)

cv2.createTrackbar('Sigma', 'window', 0, 100, nothing)

#Lower and upper HSV boundaries for pink
pinkUpper = (180, 244, 255)
pinkLower = (150, 0, 255)

yellowUpper = (45, 255, 255)
yellowLower = (18, 20, 150)

orangeUpper = (180, 255, 255)
orangeLower = (160, 0, 0)

ref_images = {}

#load in reference images
for filename in glob.glob(os.path.join('./ref_images', '*.jpg')):
    im = cv2.imread(filename, 0)
    (head, tail) = os.path.split(filename)
    name = tail.split(".")[0]
    print(name)
    im = cv2.flip(im, 1)
    (ret, thresh) = cv2.threshold(im, 127, 255, cv2.THRESH_BINARY)
    ref_images[name] = imutils.resize(thresh, width=600, height=400)

#list of tracked points, frame counter, coordinate deltas
pts = deque(maxlen=args["buffer"])
counter = 0
(dx, dy) = (0, 0)
direction = ""
(avgX, avgY) = (10, 10)

#new comment


#if no video supplied, get the webcam
if not args.get("video", False):
    camera = cv2.VideoCapture(0)

else:
    camera = cv2.VideoCapture(args["video"])

#loop over video frames
while True:
    (grabbed, frame) = camera.read()
    #If a frame wasn't grabbed and this is a video, the video is done
    if args.get("video") and not grabbed:
        break

    #resize the frame, blur it, convert it to HSV
    frame = imutils.resize(frame, height=400)
    ratio = frame.shape[0] / 400
    frame = cv2.flip(frame, 1)
    orig = frame.copy()
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)

    #find contours in mask and get the center of the objects
    colorMask = colorFinder(frame, avgX, avgY, pts, args)

        #grayScale
    gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
    grayFiltered = cv2.bilateralFilter(gray, 11, 17, 17)
    edges = auto_canny(grayFiltered, cv2.getTrackbarPos('Sigma', 'window') / 100)

    screenCnt = get_rect(edges)


    warp = warp_perspective(orig, screenCnt, ratio, 600, 400)

    if warp is not None:
        #cv2.imshow("Warp", warp)
        warp = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
        warp = exposure.rescale_intensity(warp, out_range = (0, 255))
        (ret, thresh) = cv2.threshold(warp, 127, 255, cv2.THRESH_BINARY)
        #cv2.imshow("threshold", thresh)
        if not thresh is None:
            for key, value in ref_images.items():
                diff_frame = cv2.bitwise_xor(thresh, value)
                cv2.imshow("diff", diff_frame)
                nzCount = cv2.countNonZero(diff_frame)
                if nzCount < 30000:
                    cv2.putText(frame, key, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 255), 3)

    #print(mouseX, mouseY)
    #pgui.moveTo(mouseX, mouseY, 0.0)

    #show our frame
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", colorMask)
    cv2.imshow("Edges", edges)

    key = cv2.waitKey(1) & 0xFF
    counter += 1

    #if 'q' is pressed, stop the loop
    if key == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
