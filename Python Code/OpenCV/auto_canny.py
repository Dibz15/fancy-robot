#import packages
import numpy as np
import argparse
import glob
import cv2
import imutils

def auto_canny(image, sigma=0.33):
    #compute median of single channel pixel intensities
    v = np.median(image)

    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    #return the edged image
    return edged

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help = "path to input image")
args = vars(ap.parse_args())

for imagePath in glob.glob(args["image"]):
    # load the image, convert it to grayscale, and blur it slightly
    image = cv2.imread(imagePath)
    image = imutils.resize(image, width=600, height=400)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    # apply Canny edge detection using a wide threshold, tight
    # threshold, and automatically determined threshold
    wide = cv2.Canny(blurred, 10, 200)
    tight = cv2.Canny(blurred, 225, 250)
    auto = auto_canny(blurred)

    # show the images
    cv2.imshow("Original", image)
    cv2.imshow("Edges", np.hstack([wide, tight, auto]))
    cv2.waitKey(0)
