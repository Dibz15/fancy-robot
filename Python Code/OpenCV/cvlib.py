from collections import deque
import numpy as np
import argparse
import imutils
import cv2

#this edit....

def nothing(x):
    pass

'''
****************************************************
*Function: Binarize
*Description: Recieves an image frame (numpy matrix)
*   and two HSV threshold values triplets(upper and lower)
*   and creates a binary image. White is within the range,
*   black is not.
*Parameters:
*   frame - numpy matrix image
*   lowerHSV - 3-value tuple of HSV values for lower limit
*   upperHSV - Same as lowerHSV, but the upper limit
*Returns:
*   modified black and white threshold frame (numpy matrix)
****************************************************
'''
def binarize(frame, lowerHSV, upperHSV):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #construct mask for the color then perform
    #dilations and erosions to remove fragment
    binarized = cv2.inRange(hsv, lowerHSV, upperHSV)

    binarized = cv2.erode(binarized, None, iterations=2)
    binarized = cv2.dilate(binarized, None, iterations=2)

    return binarized

#Find area of certain color
'''
****************************************************
*Function: colorFinder
*Description: Recieves an image frame (numpy matrix)
*  and finds the area of the frame with the largest amount
*  of the specified HSV values (given by the pop-up value sliders)
*Parameters:
*   frame - numpy matrix image
*   avgX - average x value (placeholder instead of returning the value)
*   avgY - average y value
*   pts - deque of points for location memory
*Returns:
*   modified black and white threshold frame (numpy matrix) of the given image
****************************************************
'''
def colorFinder(frame, avgX, avgY, pts):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = (cv2.getTrackbarPos('H_MIN', 'window'),
            cv2.getTrackbarPos('S_MIN', 'window'),
            cv2.getTrackbarPos('V_MIN', 'window'))

    upper = (cv2.getTrackbarPos('H_MAX', 'window'),
            cv2.getTrackbarPos('S_MAX', 'window'),
            cv2.getTrackbarPos('V_MAX', 'window'))

    #construct mask for the color then perform
    #dilations and erosions to remove fragment
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        #find the largest contour
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 20:
            #draw the circle
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255, -1))
            pts.appendleft(center)

    if len(pts) >= pts.maxlen - 1 and pts[-10] is not None:
        avgX = int((pts[-10][0] + pts[0][0] + pts[5][0]) / 3)
        avgY = int((pts[-10][1] + pts[0][1] + pts[5][1]) / 3)

    cv2.putText(frame, "X:{}, Y:{}".format(avgX, avgY), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)

    return mask

#Image edge detection
'''
****************************************************
*Function: colorFinder
*Description: Recieves an grayscale numpy matrix, and returns the
*   edge-detected modified frame
*Parameters:
*   gray - grayscale frame
*   sigma - constant for calculating edge-detection
*Returns:
*   modified edge-detected image
****************************************************
'''
def auto_canny(gray, sigma=0.33):
    #compute median of single channel pixel intensities
    v = np.median(gray)

    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(gray, lower, upper)

    #return the edged image
    return edged


'''
****************************************************
*Function: get_rect
*Description: recieves a numpy matrix image, and finds the largest quadrilateral
*   shape in the image, returning the cv2 contour
*Parameters:
*   image - image to search in
*Returns:
*   None, or the contour of the rectange found
****************************************************
'''
#Find the largest rectangle contour in the image
def get_rect(image):
    cnts = cv2.findContours(image.copy(), cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]
    center = None

    screenCnt = None

    if len(cnts) > 0:
        #Find the rectangle
        for c in cnts:
            #approximate the contours
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            #If our approximated contour has 4 pts, then it's a rectangle
            if len(approx) == 4:
                screenCnt = approx

                return screenCnt

    return None

'''
****************************************************
*Function: get_rect_via_ratio
*Description: recieves a numpy matrix image, and finds the largest quadrilateral
*   shape in the image, using an area approximation instead of polygon decomp
*Parameters:
*   image - image to search in
*   ratio - ratio of area to find
*Returns:
*   None, or the contour of the rectange found
****************************************************
'''
def get_rect_via_ratio(image, ratio):
    #Find contours in our image
    cnts = cv2.findContours(image.copy(), cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:2]
    center = None

    screenCnt = None

    if len(cnts) > 0:
        #Find the rectangle
        for contour in cnts:
            #approximate the contours
            area = cv2.contourArea(contour)
            x,y,w,h = cv2.boundingRect(contour)
            rect_area = w * h
            if rect_area > 0:
                area_ratio = (float(area) / rect_area)
                if area_ratio >= ratio:
                    return contour

    return None

'''
****************************************************
*Function: getContourCenter
*Description: get the center coordinate pair of the given contour
*Parameters:
*   contour - contour to find the center of
*Returns:
*   array coordinate pair
****************************************************
'''
def getContourCenter(contour):
    M = cv2.moments(contour)

    if M["m00"] == 0:
        return [0, 0]

    x = int(M["m10"]/M["m00"])
    y = int(M["m01"]/M["m00"])

    return [ x, y ]

'''
****************************************************
*Function: getContourExtent
*Description: 
*Parameters:
*   image - image to search in
*   ratio - ratio of area to find
*Returns:
*   None, or the contour of the rectange found
****************************************************
'''
def getContourExtent(contour):
    area = cv2.contourArea(contour)
    x,y,w,h = cv2.boundingRect(contour)
    rect_area = w * h
    if rect_area > 0:
        return (float(area) / rect_area)

'''
****************************************************
*Function: warp_perspective
*Description: Recieves an image with a quadrilateral contour in it,
*   and returns a rectangle (ie. it warps the given quadrilateral into
*   a rectangle).
*Parameters:
*   image - numpy matrix image with quad
*   screenCnt - contour that describes the quad in the image
*   ratio - image size ratio
*   finalWidth - specified final width of image
*   finalHeight - specified final height of image
*Returns:
*   Image of perspective-corrected rectangle
****************************************************
'''
#Use matrix math to convert a warped rectangle to one that is seen straight
def warp_perspective(image, screenCnt, ratio, finalWidth = -1, finalHeight = -1):
    if screenCnt is not None:
        #cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3)
        #now to identify corners in order to reshape image
        pts = screenCnt.reshape(4, 2)
        rect = np.zeros((4, 2), dtype = "float32")

        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)] #top-left point
        rect[2] = pts[np.argmax(s)] #top-right point

        #compute difference between points
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        #scale the rectangle to the original image (not scaled down one)
        rect *= ratio
        (maxWidth, maxHeight) = (0, 0)

        if finalWidth == -1 and finalHeight == -1:
            #get the four corners of the rectangle
            (tl, tr, br, bl) = rect

            #Get the distance between the bottom corners
            widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))

            #Get the distance between the top corners
            widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((br[1] - bl[1]) ** 2))

            #Get the height of the right side
            heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))

            #Get the height of the left side
            heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))

            #Get the largest height and width bounds
            maxWidth = max(int(widthA), int(widthB))
            maxHeight = max(int(heightA), int(heightB))
        else:
            maxWidth = finalWidth
            maxHeight = finalHeight

        #construct destination points used to map the screen to a top-down view
        dst = np.array([
            [0, 0],      #top-left
            [maxWidth - 1, 0],  #top-right
            [maxWidth - 1, maxHeight - 1],  #bottom-right
            [0, maxHeight - 1]], dtype = "float32") #bottom-left

        #calculate perspective transform matrix and warp perspective
        M = cv2.getPerspectiveTransform(rect, dst)
        warp = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

        return warp
    return None
