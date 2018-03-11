'''
*	File: LineFollowUtils.py
*	Description:	Utility functions/classes for line following. Provided class
*       is LineImageSlice.
*   Code inspired by (and some functions copied from) https://github.com/CRM-UAM/VisionRace
*	Author(s):		Austin Dibble
*	Date Created:	12/27/17
'''
import numpy as np
import cv2
from collections import deque


def centerOfContour(moments):
    if moments["m00"] == 0:
        return 0

    x = int(moments["m10"]/moments["m00"])
    y = int(moments["m01"]/moments["m00"])

    return x, y

def sliceImage(image, imageParts, numSlices, renderText = True, rawImage = None):
    height, width = image.shape[:2]
    sliceHeight = int(height / numSlices);

    for i in range(numSlices):
        part = sliceHeight * i
        crop_img = image[part:part+sliceHeight, 0:width]
        crop_raw = rawImage[part:part+sliceHeight, 0:width]
        imageParts[i].image = crop_img
        imageParts[i].process(renderText, crop_raw)
        imageParts[i].absoluteY = imageParts[i].middleY + part

def repackImages(images):
    img = images[0].image
    for i in range(len(images)):
        if i == 0:
            img = np.concatenate((img, images[1].image), axis=0)
        if i > 1:
            img = np.concatenate((img, images[i].image), axis=0)

    return img

def repackRawImages(images):
    img = images[0].rawImage
    for i in range(len(images)):
        if i == 0:
            img = np.concatenate((img, images[1].rawImage), axis=0)
        if i > 1:
            img = np.concatenate((img, images[i].rawImage), axis=0)

    return img


'''
*	Class: BaseFinder
*	Description: Class for operating on images, and searching for our base marker
*       The RoboCV interface allows access to the data acquired by this class
*	Author(s):		Austin Dibble
*	Date Created:	3/1/18
'''
class LineImageSlice:
    def __init__(self):
        self.image = None
        self.rawImage = None
        self.contourCenterX = 0
        self.contourCenterY = 0
        self.middleY = 0
        self.absoluteY = 0
        self.mainContour = None
        self.dir = 0
        self.isLine = False

    def process(self, renderText = True, rawImage = None):
        #imgray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY) #Convert to Gray Scale
        #ret, thresh = cv2.threshold(imgray,100,255,cv2.THRESH_BINARY_INV) #Get Threshold
        self.rawImage = rawImage
        _, self.contours, _ = cv2.findContours(self.image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Get contour

        self.prev_MC = self.mainContour
        if self.contours:
            self.mainContour = max(self.contours, key=cv2.contourArea)

            self.height, self.width  = self.image.shape[:2]

            self.middleX = int(self.width / 2) #Get X coordenate of the middle point
            self.middleY = int(self.height / 2) #Get Y coordenate of the middle point

            self.prev_cX = self.contourCenterX

            contourCenter = self.getContourCenter(self.mainContour)
            if contourCenter != 0:
                self.contourCenterX = contourCenter[0]
                self.contourCenterY = contourCenter[1]
                if abs(self.prev_cX - self.contourCenterX) > 5:
                    self.correctMainContour(self.prev_cX)
            else:
                self.contourCenterX = 0

            #Get size ratio
            extent = self.getContourExtent(self.mainContour)

            #Give a direction value, based on the pixels time the size ratio
            self.dir =  (self.middleX - self.contourCenterX) * extent

            '''
            if extent < 0.74: #Exclude sections with holes?
                self.isLine = False
                return
            '''

            #self.dir =  int((self.middleX - self.contourCenterX))

            #Render information to the screen
            if renderText and rawImage is not None:
                cv2.drawContours(rawImage, self.mainContour, -1, (0,255,0), 3) #Draw Contour GREEN
                cv2.circle(rawImage, (self.contourCenterX, self.middleY), 7, (0,255,255), -1) #Draw dX circle WHITE
                cv2.circle(rawImage, (self.middleX, self.middleY), 3, (0,0,255), -1) #Draw middle circle RED

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(rawImage, str(self.middleX-self.contourCenterX), (self.contourCenterX+20, self.middleY), font, 1, (200,0,200), 2, cv2.LINE_AA)
                cv2.putText(rawImage, "Weight:%.3f" % self.getContourExtent(self.mainContour), (self.contourCenterX + 20, self.middleY + 20), font, 0.5, (200,0,200), 1, cv2.LINE_AA)

            self.isLine = True
        else:
            self.isLine = False

    def isLinePresent(self):
        return self.isLine

    def getContourCenter(self, contour):
        M = cv2.moments(contour)

        if M["m00"] == 0:
            return 0

        x = int(M["m10"]/M["m00"])
        y = int(M["m01"]/M["m00"])

        return [x,y]

    def getContourExtent(self, contour):
        area = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour)
        rect_area = w * h
        if rect_area > 0:
            return (float(area) / rect_area)

    def approx(self, a, b, error):
        if abs(a - b) < error:
            return True
        else:
            return False

    def correctMainContour(self, prev_cx):
        if abs(prev_cx - self.contourCenterX) > 5:
            for i in range(len(self.contours)):
                if self.getContourCenter(self.contours[i]) != 0:
                    tmp_cx = self.getContourCenter(self.contours[i])[0]
                    if self.approx(tmp_cx, prev_cx, 5) == True:
                        self.mainContour = self.contours[i]
                        if self.getContourCenter(self.mainContour) != 0:
                            self.contourCenterX = self.getContourCenter(self.mainContour)[0]
