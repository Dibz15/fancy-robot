'''
*	File: RoboCV.py
*	Description:	Main module for the CV functions of the robot. Holds the class that initializes
*       the camera stream, and cycles through and processes images.
*	Author(s):		Austin Dibble
*	Date Created:	12/27/17
'''

from PiCameraStream import *
from cvlib import *
from LineFollowUtils import *
from imutils import *
import math


'''
*	Class: RoboCV
*	Description: Class for CV functions and control.
*	Author(s):		Austin Dibble
*	Date Created:	12/27/18
'''
class RoboCV:
    def __init__(self) :
        self.cameraStream = PiCameraStream()
        self.stopped = True
        self.currentVisionFunction = None
        self.visionFunctions = {}
        self.visionFunctions["LineFollower"] = LineFollower()
        self.visionFunctions["HSVFinder"] = HSVFinder()
        self.visionFunctions["BaseFinder"] = BaseFinder()
        self.visionFunctions["DistanceCalibrator"] = DistanceCalibrator()
        self.currFrame = None

    #Name of this class
    def __str__ ( self ) :
        return "RoboCV: \n\t" + self.currentVisionFunction.__str__() + "\n\t" + self.cameraStream.__str__()

    #Start our camera stream, and our image processing thread.
    def start ( self ) :
        print("Starting RoboCV thread")
        self.stopped = False

        #Camera stream
        self.cameraStream.start()
        #New thread
        Thread(target = self.update, args=()).start()

        return self

    #Use this to stop threads
    def stop ( self ) :
        print("Stopping RoboCV thread")
        self.stopped = True

        #Stop our camera stream, and clean up
        self.cameraStream.stop()

    #This is called by the processing thread
    def update ( self ) :
        #While our thread is running
        while not self.stopped:
            #Make sure we have a vision function to work on
            if self.currentVisionFunction is not None:
                #Read the current camera frame
                self.currFrame = self.cameraStream.read()
                #Operate on the frmae
                self.currentVisionFunction.operate(self.currFrame)
        return

    def setIfRenderText( self , renderText ) :
        self.currentVisionFunction.renderText = renderText

    def getPiCameraStream( self ) :
        return self.cameraStream

    def getUnmodifiedFrame(self) :
        return self.currFrame

    def getCurrentVisionFunction (self):
        return self.currentVisionFunction

    #Change the vision function via a string
    def setCurrentVisionFunction (self, visionFunctionString):
        #Get our function
        newFunction = self.visionFunctions[visionFunctionString]
        #Make sure it wasn't a bad string
        if newFunction is not None:
            self.currentVisionFunction = newFunction
        else:
            print("Error in RoboCV: Given visionFunctionString is not known")

    def getCurrentVisionFunctionValues( self ):
        return self.currentVisionFunction.getValues()

    def getCurrentVisionFunctionValue( self, valueName ):
        return self.currentVisionFunction.getValue( valueName )

'''
*	Class: VisionFunction
*	Description: Abstract class for operating on images
*	Author(s):		Austin Dibble
*	Date Created:	12/28/17
'''
class VisionFunction:
    def __init__(self) :
        self.values = {}
        self.values["name"] = "VisionFunction"
        self.renderText = False
        return

    '''
    ****************************************************
    *Function: operate
    *Description: performs some vision function operation
    *Params: rawFrame ; frame to operate on
    *Returns:
    *   Nothing
    ****************************************************
    '''
    def operate( self, rawFrame ):
        return

    def getValues ( self ) :
        return self.values

    def getValue ( self, valueName ) :
        return self.values.get(valueName)

'''
*	Class: LineFollower
*	Description: Class for operating on images, and searching for the line.
*       The RoboCV interface allows access to the data acquired by this class
*	Author(s):		Austin Dibble
*	Date Created:	12/28/17
'''
class LineFollower(VisionFunction):
    def __init__(self, numSlices = 5):
        VisionFunction.__init__(self)
        self.values["name"] = "LineFollower"
        self.values["lineImage"] = None
        self.values["resizedImage"] = None
        self.values["missingContours"] = numSlices
        self.values["direction"] = 0
        self.values["directionVector"] = [(0, 0), (0, 0)]
        self.values["angle"] = 90
        self.values["lineCoords"] = [(0, 0)] * numSlices

        #Our green color HSV values we're looking for
        self.minGreen = (47, 140, 60) #Perhaps this will find green?
        self.maxGreen = (70, 255, 255)

        self.minOrange = (0, 179, 105) #Perhaps this will find green?
        self.maxOrange = (20, 255, 225)

        #Array of the image parts of the current frame
        self.imageParts = []

        #Add our image slice objects to the array
        for i in range(numSlices):
            self.imageParts.append(LineImageSlice())

        self.numSlices = numSlices
        self.direction = 0

        #Gives the upper and lower coordinates of the line extents, for angle calculation
        self.directionVector = [(0, 0), (0, 0)]
        self.lineCoords = [(0, 0)] * self.numSlices
        return

    def operate( self, rawFrame ) :
        time.sleep(1.0 / 15.0)
        self.direction = 0

        #Resize our video stream for performance
        #rawResized = imutils.resize(rawFrame, height = 400)

        r = 400.0 / rawFrame.shape[1]
        dim = (400, int(rawFrame.shape[0] * r))

        # perform the actual resizing of the image and show it
        rawResized = cv2.resize(rawFrame, dim, interpolation = cv2.INTER_AREA)


        #Convert it to a binary image based on the color thresholds
        binarized = binarize(rawResized, self.minGreen, self.maxGreen)
        #binarized = binarize(rawResized, self.minOrange, self.maxOrange)

        #Slice up the image and calculate the direction center for each piece
        sliceImage(binarized, self.imageParts, self.numSlices, self.renderText, rawResized)

        #Add up the direction sum and check for missing contours
        numMissing = 0
        lowestCoord = (0, 10000)
        highestCoord = (0, 0)

        #Go through every line slice
        for i in range(self.numSlices):
            #Make sure this part of the image has our color...
            if self.imageParts[i].isLinePresent():
                #Get a weighted direction from the slice (closer is more weight)
                modDir = (self.imageParts[i].dir * ( 1 + float(i) / self.numSlices))

                #Add it to our direction sum
                self.direction += modDir

                #Get the coordinates for all line slices
                self.lineCoords[i] = (self.imageParts[i].contourCenterX, self.imageParts[i].absoluteY)

                #Make sure we don't have a divide by 0...
                if self.imageParts[i].absoluteY != 0:

                    if self.imageParts[i].absoluteY < lowestCoord[1]:
                        #Get our lowest coordinate pair
                        lowestCoord = (self.imageParts[i].contourCenterX, self.imageParts[i].absoluteY)
                    elif self.imageParts[i].absoluteY > highestCoord[1]:
                        #Get our highest coordinate  pair
                        highestCoord = (self.imageParts[i].contourCenterX, self.imageParts[i].absoluteY)

                #print("Part [" + str(i) + "]: " + str(modDir))
                continue
            else:
                #print("Part[" + str(i) + "]: No line contour")
                numMissing += 1

        #If we're missing all pieces, direction = 0. This also avoids a divide by 0
        if (self.numSlices - numMissing) == 0:
            self.direction = 0
        #Otherwise, calculate our direction
        else:
            self.direction = self.direction / (self.numSlices - numMissing)

        self.directionVector[0] = lowestCoord
        self.directionVector[1] = highestCoord

        #get our deltas for angle calculation
        deltaX = self.directionVector[0][0] - self.directionVector[1][0]
        deltaY = self.directionVector[1][1] - self.directionVector[0][1]

        #Avoid divide by 0
        if deltaX == 0:
            angle = 90.0
        #Calculate the angle
        else:
            angle = math.degrees(math.atan( float(deltaY) / float(deltaX) ))
            #Make sure it's between 0-180
            if angle < 0:
                angle += 180

        #Piece the final image pack together for display from the pieces
        finalImage = repackImages(self.imageParts)
        rawResized = repackRawImages(self.imageParts)

        #Render the direction sum
        if self.renderText:
            cv2.putText(rawResized, "Dir: " + str(self.direction), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
            cv2.putText(rawResized, "Angle: " + str(angle), (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 200), 2, cv2.LINE_AA)
            cv2.line(rawResized, self.directionVector[0], self.directionVector[1], (255,0,0), 2)

        #Put our values into the dictionary in case we need something
        self.values["lineImage"] = finalImage
        self.values["modified"] = rawResized
        self.values["missingContours"] = numMissing
        self.values["direction"] = self.direction
        self.values["directionVector"] = self.directionVector
        self.values["angle"] = angle
        self.values["lineCoords"] = self.lineCoords

        return

    def __str__( self ) :
        return "LineFollower"


'''
*	Class: HSVFinder
*	Description: Class for operating on images, and finding objects of a certain color.
*	Author(s):		Austin Dibble
*	Date Created:	12/28/17
'''
class HSVFinder(VisionFunction):
    def __init__(self):
        VisionFunction.__init__(self)
        self.values["name"] = "HSVFinder"
        #Create trackbars
        self.count = 0

        self.pts = deque(maxlen=16)

        return

    def operate( self, rawFrame ) :

        #Create our value sliders
        if (self.count == 0):
            cv2.namedWindow('window')

            #Set up our trackbar window
            cv2.createTrackbar('H_MAX', 'window', 0, 180, nothing) #H
            cv2.createTrackbar('H_MIN', 'window', 0, 180, nothing)

            cv2.createTrackbar('S_MAX', 'window', 0, 255, nothing) #S
            cv2.createTrackbar('S_MIN', 'window', 0, 255, nothing)

            cv2.createTrackbar('V_MAX', 'window', 0, 255, nothing) #V
            cv2.createTrackbar('V_MIN', 'window', 0, 255, nothing)
            time.sleep(1)

        #20 fps
        time.sleep( 1.0 / 15.0 )
        #resize frame for efficiency
        #rawResized = imutils.resize(rawFrame, height = 400)
        #print("Looping hsvFinder")

        r = 400.0 / rawFrame.shape[1]
        dim = (400, int(rawFrame.shape[0] * r))

        # perform the actual resizing of the image and show it
        rawResized = cv2.resize(rawFrame, dim, interpolation = cv2.INTER_AREA)
        self.values["resized"] = rawResized

        #Find the color :D
        colorMask = colorFinder(rawResized, 0, 0, self.pts)

        #Let's return the mask in case we need it
        self.values["binarized"] = colorMask

        self.count += 1
        return

    def __str__(self):
        return "HSVFinder"

'''
*	Class: BaseFinder
*	Description: Class for operating on images, and searching for our base marker
*       The RoboCV interface allows access to the data acquired by this class
*	Author(s):		Austin Dibble
*	Date Created:	3/1/18
'''
class BaseFinder(VisionFunction):
    def __init__(self):
        VisionFunction.__init__(self)
        self.values["name"] = "BaseFinder"
        self.values["binarizedOrange"] = None
        self.values["binarizedGreen"] = None
        self.values["resized"] = None
        #self.values["edges"] = None
        self.values["direction"] = 0
        self.values["angle"] = 90
        self.values["coords"] = [0, 0]
        self.values["lineAbsent"] = True
        self.values["distance"] = -1.0
        self.values["rectArea"] = 0
        self.values["numGreen"] = 0
        self.values["greenCoords"] = [[0, 0], [0, 0]]
        self.values["greenDirections"] = [0, 0]
        self.values["greenAreas"] = [0, 0]
        self.values["greenWidths"] = [0, 0]

        self.lineAbsent = True

        #Our green color HSV values we're looking for
        self.minOrange = (0, 179, 75) #Perhaps this will find green?
        self.maxOrange = (20, 255, 230)

        self.minGreen = (47, 140, 50) #Perhaps this will find green?
        self.maxGreen = (70, 255, 255)

        self.numGreen = 0

        #Direction for orange rectangle
        self.orangeDirection = 0
        self.greenDirections = [0, 0]
        self.greenAreas = [0, 0]
        self.greenWidths = [0, 0]
        self.distance = -1.0
        self.edgeDistance = 0

        #Gives the coordinate of the rectangle, if found
        self.coords = [0, 0]
        self.greenCoords = [[0, 0], [0, 0]]

        self.focalLength = 381.0 #in cm
        self.KNOWN_DISTANCE = 15 #in cm
        #self.KNOWN_WIDTH = 4.73 #in cm
        self.KNOWN_WIDTH = 7.3 #in cm

        #Averages: orangeDirection average, area average, distance average
        self.averageQueue = (deque(maxlen = 5), deque(maxlen = 5), deque(maxlen = 5), deque(maxlen = 5))
        self.averages = [0, 0, 0]

        self.values["averageQueue"] = self.averageQueue
        self.values["averages"] = self.averages

        return

    '''
    ****************************************************
    *Function: distance_to_camera
    *Description: Gives the distance of the object from the camera
    *Params: knownWidth, focalLength, perWidth
    *Returns:
    *   Distance in cm
    ****************************************************
    '''
    def distance_to_camera(self, knownWidth, focalLength, perWidth):
    	# compute and return the distance from the maker to the camera
    	return (knownWidth * focalLength) / perWidth

    '''
    ****************************************************
    *Function: standard_deviation
    *Description: Find the standard deviation of the dataset
    *Params: list - list of numbers to find the std of
    *Returns:
    *    the standard deviation
    ****************************************************
    '''
    def standard_deviation(self, lst):
        num_items = len(lst)
        mean = sum(lst) / num_items
        differences = [x - mean for x in lst]
        sq_differences = [d **2 for d in differences]
        ssd = sum(sq_differences)

        variance = ssd / (num_items - 1)
        sd = math.sqrt(variance)
        return sd

    '''
    ****************************************************
    *Function: isOutlier
    *Description: Finds whether a number is an outlier of a dataset
    *Params: value - value to check
    *        mean - average value of dataset
    *        standardDeviation - the standard deviation of the dataset
    *Returns:
    *    bool, whether or not the value is an outlier
    ****************************************************
    '''
    def isOutlier(self, value, mean, standardDeviation):
        return (abs(value) > (mean + (2 * standardDeviation)))

    '''
    ****************************************************
    *Function: detectRectangle
    *Description: Finds the orange rectangle data
    *Params: rect_area - area of rectangle found
            baseRectCont - rectangle contour
            width - width of the screen
    *Returns:
    *    nothing
    ****************************************************
    '''
    def detectRectangle(self, rect_area, baseRectCont, width):
        self.lineAbsent = True
        if baseRectCont is not None and rect_area > 600:
            self.lineAbsent = False

            #print("AR: " + str(area_ratio) + ", " + str(rect_area) + ", " + str(self.averages[1]))

            #Rectangle area averaging
            if len(self.averageQueue[1]) > 1:
                self.averages[1] = sum(self.averageQueue[1]) / len(self.averageQueue[1])
                #Get standard dev
                a_sd = self.standard_deviation(self.averageQueue[1])

                #if not outlier, add it to data set
                if not self.isOutlier(rect_area, self.averages[1], a_sd):
                    self.averageQueue[1].appendleft(rect_area)
            else:
                self.averageQueue[1].appendleft(rect_area)

            #Lets calculate the center coordinates
            self.coords = getContourCenter(baseRectCont)

            #Calculate orangeDirection from coordinates
            self.orangeDirection = (width / 2.0) - self.coords[0]

            #Direction averaging
            if len(self.averageQueue[0]) > 1:
                self.averages[0] = sum(self.averageQueue[0]) / len(self.averageQueue[0])
                #Calc standard dev
                dir_sd = self.standard_deviation(self.averageQueue[0])

                #If data is not outlier, add to dataset
                if not self.isOutlier(self.orangeDirection, self.averages[0], dir_sd):
                    self.averageQueue[0].appendleft(self.orangeDirection)
            else:
                self.averageQueue[0].appendleft(self.orangeDirection)

            #Get rectangle to calc distance
            minA = cv2.minAreaRect(baseRectCont)
            self.distance = self.distance_to_camera(self.KNOWN_WIDTH, self.focalLength, minA[1][0])
            #print("Dist: " + str(self.distance) + ", " + str(self.averages[2]))

            #Distance averaging
            if len(self.averageQueue[2]) > 1:
                self.averages[2] = sum(self.averageQueue[2]) / len(self.averageQueue[2])
                #Calculate standard dev
                dist_sd = self.standard_deviation(self.averageQueue[2])

                #if data is not an outlier, save it
                if not self.isOutlier(self.distance, self.averages[2], dist_sd):
                    self.averageQueue[2].appendleft(self.distance)
            else:
                self.averageQueue[2].appendleft(self.distance)

        #If we have 5 "lineAbsent" values in our queue
        if ( len(self.averageQueue[3]) >= 5 ):
            self.averageQueue[3].appendleft(self.lineAbsent)
            s = sum(self.averageQueue[3])
            if s > (0.2 * len(self.averageQueue[3])):
                #print("Bad average")
                #Line absent is true only if 4 of 5 of our queue is true
                self.lineAbsent = True
        else:
            self.averageQueue[3].appendleft(self.lineAbsent)

    '''
    ****************************************************
    *Function: detectGreen
    *Description: Finds the green!
    *Params: binarizedGreen - pixel array of green binary contours
            width - width of the screen
            rawResized - raw resized pixel array
    *Returns:
    *    bool, whether or not the value is an outlier
    ****************************************************
    '''
    def detectGreen(self, binarizedGreen, width, rawResized):
        (_, cnts, _) = cv2.findContours(binarizedGreen.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:2]

        greenConts = [None, None]
        self.greenAreas = [0, 0]
        self.numGreen = 0
        if len(cnts) > 0:
            #Find the rectangle
            for contour in cnts:
                #approximate the contours
                area = cv2.contourArea(contour)
                x,y,w,h = cv2.boundingRect(contour)
                rect_area = w * h
                if rect_area > 0:
                    area_ratio = (float(area) / rect_area)
                    #Get the ratio of the contour area to the bounding rectangle
                    if area_ratio >= 0.60 and w >= 5 and rect_area > 50:
                        greenConts[self.numGreen] = contour
                        self.greenCoords[self.numGreen] = getContourCenter(contour)
                        #Calculate orangeDirection from coordinates
                        self.greenDirections[self.numGreen] = (width / 2.0) - self.greenCoords[self.numGreen][0]
                        self.greenAreas[self.numGreen] = rect_area
                        self.greenWidths[self.numGreen] = w
                        self.numGreen += 1

        return greenConts


    def operate( self, rawFrame ) :
        time.sleep(1.0 / 15.0)
        self.orangeDirection = 0

        #Resize our video stream for performance
        #rawResized = imutils.resize(rawFrame, height = 400)

        r = 400.0 / rawFrame.shape[1]
        dim = (400, int(rawFrame.shape[0] * r))

        # perform the actual resizing of the image and show it
        rawResized = cv2.resize(rawFrame, dim, interpolation = cv2.INTER_AREA)
        height, width = rawResized.shape[:2]

        #Convert it to a binary image based on the color thresholds
        binarizedOrange = binarize(rawResized, self.minOrange, self.maxOrange)
        binarizedGreen = binarize(rawResized, self.minGreen, self.maxGreen)
        #detect rectangle contours

        area_ratio = 0.1
        rect_area = 0
        rectWidth = 0
        baseRectCont = None
        cnts = cv2.findContours(binarizedOrange, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:2]

        if len(cnts) > 0:
            #Find the rectangle
            for contour in cnts:
                #approximate the contours
                area = cv2.contourArea(contour)
                x,y,w,h = cv2.boundingRect(contour)

                rect_area = w * h
                if rect_area > 0:
                    area_ratio = (float(area) / rect_area)
                    if area_ratio >= 0.80:
                        rectWidth = w
                        baseRectCont = contour
                        break

        #Did we get a rectangle?
        self.detectRectangle(rect_area, baseRectCont, width)

        greenConts = self.detectGreen(binarizedGreen, width, rawResized)

        #If we see only 1 green, and also the rectangle
        if self.numGreen == 1 and not self.lineAbsent:
            #If the green is to the left of the rectangle
            if (self.greenDirections[0] > self.orangeDirection):
                #Calculate the distance between the green's right edge, and the
                #orange's left edge
                greenEdge = self.greenDirections[0] - (self.greenWidths[0] / 2)
                orangeEdge = self.orangeDirection + (rectWidth / 2)
                self.edgeDistance = greenEdge - orangeEdge
            else:
                #Else if the green is to the right of the orange
                #Calculate the distance between the green's left edge and the
                #orange's right edge
                greenEdge = self.greenDirections[0] + (self.greenWidths[0] / 2)
                orangeEdge = self.orangeDirection - (rectWidth / 2)
                self.edgeDistance = orangeEdge - greenEdge

        if self.renderText:
            #If we want to render data to the screen
            if not self.lineAbsent:
                cv2.putText(rawResized, "ODir: " + str(self.orangeDirection), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
                cv2.putText(rawResized, "ODist: " + str(self.distance) + " cm", (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
                cv2.putText(rawResized, "OArea: " + str(rect_area) + " cm", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
                if self.numGreen == 1:
                    cv2.putText(rawResized, "ED: " + str(self.edgeDistance) + "px", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)

                #Draw the orange rectangle
                cv2.drawContours(rawResized, [baseRectCont], -1, 255, 3)

            if self.numGreen >= 1:
                cv2.putText(rawResized, "GDir: " + str(self.greenDirections), (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
                cv2.putText(rawResized, "GNum" + str(self.numGreen) + "", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
                cv2.putText(rawResized, "GW: " + str(self.greenWidths), (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)

                if self.numGreen == 1:
                    #Draw just the one contour
                    cv2.drawContours(rawResized, [greenConts[0]], -1, 255, 3)
                    pass
                else:
                    #Draw both contours
                    cv2.drawContours(rawResized, greenConts, -1, 255, 3)
                    pass
        #Put our values into the dictionary in case we need something
        self.values["binarizedOrange"] = binarizedOrange
        self.values["binarizedGreen"] = binarizedGreen
        self.values["resized"] = rawResized
        #self.values["edges"] = edges
        self.values["orangeDirection"] = self.orangeDirection
        self.values["angle"] = 90
        self.values["coords"] = self.coords
        self.values["lineAbsent"] = self.lineAbsent
        self.values["distance"] = self.distance
        self.values["averageQueue"] = self.averageQueue
        self.values["averages"] = self.averages
        self.values["rectArea"] = rect_area

        self.values["numGreen"] = self.numGreen
        self.values["greenCoords"] = self.greenCoords
        self.values["greenDirections"] = self.greenDirections
        self.values["greenAreas"] = self.greenAreas
        self.values["greenWidths"] = self.greenWidths
        self.values["edgeDistance"] = self.edgeDistance

        return

    def __str__( self ) :
        return "BaseFinder"


'''
*	Class: LineFollower
*	Description: Class for operating on images, and searching for the line.
*       The RoboCV interface allows access to the data acquired by this class
*	Author(s):		Austin Dibble
*	Date Created:	3/1/2018
'''
class DistanceCalibrator(VisionFunction):
    def __init__(self):
        VisionFunction.__init__(self)
        self.values["name"] = "DistanceCalibrator"
        self.values["binarized"] = None
        self.values["resized"] = None
        self.values["edges"] = None
        self.values["focalLength"] = None

        #Our green color HSV values we're looking for
        self.minOrange = (0, 179, 105) #Perhaps this will find green?
        self.maxOrange = (20, 255, 225)

        #Render the overlay?
        self.renderText = True

        #Gives the coordinate of the rectangle, if found
        self.coords = [0, 0]
        self.focalLength = 0.0 # in cm

        self.KNOWN_DISTANCE = 15 #in cm
        self.KNOWN_WIDTH = 4.73 #in cm
        return

    def operate( self, rawFrame ) :
        time.sleep(1.0 / 15.0)

        #Resize our video stream for performance
        #rawResized = imutils.resize(rawFrame, height = 400)

        r = 400.0 / rawFrame.shape[1]
        dim = (400, int(rawFrame.shape[0] * r))

        # perform the actual resizing of the image and show it
        rawResized = cv2.resize(rawFrame, dim, interpolation = cv2.INTER_AREA)
        height, width = rawResized.shape[:2]

        #Convert it to a binary image based on the color thresholds
        binarized = binarize(rawResized, self.minOrange, self.maxOrange)
        #detect rectangle contours
        edges = auto_canny(binarized)

        baseRectCont = get_rect(edges)

        #Did we get a rectangle?
        lineAbsent = True
        if baseRectCont is not None:
            #If we did, calculate center coordinates.
            self.coords = getContourCenter(baseRectCont)
            minA = cv2.minAreaRect(baseRectCont)
            self.focalLength = (minA[1][0] * self.KNOWN_DISTANCE) / self.KNOWN_WIDTH
            #Calculate direction from coordinates

            if self.renderText:
                cv2.putText(rawResized, "FL: " + str(self.focalLength), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,200), 2, cv2.LINE_AA)
                #cv2.putText(rawResized, "Angle: " + str(angle), (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 200), 2, cv2.LINE_AA)
                #cv2.line(rawResized, self.directionVector[0], self.directionVector[1], (255,0,0), 2)

                cv2.drawContours(rawResized, [baseRectCont], -1, 255, 3)
                pass


        #Put our values into the dictionary in case we need something
        self.values["binarized"] = binarized
        self.values["resized"] = rawResized
        self.values["edges"] = edges
        self.values["focalLength"] = self.focalLength

        return

    def distance_to_camera(knownWidth, focalLength, perWidth):
    	# compute and return the distance from the maker to the camera
    	return (knownWidth * focalLength) / perWidth

    def __str__( self ) :
        return "DistanceCalibrator"
