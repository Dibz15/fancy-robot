from PiCameraStream import *
from cvlib import *
from LineFollowUtils import *
from imutils import *
import math

class RoboCV:
    def __init__(self) :
        self.cameraStream = PiCameraStream()
        self.stopped = True
        self.currentVisionFunction = None
        self.visionFunctions = {}
        self.visionFunctions["LineFollower"] = LineFollower()
        self.visionFunctions["HSVFinder"] = HSVFinder()
        self.currFrame = None

    def __str__ ( self ) :
        return "RoboCV: \n\t" + self.currentVisionFunction.__str__() + "\n\t" + self.cameraStream.__str__()

    def start ( self ) :
        print("Starting RoboCV thread")
        self.stopped = False

        self.cameraStream.start()
        Thread(target = self.update, args=()).start()

        return self

    def stop ( self ) :
        print("Stopping RoboCV thread")
        self.stopped = True
        self.cameraStream.stop()

    def update ( self ) :
        #self.cameraStream.update()
        while not self.stopped:
            if self.currentVisionFunction is not None:
                self.currFrame = self.cameraStream.read()
                self.currentVisionFunction.operate(self.currFrame)


        return


    def getPiCameraStream( self ) :
        return self.cameraStream

    def getUnmodifiedFrame(self) :
        return self.currFrame

    def getCurrentVisionFunction (self):
        return self.currentVisionFunction

    def setCurrentVisionFunction (self, visionFunctionString):
        newFunction = self.visionFunctions[visionFunctionString]
        if newFunction is not None:
            self.currentVisionFunction = newFunction
        else:
            print("Error in RoboCV: Given visionFunctionString is not known")

    def getCurrentVisionFunctionValues( self ):
        return self.currentVisionFunction.getValues()

    def getCurrentVisionFunctionValue( self, valueName ):
        return self.currentVisionFunction.getValue( valueName )

class VisionFunction:
    def __init__(self) :
        self.values = {}
        self.values["name"] = "VisionFunction"
        return

    def operate( self, rawFrame ):
        return

    def getValues ( self ) :
        return self.values

    def getValue ( self, valueName ) :
        return self.values.get(valueName)

class LineFollower(VisionFunction):
    def __init__(self, numSlices = 5):
        VisionFunction.__init__(self)
        self.values["name"] = "LineFollower"
        self.values["lineImage"] = None
        self.values["resizedImage"] = None
        self.values["missingContours"] = numSlices
        self.values["direction"] = 0

        self.minGreen = (47, 140, 60) #Perhaps this will find green?
        self.maxGreen = (70, 255, 255)

        self.minIRGreen = (53, 76, 20)
        self.maxIRGreen = (152, 168, 253)

        self.imageParts = []

        self.renderText = True

        for i in range(numSlices):
            self.imageParts.append(LineImageSlice())

        self.numSlices = numSlices
        self.direction = 0
        self.directionVector = [(0, 0), (0, 0)]
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
        #binarizedIR = binarize(rawResized, self.minIRGreen, self.maxIRGreen)

        #finalBinarized = cv2.bitwise_or(binarized, binarizedIR)

        #Slice up the image and calculate the direction center for each piece
        sliceImage(binarized, self.imageParts, self.numSlices, self.renderText, rawResized)

        #Add up the direction sum and check for missing contours
        numMissing = 0
        lowestCoord = (0, 10000)
        highestCoord = (0, 0)
        for i in range(self.numSlices):
            if self.imageParts[i].isLinePresent():
                modDir = (self.imageParts[i].dir * ( 1 + float(i) / 4.0))
                self.direction += modDir
                if self.imageParts[i].absoluteY != 0:
                    if self.imageParts[i].absoluteY < lowestCoord[1]:
                        lowestCoord = (self.imageParts[i].contourCenterX, self.imageParts[i].absoluteY)
                    elif self.imageParts[i].absoluteY > highestCoord[1]:
                        highestCoord = (self.imageParts[i].contourCenterX, self.imageParts[i].absoluteY)

                #print("Part [" + str(i) + "]: " + str(modDir))
                continue
            else:
                #print("Part[" + str(i) + "]: No line contour")
                numMissing += 1

        if (self.numSlices - numMissing) == 0:
            self.direction = 0
        else:
            self.direction = self.direction / (self.numSlices - numMissing)

        self.directionVector[0] = lowestCoord
        self.directionVector[1] = highestCoord

        deltaX = self.directionVector[0][0] - self.directionVector[1][0]
        deltaY = self.directionVector[1][1] - self.directionVector[0][1]

        if deltaX == 0:
            angle = 90.0
        else:
            angle = math.degrees(math.atan( float(deltaY) / float(deltaX) ))
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

        return

    def __str__( self ) :
        return "LineFollower"

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

            cv2.createTrackbar('H_MAX', 'window', 0, 180, nothing) #H
            cv2.createTrackbar('H_MIN', 'window', 0, 180, nothing)

            cv2.createTrackbar('S_MAX', 'window', 0, 255, nothing) #S
            cv2.createTrackbar('S_MIN', 'window', 0, 255, nothing)

            cv2.createTrackbar('V_MAX', 'window', 0, 255, nothing) #V
            cv2.createTrackbar('V_MIN', 'window', 0, 255, nothing)
            time.sleep(1)

        time.sleep( 1 / 20 )
        rawResized = imutils.resize(rawFrame, height = 400)
        self.values["resized"] = rawResized

        #Find the color :D
        colorMask = colorFinder(rawResized, 0, 0, self.pts)

        #Let's return the mask in case we need it
        self.values["binarized"] = colorMask

        self.count += 1
        return

    def __str__(self):
        return "HSVFinder"
