from PiCameraStream import *
from cvlib import *

class RoboCV:
    def __init__(self) :
        self.cameraStream = PiCameraStream()
        self.stopped = True
        self.currentVisionFunction = None
        self.visionFunctions = {}
        self.visionFunctions["LineFollower"] = LineFollower()
        self.visionFunctions["HSVFinder"] = HSVFinder()

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
                self.currentVisionFunction.operate(self.cameraStream.read())

        return


    def getPiCameraStream( self ) :
        return self.cameraStream

    def getUnmodifiedFrame(self) :
        return self.cameraStream.read()

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
    def __init__(self):
        VisionFunction.__init__(self)
        self.values["name"] = "LineFollower"
        return

    def operate( self, rawFrame ) :

        return

    def __str__( self ) :
        return "LineFollower"

class HSVFinder(VisionFunction):
    def __init__(self):
        VisionFunction.__init__(self)
        self.values["name"] = "HSVFinder"
        #Create trackbars
        cv2.namedWindow('window')

        cv2.createTrackbar('H_MAX', 'window', 0, 180, nothing) #H
        cv2.createTrackbar('H_MIN', 'window', 0, 180, nothing)

        cv2.createTrackbar('S_MAX', 'window', 0, 255, nothing) #S
        cv2.createTrackbar('S_MIN', 'window', 0, 255, nothing)

        cv2.createTrackbar('V_MAX', 'window', 0, 255, nothing) #V
        cv2.createTrackbar('V_MIN', 'window', 0, 255, nothing)

        self.pts = deque(maxlen=16)

        return

    def operate( self, rawFrame ) :
        time.sleep( 1 / 20 )
        rawResized = imutils.resize(rawFrame, height = 400)
        self.values["resized"] = rawResized

        colorMask = colorFinder(rawResized, 0, 0, self.pts)

        self.values["binarized"] = colorMask

        return

    def __str__(self):
        return "HSVFinder"
