'''
*	File: PiCameraStream.py
*	Description:  This module interfaces with the RPi camera API
*	Author(s):		Austin Dibble
*	Date Created:	12/22/17
'''

from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import time

'''
*	Class: PiCameraStream
*	Description:  This class provides an interface to get camera data
*	Author(s):		Austin Dibble
*	Date Created:	12/22/17
'''

class PiCameraStream :
    def __init__(self, w = 640, h = 480, fps = 20, vflip = True, hflip = True) :
        self.width = w
        self.height = h

        #initialize pi camera
        self.camera = PiCamera()
        self.camera.resolution = (self.width, self.height)
        self.camera.framerate = fps
        self.camera.vflip = vflip
        self.camera.hflip = hflip

        #initialize camera stream
        self.rawCapture = PiRGBArray(self.camera, size=(self.width, self.height))
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        #Threading
        self.stopped = True

        #Frame
        self.frame = None

    def start( self ) :
        print("PiCameraStream: Starting Camera")
        self.stopped = False

        #Start our frame retrieval thread
        Thread(target = self.update, args=()).start()

        #Wait for frames
        while self.frame is None: time.sleep(0.01)

        return self

    def update ( self ) :
        #Go through all the frames
        for frame in self.stream :
            #Get the frame
            image = frame.array

            self.frame = image
            self.rawCapture.truncate(0)

            #Close everything if needed
            if self.stopped :
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def stop ( self ) :
        print("PiCameraStream: Stopping camera")
        self.stopped = True

    def __str__( self ) :
        return "PiCamStream: (" + str(self.width) + ", " + str(self.height) + ")"

    def getWidth ( self ) :
        return self.width

    def getHeight ( self ) :
        return self.height

    def getCamera ( self ) :
        return self.camera

    def getRawCapture ( self ) :
        return self.rawCapture

    def getStream ( self ) :
        return self.stream

    #Read the most recent frame
    def read ( self ) :
        return self.frame
