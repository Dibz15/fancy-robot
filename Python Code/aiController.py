import pigpio
import time
import Constants
from MotorController import *
from SensorsController import *
from SpeechController import *
import PanTiltControl
from aiState import *
from threading import Thread
import imp


#the forum for aiStates to be ran and transitioned
#manages current and next states by use of stack
class aiController:

    def __init__(self, pi, speech, motors, panTilt, rCV, initialState) :
        print("Initializing aiController")
        self.pi = pi
        self.speech = speech
        self.motors = motors
        self.panTilt = panTilt

        self.stopped = True
        self.speechString = "speech"
        self.motorsString = "motors"
        self.panTiltString = "panTilt"
        self.rCVString = "rCV"
        #initialize stack with given initialState(from Main)

    #gets access to any of the controller classes
    def getController(self, controlName):
        if controlName == self.speechString:
            return self.speech
        elif controlName == self.motorsString:
            return self.motors
        elif controlName == self.panTiltString:
            return self.panTilt
        elif controlName == self.rCVString:
            return self.rCV
       else:
            self.speech.speak("ay eye controller. get controller received invalid name")

        #list of States:  "lineFollowState", "findHomeState", "chargeState"


    def start (self):
        print("Starting aiController")
        self.stopped = False

        # We shouldn't implement a new thread here, but call update from main
        #Thread(target = self.update, args = ()).start()

    def stop (self):
        print("Stopping aiController")
        self.stopped = True

    #calls the collision callback function within currentState, to transitione
    # to avoidanceState
    def collisionCB():
        self.stack.peek().collisionCB()

    #pushes a given state onto stack
    def pushState (self, stateName):
        #put stateName on top of stack
        return

    #calls update of the current state
    def update(self):
        #stack.peek.update()
        print("Updating aiController")
