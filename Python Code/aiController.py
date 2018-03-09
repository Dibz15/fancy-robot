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
import collections


#the forum for aiStates to be ran and transitioned
#manages current and next states by use of stack
class aiController:

    def __init__(self, pi, speech, motors, panTilt, rCV, sensors) :
        print("Initializing aiController")
        self.pi = pi
        self.speech = speech
        self.motors = motors
        self.panTilt = panTilt
        self.rCV = rCV
        self.sensors = sensors

        self.stopped = True
        self.speechString = "speech"
        self.motorsString = "motors"
        self.panTiltString = "panTilt"
        self.rCVString = "rCV"
        self.sensorString = "sensors"

        initialState = lineFollowState(self)

        self.collisionCBtriggered = True
        self.collisionCBenable = True       #used within aiState's findHomeState's alignCenter function
        self.voltageCBtriggered = True
        self.sensors.setDistanceCallback(self.collisionCB)
        self.sensors.setVoltageCallback(self.voltageCB)

        #initialize stack with given initialState(from Main)

        print ("pushing initial state onto stack")
        self.aiStack = collections.deque()
        self.aiStack.appendleft(initialState)
        time.sleep(0.1)

        self.collisionCBtriggered = False
        self.voltageCBtriggered = False

#gets access to any of the controller classes
    def getController(self, controlName):
        #aiC.getController(aiC.speechString)
        if controlName == self.speechString:
            return self.speech
        elif controlName == self.motorsString:
            return self.motors
        elif controlName == self.panTiltString:
            return self.panTilt
        elif controlName == self.rCVString:
            return self.rCV
        elif controlName == self.sensorString:
            return self.sensors
        else:
            self.speech.speak("ay eye controller. get controller received invalid name")

        #list of States:  "lineFollowState", "findHomeState", "chargeState"


    def start (self):
        print("Starting aiController")
        self.stopped = False


    def stop (self):
        print("Stopping aiController")
        self.stopped = True

#called from sensorsController if immminent collision detected.
    def collisionCB(self):
        if not self.collisionCBtriggered and self.collisionCBenable:
            self.collisionCBtriggered = True
            self.motors.halt()                   #imminent collision, stop motors
            self.pushState(avoidanceState(self)) #push the avoidanceState onto the stack
            print ("in aiC collisionCB")
            #self.speech.speak("imminent collision detected")
            #self.speech.speak("halting motors.")
            #self.speech.speak("Avoidance state")

#called from sensorsController if battery voltage polled as critical value
    def voltageCB(self):
        if not self.voltageCBtriggered:
            self.voltageCBtriggered = True
            self.motors.halt()
            print("Voltage low")
            #print("Stack top: " + str(self.peekState()))
            self.aiStack.clear()                #clear stack of all States
            time.sleep(0.1)
            self.pushState(findHomeState(self))  #push findHomeState onto the stack
            #self.speech.speak("critical battery voltage detected.")
            #self.speech.speak("halting motors.")
                              #halt motors
            #self.speech.speak("switching to find home state.")
            print("Going to find home state")




#pushes a given state onto stack
    def pushState (self, stateName):
        self.aiStack.appendleft(stateName)

#pops a given state from stack
    def popState (self):
        if (len(self.aiStack) != 0):
            return self.aiStack.popleft()
        else:
            return None

    def peekState(self):
        if (len(self.aiStack) != 0):
            return self.aiStack[0]
        else:
            return None

    def clearStack(self):
        self.aiStack.clear()


#calls update of the current state
    def update(self):
        if (len(self.aiStack) != 0):
            stateToUpdate = self.aiStack[0]

            if stateToUpdate is not None:
                print("Updating state: " + str(stateToUpdate))
                stateToUpdate.update()

        #print("Updating aiController")
