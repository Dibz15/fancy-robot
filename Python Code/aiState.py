import pigpio
import time
import Constants
#from RoboCV import *
from MotorController import *
from SensorsController import *
from SpeechController import *
from aiController import *
import PanTiltControl
from threading import Thread
import imp

#parent class for states
class aiState:

    def __init__(self, aiC, avoidState):
        print("Initializing aiController")
        self.pi = self.aiC.pi

    #gets access to any of the controller classes
    def getController(self, controlName):
        return self.aiC.getController(controlName)

    #function shell, where substates of each child classe
    #will be called, define substates as separate functions
    def update(self):
        pass

    #pushes avoidanceState onto stack
    def collisionCB(self):
        self.aiC.pushState(avoidState)

class lineFollowState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)

    def update(self):

        return

class findHomeState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)

    def update(self):

class chargeState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)

    def update(self):

        return

class avoidanceState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)

    def update(self):

        return
