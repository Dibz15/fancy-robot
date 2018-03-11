'''
*	File: aiController.py
*	Description:  This file holds the aiController class which acts as a layer of abstraction for the main code
by managing past and current states within a stack. Also handles any callBack functions tied to Sensors Controller.
*
*	Author(s):		Michael Lee
*	Date Created:	1/28/17
'''

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
'''
*	Class: aiController
*	Description:	aiController is a class which provides a layer of abstraction for the main code block, and
    manages the different aiStates of the robot within a stack. It also handles any callBack functions that are high priority
    and need the ability to interrupt a current mode
*	Author(s):	   Michael Lee
*	Date Created:	1/28/18
'''

class aiController:

    def __init__(self, pi, speech, motors, panTilt, rCV, sensors) :  #calls constructor of aiController
        print("Initializing aiController")
        self.pi = pi
        self.speech = speech                 #assign speech controller as variable
        self.motors = motors                 #assign motors controller as variable
        self.panTilt = panTilt               #assign panTilt controller as variable
        self.rCV = rCV                       #assign rCV controller as variable
        self.sensors = sensors               #assign sensors controller as variable

        self.stopped = True                  #used to signify that aiController is stopped
        self.speechString = "speech"
        self.motorsString = "motors"
        self.panTiltString = "panTilt"
        self.rCVString = "rCV"
        self.sensorString = "sensors"

        initialState = lineFollowState(self)  #lineFollowState is initial mode of robot

        self.collisionCBenable = True         #used within aiState's findHomeState's alignCenter function
        self.voltageCBtriggered = True        #flag used so that voltageCB can't be called if already triggered
        self.sensors.setDistanceCallback(self.collisionCB)  #
        self.sensors.setVoltageCallback(self.voltageCB)

        print ("pushing initial state onto stack")
        self.aiStack = collections.deque()    #create and initialize stack
        self.aiStack.appendleft(initialState)
        time.sleep(0.1)

        self.collisionCBtriggered = False     #flag used so that avoidance state can't be called within itself
        self.collisionCBenable = True         #used within aiState's findHomeState's alignCenter function (to disable obstacle avoidance)
        self.voltageCBtriggered = False       #flag used so that voltageCB can't be called if already triggered

    '''
    ****************************************************
    *Function: getController(self, controlName)
    *Description: Gets access to any of the controller classes
    *
    *Parameters:
    *   self, name of the controller class
    *Returns:
    *   controller class
    ****************************************************
    '''
    #gets access to any of the controller classes
    def getController(self, controlName):
        #aiC.getController(aiC.speechString)
        if controlName == self.speechString:    #used by aiState class to get access to SpeechController
            return self.speech
        elif controlName == self.motorsString:  #used by aiState class to get access to MotorController
            return self.motors
        elif controlName == self.panTiltString: #used by aiState class to get access to PanTiltControl
            return self.panTilt
        elif controlName == self.rCVString:     #used by aiState class to get access to SpeechController
            return self.rCV
        elif controlName == self.sensorString:  #used by aiState class to get access to SensorsController
            return self.sensors
        else:
            print("invalid controller name")

    def start (self):                           #starting aiController
        print("Starting aiController")
        self.stopped = False


    def stop (self):                            #stopping aiController
        print("Stopping aiController")
        self.stopped = True

    '''
    ****************************************************
    *Function: collisionCB(self)
    *Description: called from within Sensors Controller if
    *ultrasonic sensor detects imminent collision
    *Parameters:
    *   self
    *Returns:
    *
    ****************************************************
    '''
    #called from sensorsController if immminent collision detected.
    def collisionCB(self):
        if not self.collisionCBtriggered and self.collisionCBenable: #if not already in avoidance state or prior to docking
            self.collisionCBtriggered = True                         #set flag, so avoidance can't be triggered within
            self.motors.halt()                   #imminent collision, stop motors
            self.pushState(avoidanceState(self)) #push the avoidanceState onto the stack
            print ("in aiC collisionCB")
            #self.speech.speak("imminent collision detected")
            #self.speech.speak("halting motors.")
            #self.speech.speak("Avoidance state")

    '''
    ****************************************************
    *Function: voltageCB(self)
    *Description: called from within Sensors Controller if
    *ultrasonic sensor detects imminent collision
    *
    *Parameters:
    *   self, name of the controller class
    *Returns:
    *   controller class
    ****************************************************
    '''
    #called from sensorsController if battery voltage polled as critical value
    def voltageCB(self):
        if not self.voltageCBtriggered:             #checks to see if callback has already been executed
            self.voltageCBtriggered = True          #sets flag so that findHomeState can't be re-pushed onto stack
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

    '''
    ****************************************************
    *Function: pushState(stateName)
    *Description: pushes a given state onto stack
    *Parameters:
    *   self, aiState
    *Returns:
    *
    ****************************************************
    '''
    def pushState (self, stateName):
        self.aiStack.appendleft(stateName)

    '''
    ****************************************************
    *Function: popState(stateName)
    *Description: pops a given state off of stack
    *Parameters:
    *   self, aiState
    *Returns:
    *   aiState from top of stack
    ****************************************************
    '''
    def popState (self):
        if (len(self.aiStack) != 0):               #check if stack is empty
            return self.aiStack.popleft()
        else:
            return None

    '''
    ****************************************************
    *Function: peekState(stateName)
    *Description: peeks at current state on stack
    *Parameters:
    *   self, aiState
    *Returns:
    *   aiState from top of stack
    ****************************************************
    '''
    def peekState(self):
        if (len(self.aiStack) != 0):            #check if stack is empty
            return self.aiStack[0]
        else:
            return None

    '''
    ****************************************************
    *Function: clearStack(self)
    *Description: clear stack of all states
    *Parameters:
    *   self
    *Returns:
    *
    ****************************************************
    '''
    def clearStack(self):
        self.aiStack.clear()                    #clear stack

    '''
    ****************************************************
    *Function: update(self)
    *Description: looks at the top aiState on stack, and
    calls its update function
    *Parameters:
    *   self
    *Returns:
    *
    ****************************************************
    '''
    #calls update of the current state
    def update(self):
        if (len(self.aiStack) != 0):            #check if stack is empty
            stateToUpdate = self.aiStack[0]     #get state on top of stack

            if stateToUpdate is not None:
                print("Updating state: " + str(stateToUpdate))
                stateToUpdate.update()          #call update function of state

        #print("Updating aiController")
