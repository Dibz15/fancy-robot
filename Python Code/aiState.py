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
import collections



'''
*	Class: aiState
*	Description: aiState is the parent class for a set of classes used by the aiController as an abstraction to
handle the functionality of one of the states, or objectives, the robot is to execute.
The transition between each state will be managed through the aiStack, within the aiController class.
*	Author:       Michael Lee
*	Date Created:	2/12/18
'''
class aiState:          #parent class for states

    def __init__(self, aiC):                #constructor for aiState
        print("Initializing aiState")
        self.aiC = aiC
        self.pi = self.aiC.pi

    #gets access to any of the controller classes
    def getController(self, controlName):
        return self.aiC.getController(controlName)  #returns controller from aiController

    def update(self):
        pass

'''
*	Class: lineFollowState
*	Description: lineFollowState is a child class of aiState. Utilized by the aiController class, it outlines the routine
    which directs the robot to search for a line that is of bright green color that it will then proceed to follow
*	Author:       Austin Dibble
*	Date Created:
'''
class lineFollowState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)
        self.ptC = self.aiC.getController(self.aiC.panTiltString)
        self.ptC.setServo(ptC.getTiltPin(), 110)

        self.rCV = self.aiC.getController(self.aiC.rCVString)
        self.rCV.setCurrentVisionFunction("LineFollower")

        self.motors = self.aiC.getController(self.aic.motorsString)

        self.anglePID = PID(initSetpoint = 90, Kp = 0.01, Ki = 0.001, Kd = 0.05)

        while rCV.getCurrentVisionFunctionValue("lineImage") is None:
            time.sleep(0.1)
            continue

        self.prevDirection = 0.0

    def updateServo(self, direction, angle, ptAngle):
        ptAngleDelta = (direction * 0.009)
        directionDelta = (direction - self.prevDirection) * 0.04

        ptAngle = max(70, min(110, ptAngle + ptAngleDelta + directionDelta))
        self.ptC.setServo(self.ptC.pan, ptAngle)
        time.sleep(0.1) #Make sure our servo has a bit of time to get in position

        self.prevDirection = direction

    def update(self):

        #If we have most of our contours in view
        if rCV.getCurrentVisionFunctionValue("missingContours") <= 3:
            #Get our direction value from roboCV
            direction = self.rCV.getCurrentVisionFunctionValue("direction")
            #Get our approx line angle
            angle = self.rCV.getCurrentVisionFunctionValue("angle")

            #Let's move our camera to look at the line
            ptAngle = self.ptC.getCurrentAngle(self.ptC.getPanPin())
            self.updateServo(direction, angle, ptAngle)

            self.motors.forwardFunction(speed = 25)

            #We need to take into consideration 3 factors: "direction" (how far left or right)
            # the line is ), "angle" (what the angle is to the camera), and camera angle,
            #or what angle the camera is turned to.

            #We're super straight
            if (angle <= 95 and angle >= 85) and (ptAngle <= 95 and ptAngle >= 85) and (abs(direction) < 5):
                self.motors.forwardFunction(speed = 25)

            #Our servo is pointing a bit in either direction
            if (ptAngle > 100 or ptAngle < 80):
                angleDelta = ptAngle - 90
                self.motors.halt()
                wheelToUse = self.motors.LEFT if (angleDelta < 0) else self.motors.RIGHT
                self.motors.turnAngleFunction(angle = angleDelta, wheel = wheelToUse)

                while not self.currentOperation.complete:
                    ptAngle = self.ptC.getCurrentAngle(self.ptC.getPanPin())
                    self.updateServo(direction, angle, ptAngle)

            #Our line is pointed in either direction relative to the robot
            if (angle > 115 or angle < 65):
                angleDelta = angle - 90
                self.motors.halt()
                wheelToUse = self.motors.LEFT if (angleDelta < 0) else self.motors.RIGHT
                self.motors.turnAngleFunction(angle = angleDelta, wheel = wheelToUse)

                while not self.currentOperation.complete:
                    ptAngle = self.ptC.getCurrentAngle(self.ptC.getPanPin())
                    self.updateServo(direction, angle, ptAngle)


        else:
            time.sleep( 1.0 / 10.0 )

        return

'''
*	Class: findHomeState
*	Description: findHomeState is a child class of aiState. Utilized by the aiController class, it outlines the routine
    which directs the robot to search for the base station relying on input from its IR receivers and computer vision.
*	Author:
*	Date Created:
'''
class findHomeState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)
        self.irAngle = 0
        self.irBeamLocked = False
        self.correctionAngle = 0
        self.angleStack = collections.deque(5*[0], 5)

   def update(self):
        print("in findHomeState")
        print("scanning for IR signal")
        sensors = self.getController(self.aiC.sensorString)      #for easier access to sensorController's functions
        motors = self.getController(self.aiC.motorsString)       #set variable for easier access to MotorController

        while not self.irBeamLocked:                             #corrects orientation until lined up with transmitter
            time.sleep( 0.2 )

            for i in xrange(5):                                  #get 5 IR angle values
                self.irAngle = sensors.getIRAngle():                     #get IR angle
                self.angleStack.appendleft(self.irAngle)



            if self.irAngle == 90:                                   #robot is lined up with base station
                self.irBeamLocked = true

            elif self.irAngle < 90:                                  #transmitter is offcenter to the right
                self.correctionAngle = self.irAngle - 90 + 20        #calculate angle needed to line up with ir transmitter
                                                                     #extra 20 degrees added to compensate for turnAngleFunction
                                                                     #may need recalibration
                motors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.waitOnAction()                                    #wait for turn

            elif self.irAngle > 90:                                  #transmitter is off center to the left
                self.correctionAngle = self.irAngle - 90 - 20        #calculate angle needed to line up with ir transmittermotors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.waitOnAction()                                    #wait for turn

            elif self.irAngle == 300
                 return

'''
*	Class: chargeState
*	Description: chargeState is a child class of aiState. Utilized by the aiController class, it outlines the routine
    which directs the robot to poll the battery voltage of its battery pack, and after detecting that its battery is
    sufficiently charged, leaves the base station.
*	Author:      Michael Lee
*	Date Created:	2/22/18
'''
class chargeState(aiState):         #robot's charging state


    def __init__(self, aiC):        #constructor for chargeState
        aiState.__init__(self, aiC)


    def update(self):
        print ("in chargeState's update")

        sensors = self.getController(self.aiC.sensorString)  #variable for easier access to sensors controller

        while not chargeComplete:   #loops until chargeComplete is True
            time.sleep( 0.2 )
            battVoltage = sensors.getBatteryVoltage()     #polls battery voltage from sensors thread
            if battVoltage >= 4.00:                       #charge complete at battery voltage = 4 volts
                chargeComplete = True                     #set flag to end while loop

        motors = self.getController(self.aiC.motorsString)
        motors.reverseFunction(distance = 40)             #reverse out of base station
        motors.waitOnAction()                             #wait until reverse function has completed
        motors.turnAngleFunction(angle = -160)            #reorient in opposite direction
        motors.waitOnAction()                             #wait for turn

        self.aiC.popState()                               #pop chargeState off stack

        return

'''
*	Class: AvoidanceState
*	Description: lineFollowState is a child class of aiState. After receiving a critical value alert from the ultrasonic sensor,
    sensorsController calls the collisionCB function within aiController class which will push AvoidanceState onto the stack.
    AvoidanceState directs the robot to reorient itself away from the obstacle that the ultrasonic sensor has detected.
*	Author:       Michael Lee and Austin Dibble
*	Date Created:	2/21/18
'''

class avoidanceState(aiState):

    def __init__(self, aiC):                            #constructor for avoidanceState
        aiState.__init__(self, aiC)
        self.turnObstruct = False                       #default value for variable representing if a turn attempt was obstructed

    def detectTurnStall(self, motors, pivotWheel):      #function to determine if there was turn obstruction
        self.turnObstruct = False
        motors.setStallState([False, False])            #reset the stall state of both wheels to false
        time.sleep(0.25)

        while not motors.currentOperation.complete:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            stallState = motors.getStallState()         #polls the stall state using motor controller for both wheels
            print("Stallstate: " + str(stallState))
            print("Pivotwheel: " + str(pivotWheel))

            if (stallState[0] == True and pivotWheel == motors.RIGHT) or (stallState[1] == True and pivotWheel == motors.LEFT):
                if not motors.currentOperation.complete:    #checks if a non-pivot wheel stalled during turn
                    print ("stall detected during turn")
                    self.turnObstruct = True                #sets variable to true, signifying stall has occured
                    motors.halt()                           #cancels turn function by halting motors
                    break

        motors.forwardFunction(distance = 25)               #move forward 25 cm
        motors.waitOnAction()

    def update(self):                                       #sets routine for obstacle avoidance
        print ("in avoidanceState's update")

        motors = self.getController(self.aiC.motorsString)  #set variable for easier access to MotorController
        print("turning left 70 degrees, pivot on right wheel")
        motors.turnAngleFunction(angle = 70, wheel = motors.LEFT) #turn command for a 70 degree left turn, pivoting on right wheel

        self.detectTurnStall(motors, motors.RIGHT)          #calls function to determine if stall occurs during turn


        if self.turnObstruct == True:                       #if stall has occurred during turn
            print("turning right 120 degrees, pivot on left wheel")
            time.sleep(0.1)
            motors.turnAngleFunction(angle = -120, wheel = motors.RIGHT)  #turn 120 degrees in the other direction, pivot on left

            self.turnObstruct = False                       #reset variable to default
            self.detectTurnStall(motors, motors.LEFT)       #detect if stall occurs during this second turn
            if self.turnObstruct == True:
                motors.reverseFunction(distance = 30)       #if impeded from turning in both left and right directions, reverse
                motors.waitOnAction()                       #wait until reverse function has completed
                motors.turnAngleFunction(angle = -160)      #reorient in opposite direction

        self.aiC.collisionCBtriggered = False               #clear collisionCBtriggered flag before popping

        self.aiC.popState()                                 #pop avoidanceState off stack

        return
