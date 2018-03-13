'''
*	File: aiState.py
*	Description:  This file holds the aiState class which handles the functionality of each mode
*   of the robot, including obstacle avoidance, line following, finding the base station, and charging
*
*	Author(s):		Michael Lee, Austin Dibble
*	Date Created:	1/28/17
'''

import pigpio
import time
import Constants
import cv2
from OpenCV.RoboCV import *
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
        self.aiC = aiC                      #passed aiController as parameter
        self.pi = self.aiC.pi

    #gets access to any of the controller classes
    def getController(self, controlName):
        return self.aiC.getController(controlName)  #returns controller from aiController

    def update(self):                       #shell function which is individualized within each class
        pass

'''
*	Class: lineFollowState
*	Description: lineFollowState is a child class of aiState. Utilized by the aiController class, it outlines the routine
    which directs the robot to search for a line that is of bright green color that it will then proceed to follow
*	Author:       Austin Dibble
*	Date Created: 2/15/18
'''
class lineFollowState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)

        self.ptC = self.aiC.getController(self.aiC.panTiltString)
        self.ptC.pointDown()

        self.rCV = self.aiC.getController(self.aiC.rCVString)
        self.rCV.setCurrentVisionFunction("LineFollower")

        self.motors = self.aiC.getController(self.aiC.motorsString)

        self.anglePID = PID(initSetpoint = 90, Kp = 0.01, Ki = 0.001, Kd = 0.05)

        ''''
        print("Waiting for line image")
        while self.rCV.getCurrentVisionFunctionValue("modified") is None:
            time.sleep(0.1)
            continue
        print("Got first image")
        '''

        print("Starting line follow state")
        self.prevDirection = 0.0

        self.showDisplay = False
        self.moveServo = False
        self.hasSearched = False
        self.lastDirection = 0

    '''
    ****************************************************
    *Function: updateDisplay
    *Description: updates our image display, for debuggig
    *
    *Parameters:
    *   self
    *Returns:
    *   nothing
    ****************************************************
    '''
    def updateDisplay(self):
        cv2.imshow("Frame", self.rCV.getCurrentVisionFunctionValue("modified"))
    	#Show our binary color-detection image
    	cv2.imshow("Line Image", self.rCV.getCurrentVisionFunctionValue("lineImage"))
    	key = cv2.waitKey(1) & 0xFF

    	# if the `q` key was pressed, break from the loop
    	if key == ord("q"):
    		self.aiC.popState()

    '''
    ****************************************************
    *Function: updateServo
    *Description: Function for moving the servo in small increments
    * when we want small movement
    *
    *Parameters:
    *   self
    *Returns:
    *   nothing
    ****************************************************
    '''
    def updateServo(self, direction, angle, ptAngle):
        #Make sure we're looking at the ground
        self.ptC.pointDown()

        #Global constant to enable or disable this function
        if self.moveServo:
            #Get some delta based on the desired direction
            ptAngleDelta = (direction * 0.02)
            #Calculate the diff from old direction and prev direction
            directionDelta = (direction - self.prevDirection) * 0.04

            #Calculate the new angle
            ptAngle = max(70, min(110, ptAngle + ptAngleDelta + directionDelta))
            #Set it to the new angle
            self.ptC.setServo(self.ptC.pan, ptAngle)
            time.sleep(0.1) #Make sure our servo has a bit of time to get in position

            self.prevDirection = direction

        #If we are to update the display, do so
        if self.showDisplay:
            self.updateDisplay()

    '''
    ****************************************************
    *Function:  detectTurnStall
    *Description:
    *   detect  a stall during a turn, and see what wheel it was
    *Parameters:
    *   self
    *Returns:
    *   nothing
    ****************************************************
    '''
    def detectTurnStall(self, motors, pivotWheel):      #function to determine if there was turn obstruction
        self.turnObstruct = False
        self.motors.setStallState([False, False])            #reset the stall state of both wheels to false

        #While the motor operation is running
        while not self.motors.currentOperation.complete and not self.aiC.stopped:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            stallState = self.motors.getStallState()         #polls the stall state using motor controller for both wheels

            if (stallState[0] == True and pivotWheel == motors.RIGHT) or (stallState[1] == True and pivotWheel == motors.LEFT):
                if not self.motors.currentOperation.complete:    #checks if a non-pivot wheel stalled during turn
                    self.turnObstruct = True                #sets variable to true, signifying stall has occured
                    self.motors.halt()                           #cancels turn function by halting motors
                    break

    '''
    ****************************************************
    *Function:  linePresent
    *Description: uses computer vision function to determine if we can
    *   see the line
    *Parameters:
    *   self
    *Returns:
    *   bool whether line is present or not
    ****************************************************
    '''
    def linePresent(self):
        return (self.rCV.getCurrentVisionFunctionValue("missingContours") <= 3)

    '''
    ****************************************************
    *Function:  linePresentLow
    *Description: uses computer vision to see if we can see any part of the line
    *     This is more sensitive than the the other function linePresent
    *Parameters:
    *   self
    *Returns:
    *   bool whether line is present or not
    ****************************************************
    '''
    def linePresentLow(self):
        return (self.rCV.getCurrentVisionFunctionValue("missingContours") < 5)


    '''
    ****************************************************
    *Function: detectLinePresent
    *Description: allows a current motor operation to run, but also look for
    *   the line to be found. If the line is found during the operation, the op
    *   is cancelled and this returns true
    *Parameters:
    *   self
    *Returns:
    *   wether or not the line is present during the motor operation
    ****************************************************
    '''
    def detectLinePresent(self):
        #print("inside of detectLinePresent")
        time.sleep(0.2)

        while not self.motors.currentOperation.complete and not self.aiC.stopped:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            #If we glimpse the line
            if self.linePresentLow():
                return True

        return False

    '''
    ****************************************************
    *Function: lineSweep
    *Description: sweeps the pan servo left and right to look for the line using
    *   computer vision. If the line is found in one direction or the other, this function
    *   will move the robot in that direction a little bit
    *
    *Parameters:
    *   self
    *Returns:
    *   returns whether or not it found the line
    ****************************************************
    '''
    def lineSweep(self):
        #If no line in sight.
        if not self.linePresentLow():
            #print("Looking left")
            #center camera
            self.ptC.setServo(self.ptC.getPanPin(), 90)
            #Sweep left
            for i in range(19):
                time.sleep(0.1)
                self.ptC.incServo(self.ptC.getPanPin(), 2)
                if self.linePresentLow():
                    #If we found it to the left, turn slightly that way
                    self.motors.turnAngleFunction(angle = 15)
                    self.motors.waitOnAction()
                    return True

        if self.aiC.collisionCBtriggered:
            return False

        #If we still don't see the line
        if not self.linePresentLow():
            #print("Looking right")
            #Center camera
            self.ptC.setServo(self.ptC.getPanPin(), 90)
            #Look right
            for i in range(19):
                time.sleep(0.1)
                self.ptC.incServo(self.ptC.getPanPin(), -2)
                if self.linePresentLow():
                    #If we find it to the right, turn slightly that way
                    self.motors.turnAngleFunction(angle = -15)
                    self.motors.waitOnAction()
                    return True

        return False

    '''
    ****************************************************
    *Function: searchForLine(self, motors)
    *Description: moves in "concentric" hexagons, while scanning for a
    *line of the specified color in each leg of the hexagon
    *Parameters: self, motors
    *
    *Returns:
    *
    ****************************************************
    '''
    def searchForLine(self, motors):                       #drive around in expanding concentric circles
            #drive in circles
        lineFound = False
        n = 1                                              #number of hexagon iterations
        hexRate = 3                                        #rate of increase for perimeter of hexagon (hexrate*6 = increase in perimeter)

        speech = self.aiC.getController(self.aiC.speechString)
        speech.speak("Searching. in. hexagon. pattern.")
        time.sleep(0.5)
        if self.aiC.voltageCBtriggered:
            return
        while not lineFound and not self.aiC.stopped:
            time.sleep(0.1)

            if self.linePresentLow():                        #calls function to determine if line detected during forward movement
                lineFound = True
                return

            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward first leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                return
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():                                       #pans camera to search and detect the line
                lineFound = True
                return
            if self.aiC.collisionCBtriggered:                          #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return
            #speech.speak("Searching. in. hexagon. pattern.")
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward second leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                return
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():                                        #pans camera to search and detect the line
                lineFound = True
                return
            if self.aiC.collisionCBtriggered:                   #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return

            #speech.speak("Searching. in. hexagon. pattern.")
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward third leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                return
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():                                #pans camera to search and detect the line
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:                   #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return
            #speech.speak("Searching. in. hexagon. pattern.")
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward fourth leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():                                #pans camera to search and detect the line
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:                   #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return
            #speech.speak("Searching. in. hexagon. pattern.")
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward fifth leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():                                #pans camera to search and detect the line
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:                   #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return
            #speech.speak("Searching. in. hexagon. pattern.")
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward sixth leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = 51, wheel = motors.RIGHT) #turn command for a 60 degree left turn, pivoting on left wheel
            motors.waitOnAction()
            if self.lineSweep():                                #pans camera to search and detect the line
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:               #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return
            #speech.speak("Searching. in. hexagon. pattern.")
            motors.forwardFunction(distance = hexRate)       #travel to top left vertex of 2 * perimeter hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -102, wheel = motors.LEFT) #turn command for a 120 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():                            #pans camera to search and detect the line
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:               #checks for a collision, since time between updates is long
                return
            if self.aiC.voltageCBtriggered:
                return
            n  += 1

    '''
    ****************************************************
    *Function: update
    *Description: Performs the algorithm for line following. The two major cases
    *   driving the algorithm are if the line is present, and if it isn't. If it
    *   isn't present, the robot uses the pan and tilt camera to look for the line.
    *
    *Parameters:
    *   self
    *Returns:
    *
    ****************************************************
    '''
    def update(self):

        speech = self.getController(self.aiC.speechString)
        #speech.speak("Line. Follow. State.")
        if self.showDisplay:
            self.updateDisplay()

        defaultSpeed = 5

        self.ptC.pointDown()

        if self.aiC.voltageCBtriggered:
            return

        #If we have most of our contours in view
        if self.linePresent():
            #Get our direction value from roboCV
            direction = self.rCV.getCurrentVisionFunctionValue("direction")
            self.lastDirection = direction
            #Get our approx line angle
            angle = self.rCV.getCurrentVisionFunctionValue("angle")

            #Let's move our camera to look at the line
            ptAngle = self.ptC.getCurrentAngle(self.ptC.getPanPin())
            self.updateServo(direction, angle, ptAngle)

            self.motors.forwardFunction(speed = defaultSpeed)

            #We need to take into consideration 3 factors: "direction" (how far left or right)
            # the line is ), "angle" (what the angle is to the camera), and camera angle,
            #or what angle the camera is turned to.

            #We're super straight
            if (angle <= 110 and angle >= 70) and (abs(direction) < 160):
                #print("We're straight")
                self.motors.forwardFunction(speed = defaultSpeed)


            #Our line is pointed in either direction relative to the robot
            if (angle > 115 or angle < 65):
                #print("Line is off")
                angleDelta = angle - 90
                self.motors.halt()
                wheelToUse = self.motors.LEFT if (angleDelta < 0) else self.motors.RIGHT
                self.motors.turnAngleFunction(angle = ( angleDelta / 5), wheel = wheelToUse)
                self.motors.waitOnAction()

            if self.aiC.collisionCBtriggered:
                return

            if self.voltageCBtriggered:
                return

            #If the line is too far to one side
            if ( abs(direction) > 150 ):
                angleDelta = direction * 0.05
                if ( direction < 1):
                    #print("Line is on the right")
                    angleDelta = min(-5, angleDelta)
                else:
                    #print("Line is on the left")
                    angleDelta = max(5, angleDelta)

                #Stop the motors
                self.motors.halt()

                #Figure out which wheel to turn with
                wheelToUse = self.motors.LEFT if (angleDelta < 0) else self.motors.RIGHT

                #Make our turn
                self.motors.turnAngleFunction(angle = ( angleDelta ), wheel = wheelToUse)
                self.motors.waitOnAction()

            if self.aiC.collisionCBtriggered:
                return

            if self.voltageCBtriggered:
                return

        #If line is not present
        else:
            self.motors.halt()
            time.sleep( 1.0 / 10.0 )

            if self.voltageCBtriggered:
                return

            if not self.hasSearched:

                if self.aiC.collisionCBtriggered:
                    return

                #If no line in sight
                if not self.linePresent():
                    #print("Looking left")
                    self.ptC.setServo(self.ptC.getPanPin(), 90)
                    #Look left
                    for i in range(19):
                        time.sleep(0.1)
                        self.ptC.incServo(self.ptC.getPanPin(), 2)
                        if self.linePresent():
                            #If we find the line, turn left slightly
                            self.motors.turnAngleFunction(angle = 15)
                            self.motors.waitOnAction()
                            break

                if self.aiC.collisionCBtriggered:
                    return

                if self.voltageCBtriggered:
                    return

                #Still no line in sight
                if not self.linePresent():
                    #print("Looking right")
                    self.ptC.setServo(self.ptC.getPanPin(), 90)
                    #Look right
                    for i in range(19):
                        time.sleep(0.1)
                        self.ptC.incServo(self.ptC.getPanPin(), -2)
                        if self.linePresent():
                            #If we find the line, turn right slightly
                            self.motors.turnAngleFunction(angle = -15)
                            self.motors.waitOnAction()
                            break

                if self.aiC.collisionCBtriggered:
                    return

                if self.voltageCBtriggered:
                    return

                #Start looking here
                self.searchForLine(self.motors)

            self.ptC.setServo(self.ptC.getPanPin(), 90)

        return

'''
*	Class: findHomeState
*	Description: findHomeState is a child class of aiState. Utilized by the aiController class, it outlines the routine
    which directs the robot to search for the base station relying on input from its IR receivers and computer vision,
    and dock within it
*	Authors: Michael Lee and Austin Dibble
*	Date Created: 2/25/18
'''
class findHomeState(aiState):

    def __init__(self, aiC):
        aiState.__init__(self, aiC)
        self.irAngle = 0                                #90 degrees signifies that robot is center aligned with IR transmitter
        self.irBeamLocked = False                       #flag set when ir value of '90' reliably polled
        self.cvInRange = False                          #flag set when computer vision becomes available as navigation tool
        self.alignedWithCenter = False                  #flag set when robot is ready to begin docking procedure
        self.dockFlag = False                           #flag set when CV determines alignment is centered enough to begin docking protocol
        self.correctionAngle = 0                        #angle signifies correction angle needed to center align with IR transmitter
        self.angleStack = collections.deque(5*[0], 5)   #max cap 5 stack, holds IR angle values to find average value

        self.GREEN_LEFT = 0
        self.GREEN_RIGHT = 1

    '''
    ****************************************************
    *Function: cvScan(self)
    *Description: Scans around it to determine if the orange tape on center
    of base station is visible, and of a certain area. If the distance away
    is determined to be within range, and area of tape over a specified threshold,
    sets "cvInRange" variable to be True, indicating that computer vision is now
    a reliable resource in navigating back to the base station.
    *
    *Parameters:
    *   self
    *Returns:
    *   float value representing angle to orange tape, or 0 if a collision
    has been triggered
    ****************************************************
    '''
    def cvScan(self):
        print(" first iteration of CVScan")
        self.aiC.collisionCBtriggered = False
        speech = self.getController(self.aiC.speechString)
        dist_thresh = 70
        area_thresh = 1300
        #pans if needed to attempt to recognize secondary recognition color adhered
        #to center of base station. Sets 'cvInRange' to 'True' if recognizable
        rCV = self.aiC.getController(self.aiC.rCVString)
        ptC = self.aiC.getController(self.aiC.panTiltString)

        ptC.center()

        rCV.setCurrentVisionFunction("BaseFinder")
        while rCV.getCurrentVisionFunctionValue("binarizedOrange") is None:
            time.sleep(1.0 / 10.0)
            #print("Waiting for binarized image")
            continue

        if self.aiC.collisionCBtriggered:           #checks if imminent collision detected
            return 0.0

        print("Looking for tape in current view")
        if not rCV.getCurrentVisionFunctionValue("lineAbsent"):   #orange tape detected
            distance = rCV.getCurrentVisionFunctionValue("averages")[2]
            rect_area = rCV.getCurrentVisionFunctionValue("averages")[1]
            if (distance < dist_thresh or rect_area < area_thresh):  #checks to see if CV is reliably available for use
                self.cvInRange = True
            else:
                self.cvInRange = False

            return ptC.getCurrentAngle(ptC.getPanPin())

        if self.aiC.collisionCBtriggered:          #checks if imminent collision detected
            return 0.0

        #If we don't see a line, scan left
        print("Scanning left")
        if rCV.getCurrentVisionFunctionValue("lineAbsent"):
            #print("Looking left")
            ptC.setServo(ptC.getPanPin(), 90)
            for i in range(19):
                time.sleep(0.1)
                ptC.incServo(ptC.getPanPin(), 2)
                if not rCV.getCurrentVisionFunctionValue("lineAbsent"):   #orange tape detected
                    distance = rCV.getCurrentVisionFunctionValue("averages")[2]
                    rect_area = rCV.getCurrentVisionFunctionValue("averages")[1]
                    if (distance < dist_thresh or rect_area < area_thresh):  #checks to see if in range, and area is over threshold
                        self.cvInRange = True
                    else:
                        self.cvInRange = False  #CV is not yet available

                    print("Found orange")
                    speech.speak("orange, tape, found")
                    angle = ptC.getCurrentAngle(ptC.getPanPin())
                    ptC.setServo(ptC.getPanPin(), 90)
                    return angle


        if self.aiC.collisionCBtriggered:           #checks if imminent collision detected
            return 0.0

        print("Scanning right")
        #If we still don't see a line, scan right
        if rCV.getCurrentVisionFunctionValue("lineAbsent"):     #if orange tape not detected
            #print("Looking right")
            ptC.setServo(ptC.getPanPin(), 90)
            for i in range(19):                         #search for orange tape
                time.sleep(0.1)
                ptC.incServo(ptC.getPanPin(), -2)
                if not rCV.getCurrentVisionFunctionValue("lineAbsent"):   #orange tape found
                    distance = rCV.getCurrentVisionFunctionValue("averages")[2]
                    rect_area = rCV.getCurrentVisionFunctionValue("averages")[1]
                    if (distance < dist_thresh or rect_area < area_thresh):
                        self.cvInRange = True       #sets flag to showing that CV is avaiable
                    else:
                        self.cvInRange = False
                    print("Found orange")
                    speech.speak("orange, tape, found")
                    angle = ptC.getCurrentAngle(ptC.getPanPin())
                    ptC.setServo(ptC.getPanPin(), 90)
                    return angle

        if self.aiC.collisionCBtriggered:           #checks if imminent collision detected
            return 0.0

        ptC.setServo(ptC.getPanPin(), 90)
        print("Leaving cvscan, no orange found")
        #speech.speak("unable, to, find, orange, tape")
    '''
    ****************************************************
    *Function: getGreenInfo(greenDirs, greenAreas, greenWidths)
    *Description: gets the angles, area values, and width values
    for 1 or both green tapes that are detected
    *Parameters:
    *   self, greenDirs, greenAreas, greenWidths
    *Returns:
    *   greenDirL, greenDirR, greenAreaL, greenAreaR, greenWidthL, greenWidthR
    ****************************************************
    '''
    def getGreenInfo(self, greenDirs, greenAreas, greenWidths):
            greenDirL = max(greenDirs[0], greenDirs[1])   #set angle direction of green tape on left of base station
            greenDirR = min(greenDirs[0], greenDirs[1])    #set angle direction of green tape on right of base station
            indexL = 0 if (greenDirL == greenDirs[0]) else 1
            indexR = 0 if (greenDirR == greenDirs[0]) else 1
            greenAreaL = greenAreas[indexL]              #set pixel area of green tape on left of base station
            greenAreaR = greenAreas[indexR]              #set pixel area of green tape on left of base station
            greenWidthL = greenWidths[indexL]            #set pixel width of green tape on left of base station
            greenWidthR = greenWidths[indexR]            #set pixel width of green tape on right of base station

            return greenDirL, greenDirR, greenAreaL, greenAreaR, greenWidthL, greenWidthR

    '''
    ****************************************************
    *Function: whichSide(greenDirs, orangeDirection)
    *Description: determines which side of the base station the robot
    is on, from the angles of the green and orange tape, found by CV
    *Parameters:
    *   self, greenDirs, orangeDirection
    *Returns:
    *   self.GREEN_LEFT, self.GREEN_RIGHT
    ****************************************************
    '''
    def whichSide(self, greenDirs, orangeDirection):
        #If the green is left of orange
        if greenDirs[0] >= orangeDirection:
            return self.GREEN_LEFT
        else:
            return self.GREEN_RIGHT

        pass

    '''
    ****************************************************
    *Function: noColors(lineAbsent, numGreen)
    *Description: determines if no colored tapes are found
    *Parameters: self, lineAbsent, numGreen
    *Returns:
    *   True, False
    ****************************************************
    '''
    def noColors(self, lineAbsent, numGreen):
        if lineAbsent and numGreen == 0:
            return True
        else:
            return False

    '''
    ****************************************************
    *Function: alignWithGreen(rCV)
    *Description: aligns robot to the center of the green
    *Parameters: self, rCV, motors
    *Returns:
    *
    ****************************************************
    '''
    def alignWithGreen(self, rCV, motors):
        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
        if numGreen == 1:               #only 1 green tape visible on base station
            greenDirs = rCV.getCurrentVisionFunctionValue("greenDirections")
            print("Centering with green")
            while (abs(greenDirs[0]) >= 25) and not self.aiC.stopped:  #Try to get centered on the green, loop while offset is not within specified range
                angle = max(min(20, abs(greenDirs[0]) / 10), 5)  #set correction angle to center on green
                print("Centering: " + str(angle))
                if greenDirs[0] >= 0:         #checks if angle correction is positive, signifying that is located to our left
                    motors.turnAngleFunction(angle = angle)  #make the alignment correction, and turn left
                    motors.waitOnAction()
                elif greenDirs[0] < 0:        #checks if angle correction is negative, signifying that is located to our right
                    motors.turnAngleFunction(angle = -angle) #make the alignment correction, and turn right
                    motors.waitOnAction()
                time.sleep(0.5)
                greenDirs = rCV.getCurrentVisionFunctionValue("greenDirections") #get updated angle information

    '''
    ****************************************************
    *Function: wiggleIn(self, motors, rCV, sensors)
    *Description: Wiggles in to complete the dock, reversing and course correcting
    if a stall is detected during the procedure
    *Parameters: self, motors, rCV, sensors
    *Returns:
    *
    ****************************************************
    '''
    def wiggleIn(self, motors, rCV, sensors):
        #Next, wiggle forward
        wheelToUse = 1
        self.collisionCBenable = True         #used within aiState's findHomeState's alignCenter function
        while not self.detectCharge(sensors) and not self.aiC.stopped:
            #First, center with the orange
            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection") #sets direction for orange tape on base station
            print("Centering with orange")
            while abs(orangeDirection) >= 40 and not self.aiC.stopped:       #loops until deemed sufficiently centered with orange tape
                angle = min(10, abs(orangeDirection) / 10)
                if orangeDirection >= 0:            #orange tape is on left side
                    motors.turnAngleFunction(angle = angle, wheel = motors.RIGHT)   #correct alignment
                    motors.waitOnAction()
                elif orangeDirection < 0:           #orange tape is on right side
                    motors.turnAngleFunction(angle = -angle, wheel = motors.LEFT)   #turn right to correct alignment
                    motors.waitOnAction()
                time.sleep(0.5)
                orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection") #sample new orange direction

            rectArea = rCV.getCurrentVisionFunctionValue("rectArea")    #determine area of orange rectangle
            if abs(orangeDirection) < 50 or rectArea > 4500:   #if sufficiently centered, move forward 30 cm
                motors.forwardFunction(distance = 30)
                while not motors.currentOperation.complete and not self.aiC.stopped:   #check if contacts on system detect a charging state
                    time.sleep(0.1)
                    if self.detectCharge(sensors):
                        motors.halt()
                        self.aiC.popState()                                 #if charge detected, we are docked, pop state off stack
                        self.pushState(chargeState(self.aiC))               #go into charging state

            print("Wiggle " + str(wheelToUse))    #robot is within the arms of the base station, start wiggling in
            angle = 15 if wheelToUse == motors.RIGHT else -15 #15 degree turns in alternating directions
            motors.setStallState([False, False])            #reset the stall state of both wheels to false
            time.sleep(0.5)
            motors.turnAngleFunction(angle = angle, wheel = wheelToUse)
            while not motors.currentOperation.complete and not self.aiC.stopped:
                time.sleep(0.1)
                stallState = motors.getStallState()         #polls the stall state using motor controller for both wheels
                print("stallState is:" + str(stallState))
                if (stallState[0] == True and wheelToUse == motors.LEFT) or (stallState[1] == True and wheelToUse == motors.RIGHT):
                    motors.halt()                           #stall was detected during a wiggle, stop motors
                    motors.reverseFunction(distance = 2)
                    motors.waitOnAction()
                    if wheelToUse == motors.LEFT:           #if stall detected on left turn, turn right, away from obstacle
                        print("turning right")
                        motors.turnAngleFunction(angle = -angle, wheel = motors.LEFT)
                        motors.waitOnAction()
                    elif wheelToUse == motors.RIGHT:        #if stall detected on right turn, turn left, away from obstacle
                        print("turning left")
                        motors.turnAngleFunction(angle = angle, wheel = motors.RIGHT)
                        motors.waitOnAction()

                    motors.setStallState([False, False])    #reset stall state
                    time.sleep(0.5)
                    stallDetectedOnForward = True           #flag to determine if another stall was detected after wriggling phase
                    while stallDetectedOnForward and not self.aiC.stopped:
                        print ("stall detected during turn")
                        print("Halting")
                        motors.halt()                           #cancels turn function by halting motors
                        print("Moving backwards 4 cm")          #reverse motors to give room for course correction
                        motors.reverseFunction(distance = 4)
                        motors.waitOnAction()

                        motors.setStallState([False, False])
                        time.sleep(0.5)

                        print("Moving forward 3 cm")
                        motors.forwardFunction(distance = 3)    #move forward, try to engage base's contacts
                        while not motors.currentOperation.complete and not self.aiC.stopped:
                            time.sleep(0.1)
                            stallState = motors.getStallState()  #check if stall during forward movement
                            stallDetectedOnForward = False
                            if self.detectCharge(sensors):  #if charge detected during forward movement, stop, and change to chargingState
                                motors.halt()
                                self.aiC.popState()                                 #pop findHomeState off stack
                                self.pushState(chargeState(self.aiC))               #go into charging state
                            elif (stallState[0] == True) or (stallState[1] == True):  #another stall detected
                                stallDetectedOnForward = True                       #reset flag, so that wiggle loop continues
                                print("Stall detected on forward")
                                break

            time.sleep(0.1)
            wheelToUse ^= 1

    '''
    ****************************************************
    *Function: scan(self, rCV, ptC)
    *Description: Scans for line information using the pan tilt servo
    *Parameters: self
        rCV - reference to computer vision module
        ptC - reference to pan tilt controller
    *Returns:
    *   a dictionary with values that tell us about lines we have round.
    *   Entries:
        *   vals["orange"]["present"] - bool for orange line presence
            vals["orange"]["angle"] - angle of orange line
            vals["green"]["present"] - whether a green was found
            vals["green"]["twoFound"] - whether we saw two greens at once
            vals["green"]["angle"] - angle we found green
            vals["green"]["otherGreenFound"] - whether we found some other green
    ****************************************************
    '''
    def scan(self, rCV, ptC):
        print("Scan function")
        time.sleep(0.5)
        greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
        initialWidth = greenWidths[0]
        vals = {}
        orange = {}
        green = {}
        orange["present"] = False
        orange["angle"] = 0
        green["present"] = False
        green["twoFound"] = False
        green["angle"] = 0
        green["otherGreenFound"] = False
        vals["orange"] = orange
        vals["green"] = green

        #Looping twice, once for right, once for left
        for j in range(2):
            #Center camera
            ptC.setServo(ptC.getPanPin(), 90)
            #Sweep one direction or another
            for i in range(10):
                time.sleep(0.3)
                #Increment (or decrement) servo angle
                ptC.incServo(ptC.getPanPin(), (4 if j == 0 else -4))

                #If we found the orange
                if not rCV.getCurrentVisionFunctionValue("lineAbsent"):
                    vals["orange"]["present"] = True
                    vals["orange"]["angle"] = ptC.getCurrentAngle(ptC.getPanPin())
                    print("Found orange: " + str(ptC.getCurrentAngle(ptC.getPanPin())))
                    ptC.setServo(ptC.getPanPin(), 90)
                    return vals

                #If we found the two green
                if rCV.getCurrentVisionFunctionValue("numGreen") == 2:
                    vals["green"]["present"] = True
                    vals["green"]["twoFound"] = True
                    vals["green"]["angle"] = ptC.getCurrentAngle(ptC.getPanPin())
                    print("Found 2 green: " + str(ptC.getCurrentAngle(ptC.getPanPin())))
                    ptC.setServo(ptC.getPanPin(), 90)
                    return vals

                #Grab computer vision info
                greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
                widthsOrdered = self.smallToLarge(greenWidths[0], greenWidths[1])
                proportion = widthsOrdered[0] / initialWidth

                #If we've looked over at least 16 degrees, and we see 1 green, and its at least 15% smaller than the original
                if (i > 4) and (rCV.getCurrentVisionFunctionValue("numGreen") == 1) and (proportion < 0.85):
                    vals["green"]["present"] = True
                    vals["green"]["otherGreenFound"] = True
                    vals["green"]["angle"] = ptC.getCurrentAngle(ptC.getPanPin())
                    print("Found other green: Angle: " + str(ptC.getCurrentAngle(ptC.getPanPin())) + ", Proportion: " + str(proportion))
                    ptC.setServo(ptC.getPanPin(), 90)
                    return vals

        return vals

    '''
    ****************************************************
    *Function: alignCenter
    *Description: Uses information from the green and orange tapes on base station
    to realign the robot to an alignment where the wiggleIn function can be called to
    begin the docking procedure
    *Parameters: self
    *Returns:
    *
    ****************************************************
    '''
    def alignCenter(self):                         #uses CV to align robot with secondary recognition color in center of
        print("Align center")
        speech = self.getController(self.aiC.speechString)
        speech.speak("in, align, center")
        self.aiC.collisionCBenable = False         #disable collsionCB
        rCV = self.aiC.getController(self.aiC.rCVString)        #get access to controllers from aiController
        ptC = self.aiC.getController(self.aiC.panTiltString)
        sensors = self.aiC.getController(self.aiC.sensorString)
        motors = self.aiC.getController(self.aiC.motorsString)

        #Wait until we grab the image from the computer vision module
        rCV.setCurrentVisionFunction("BaseFinder")
        while rCV.getCurrentVisionFunctionValue("binarizedGreen") is None and rCV.getCurrentVisionFunctionValue("binarizedOrange") is None and not self.aiC.stopped:
            time.sleep(1.0 / 10.0)
            #print("Waiting for binarized image")
            continue

        #Pull our initial information for computer vision
        lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
        orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
        orangeDistance = rCV.getCurrentVisionFunctionValue("distance")
        orangeArea = rCV.getCurrentVisionFunctionValue("rectArea")

        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
        greenDirs = rCV.getCurrentVisionFunctionValue("greenDirections")
        greenAreas = rCV.getCurrentVisionFunctionValue("greenAreas")
        greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
        edgeDistance = rCV.getCurrentVisionFunctionValue("edgeDistance")

        #We're going to loop until we're charging, or until we lose sight of the colors
        while not self.detectCharge(sensors) and not self.noColors(lineAbsent, numGreen) and not self.aiC.stopped:
            print("In Aligning function")
            #Update our computer vision values every loop
            lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
            orangeDistance = rCV.getCurrentVisionFunctionValue("distance")
            orangeArea = rCV.getCurrentVisionFunctionValue("rectArea")

            numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
            greenDirs = rCV.getCurrentVisionFunctionValue("greenDirections")
            greenAreas = rCV.getCurrentVisionFunctionValue("greenAreas")
            greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
            edgeDistance = rCV.getCurrentVisionFunctionValue("edgeDistance")

            #Calculate certain info about what we see, in case we see both greens
            #and the orange
            greenDirL, greenDirR, greenAreaL, greenAreaR, greenWidthL, greenWidthR = self.getGreenInfo(greenDirs, greenAreas, greenWidths)

            if numGreen == 0:               #only orange tape visible (Case1)
                print("Numgreen 0, only orange. Wiggling in.")
                speech.speak("only, orange, tape, visible.")
                speech.speak("now, docking")

                self.wiggleIn(motors, rCV, sensors)

            elif lineAbsent and numGreen == 2:                     #only both greens visible (Case2)
                print("No orange, just 2 greens.")
                speech.speak("only, two, greens, found")
                greenNum = self.smallToLarge(greenAreaL, greenAreaR)[0]
                greenDenom = self.smallToLarge(greenAreaL, greenAreaR)[1]
                if ((greenAreaR < greenAreaL) and ((greenNum / greenDenom) < 0.85)):        #greenAreaL is less than greenAreaR, and they have a greater than 15% diff
                    print("Right green smaller than left. Greater than 15 perc diff")
                    turnAngle = 2 * abs((greenDirL - greenDirR)/2 + greenDirR)
                    motors.turnAngleFunction(angle = -0.85 * turnAngle)     #correct alignment, the adjustment is weighted factoring in the apparent distances to each
                    motors.waitOnAction()                                   #green tape
                    motors.forwardFunction(distance = 8)                    #go forward after alignment correction
                    motors.waitOnAction()
                    motors.turnAngleFunction(angle = 0.75 * turnAngle)   #scale down recorrect turn, smaller than initial turning
                    motors.waitOnAction()
                elif ((greenAreaL < greenAreaR) and ((greenNum / greenDenom) < 0.85)):  ##case where closer to green tape on right
                    print("Right green larger than left. Greater than 15 perc diff")
                    motors.turnAngleFunction(angle = 0.85 * turnAngle)
                    motors.waitOnAction()
                    motors.forwardFunction(distance = 8)
                    motors.waitOnAction()
                    motors.turnAngleFunction(angle = -0.75 * turnAngle)   #scale down recorrect turn, smaller than initial turning
                    motors.waitOnAction()
                else:
                    print("Less than 15 perc diff")                                 #if greenAreaL and greenAreaR have a less than 15% difference
                    motors.turnAngleFunction(angle = 0.85 * (2*((greenDirL - greenDirR)/2 + greenDirR)))
                    motors.waitOnAction()
                    motors.forwardFunction(distance = 15)
                    motors.waitOnAction()
            elif lineAbsent and numGreen == 1:              #only 1 green visible (Case3)
                print("No orange, one green")
                speech.speak("only, one, green, visible.")
                #line up with currently viewable green
                numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                time.sleep(0.5)
                self.alignWithGreen(rCV, motors)

                print("Calling scan")
                speech.speak("scanning")
                #call scan function
                vals = self.scan(rCV, ptC)
                #if both greens visible:
                if vals["green"]["present"] and vals["green"]["twoFound"]:
                    print("Green present, two found")
                    #turn so that aligned with both greens
                    angle = vals["green"]["angle"]
                    print("Turning towards 2nd green")
                    #speech.speak("turning, toward, green")
                    motors.turnAngleFunction(angle = angle)
                    motors.waitOnAction()

                    pass
                #elif other green is visible, (the other green width is smaller)
                elif vals["green"]["present"] and vals["green"]["otherGreenFound"]:
                    print("Green present, other green found with smaller width")

                    angle = vals["green"]["angle"]
                    print("Turning towards 2nd green")
                    #speech.speak("turning, toward, green")
                    motors.turnAngleFunction(angle = angle)  #on right side, so turning left
                    motors.waitOnAction()
                    time.sleep(0.5)

                    #turn so that aligned with smaller GW green
                    self.alignWithGreen(rCV, motors)
                    #move forward 15
                    print("Moving forward 15 cm")
                    motors.forwardFunction(distance = 15)
                    motors.waitOnAction()
                    time.sleep(0.5)

                    pass
                #elif orange is visible
                elif vals["orange"]["present"]:
                    print("Found orange")
                    speech.speak("orange, found")

                    #turn so that aligned with both green and orange
                    angle = vals["orange"]["angle"]
                    time.sleep(0.5)
                    lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
                    numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                    #Turn towards orange if it was found
                    print("Turning toward orange")
                    if lineAbsent and numGreen == 1:
                        motors.turnAngleFunction(angle = angle)
                        motors.waitOnAction()
                        time.sleep(0.2)
                    pass
                #else  (only original green visible)
                else:
                    print("Couldn't find another green")
                    #turn left 45, get ultrasonic value
                    print("Turning left 45*")
                    motors.turnAngleFunction(angle = 45)
                    motors.waitOnAction()
                    time.sleep(0.5)
                    speech.speak("using, sonar, to, check, for wall")
                    dist = sensors.getUSDistance()  #checking to see if wall is on our left or right
                    #if distance < 80
                    if dist < 80:                   #wall is on our left
                        print("US distance < 80 on turnAngleFunction")
                        #turn back toward green and center on it
                        print("Turning right 45")
                        motors.turnAngleFunction(angle = -45)
                        motors.waitOnAction()
                        time.sleep(0.5)
                        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                        self.alignWithGreen(rCV, motors)
                        #turn right 45
                        print("Turning right 45")
                        motors.turnAngleFunction(angle = -45)   #approaching from left side of base station, so turn right
                        motors.waitOnAction()
                        time.sleep(0.5)
                        #move forward 45
                        print("Moving forward 45 cm")           #attempt to clear left arm of base station
                        motors.forwardFunction(distance = 45)
                        #turn left 45
                        print("Turning left 45")                #correct alignment to face base station
                        motors.turnAngleFunction(angle = 45)
                        pass
                    #elif distance >=80
                    else:
                        print("US distance >= 80")              #ultrasonic sensor did not detect a wall on our left
                        #turn right, back toward green and center on it.
                        speech.speak("no, wall, detected")
                        print("Turning right 45")               #turn back to original alignment
                        motors.turnAngleFunction(angle = -45)
                        motors.waitOnAction()
                        time.sleep(0.5)
                        self.alignWithGreen(rCV, motors)        #align with the green tape that we saw before
                        #turn right 45, get ultrasonic values
                        print("Turning right 45")               #turn towards where the wall should be, if we are approaching from right
                        motors.turnAngleFunction(angle = -45)
                        motors.waitOnAction()
                        time.sleep(0.5)
                        speech.speak("using, sonar, to, check, for wall")
                        dist = sensors.getUSDistance()          #check for wall on right of base station
                        #if distance < 80
                        if dist < 80:                           #wall was detected, we are approaching from right side

                            print("Now US distance < 80 on right")
                            #turn back toward green and center on it
                            print("Turning left 45")            #turn back
                            motors.turnAngleFunction(angle = 45)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            self.alignWithGreen(rCV, motors)    #align again with the green tape that we saw
                            #turn left 45
                            print("turning left 45")
                            motors.turnAngleFunction(angle = 45) #turn left, to avoid right arm of base station
                            motors.waitOnAction()
                            time.sleep(0.5)
                            #move forward 45
                            print("Forward 45 cm")               #attempt to clear arm
                            motors.forwardFunction(distance = 45, speed = 20)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            #turn right 45
                            print("turning right 45")               #correct alignment to face base station
                            motors.turnAngleFunction(angle = -45)
                            motors.waitOnAction()
                            time.sleep(0.5)

                            pass
                        else:
                            pass
                        pass
                    pass
                pass

            elif not lineAbsent and numGreen == 1:          #orange and only 1 green visible (Case4)
                print("Orange and 1 green found")
                speech.speak("orange, and, one, green, tape, found")
                #First case: ed is smallest value (ED <=220)
                if edgeDistance <= 220:                     #edge of the orange tape and green tape is relatively close
                    print("Edge distance <= 220")

                    greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths") #now check the width of the green tape

                    #If green is really small
                    if greenWidths[0] <= 15:
                        print("Green width <= 15")
                        #Move forward a bit
                        motors.forwardFunction(distance = 10)
                        motors.waitOnAction()

                    #Find which side we're looking at
                    side = self.whichSide(greenDirs, orangeDirection)
                    if side == self.GREEN_LEFT: #We're looking at the left side
                        print("Green is left of orange")
                        #turn until centered on other green
                        print("Turning to lose sight of orange")
                        llineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
                        while not lineAbsent: #First, turn right until we lost sight of our green
                            motors.turnAngleFunction(angle = -5)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")


                        print("Turning to find right side green")
                        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                        while numGreen < 1 and not self.aiC.stopped: #Second, turn right until we find our right side green
                            motors.turnAngleFunction(angle = -10)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            numGreen = rCV.getCurrentVisionFunctionValue("numGreen")

                        self.alignWithGreen(rCV, motors)

                        #move forward until GW is >= 20
                        print("Moving forward until gw >= 20")
                        greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
                        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")  #the number of green tapes that are visible
                        while (greenWidths[0] < 20) and numGreen == 1 and not self.aiC.stopped: #width of the green tape is small, so move closer
                            print("Width: " + str(greenWidths[0]))
                            dist = 2
                            motors.forwardFunction(distance = dist)
                            motors.waitOnAction()
                            if self.detectCharge(sensors):            #if during this process we docked, and detect charge
                                break
                            time.sleep(1.5)
                            greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")  #get updated width value
                            numGreen = rCV.getCurrentVisionFunctionValue("numGreen")

                        if self.detectCharge(sensors):
                            break

                        #turning until centered on orange
                        print("Turning left until orange")
                        lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")  #no orange tape visible
                        while lineAbsent and not self.aiC.stopped: #Find the orange on the left
                            motors.turnAngleFunction(angle = 10)    #turn in small increments until orange tape is detected
                            motors.waitOnAction()
                            time.sleep(0.5)
                            lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")

                        #Try to center with the orange
                        print("Centering with orange")
                        #speech.speak("centering, on, orange, tape")
                        orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                        while abs(orangeDirection) >= 40 and not self.aiC.stopped:       #turn until orange direction is within specified range
                            angle = min(20, abs(orangeDirection) / 10)
                            if orangeDirection >= 0:
                                motors.turnAngleFunction(angle = angle)  #turn left, if orangeDirection is positive
                                motors.waitOnAction()
                            elif orangeDirection < 0:
                                motors.turnAngleFunction(angle = -angle) #turn right, if orangeDirection is negative
                                motors.waitOnAction()
                            time.sleep(0.5)
                            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")

                    else: #We're looking at the right side,Green is right of orange
                        print("Green is right of orange")
                        print("turning left until lose sight of orange")
                        #turn until centered on other green
                        lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
                        while not lineAbsent and not self.aiC.stopped:  #First, turn left until we lost sight of our green
                            print("inside while loop")
                            motors.turnAngleFunction(angle = 5)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")

                        print("Turning left until find left side green")
                        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                        while numGreen < 1 and not self.aiC.stopped: #Second, turn left until we find our right side green
                            motors.turnAngleFunction(angle = 10)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            numGreen = rCV.getCurrentVisionFunctionValue("numGreen")

                        #Align the robot with the green
                        self.alignWithGreen(rCV, motors)

                        #move forward until GW is >= 20
                        print("Move forward until gw >= 20")
                        greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
                        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                        while (greenWidths[0] < 20) and numGreen == 1 and not self.aiC.stopped:   #loop until green is relatively close, and still only 1 green visible
                            print("Width: " + str(greenWidths[0]))
                            dist = 2
                            motors.forwardFunction(distance = dist)  #move forward at increments of 2 cm, until green is close
                            motors.waitOnAction()
                            if self.detectCharge(sensors):  #if charge detected during movement, break out
                                break
                            time.sleep(1.5)
                            greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths") #update CV info
                            numGreen = rCV.getCurrentVisionFunctionValue("numGreen")

                        if self.detectCharge(sensors):
                            break

                        #turning until centered on orange
                        print("turning right looking for orange")
                        lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
                        while lineAbsent and not self.aiC.stopped: #Find the orange on the right
                            motors.turnAngleFunction(angle = -10)
                            motors.waitOnAction()
                            time.sleep(0.5)
                            lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")

                        #Try to center with the orange
                        print("Orange found. Try to center with orange")
                        speech.speak("orange, tape, found")
                        orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                        while abs(orangeDirection) >= 40 and not self.aiC.stopped:    #continue to align until centered on oragne
                            angle = min(20, abs(orangeDirection) / 10)
                            if orangeDirection >= 0:           #orange is to the left of us, so turn left
                                motors.turnAngleFunction(angle = angle)
                                motors.waitOnAction()
                            elif orangeDirection < 0:          #orange is to the right of us, so turn right
                                motors.turnAngleFunction(angle = -angle)
                                motors.waitOnAction()
                            time.sleep(0.5)
                            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")

                #Second case: ed is largest value (ED > 220)    turn 30, forward, turn back 10
                elif edgeDistance > 220:
                    print("Edge distance > 220")
                    greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
                    #second.1 size is smallest (GW <= 20)
                    side = self.whichSide(greenDirs, orangeDirection)

                    if greenWidths[0] <= 20:
                        print("Green width <= 20")
                        #align green and camera
                        self.alignWithGreen(rCV, motors)

                        #move forward until GW >= 40
                        print("Move forward until gw >= 20")
                        greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
                        numGreen = rCV.getCurrentVisionFunctionValue("numGreen")
                        while (greenWidths[0] < 20) and numGreen == 1 and not self.aiC.stopped:  #while green is still far, and only 1 green visible
                            print("Width: " + str(greenWidths[0]))
                            dist = 2
                            motors.forwardFunction(distance = dist)    #move foward in increments of 2 cm
                            motors.waitOnAction()
                            if self.detectCharge(sensors):
                                break
                            time.sleep(1.5)
                            greenWidths = rCV.getCurrentVisionFunctionValue("greenWidths")
                            numGreen = rCV.getCurrentVisionFunctionValue("numGreen")

                        if self.detectCharge(sensors):    #charge detected, break out of loop
                            break
                        #if leftside case
                        if side == self.GREEN_LEFT:
                            print("leftside case")
                            #turn right until orange centered
                            #Try to center with the orange
                            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                            print("turning right to line up with orange")
                            while abs(orangeDirection) >= 40 and not self.aiC.stopped:   #center on orange
                                angle = min(20, abs(orangeDirection) / 10)
                                motors.turnAngleFunction(angle = -angle)
                                motors.waitOnAction()
                                time.sleep(0.5)
                                orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")

                        #right side case
                        else:
                            print("Right side case")
                            #turning LEFT until centered on orange
                            #Try to center with the orange
                            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                            print("Turning left to line up with orange")
                            while abs(orangeDirection) >= 40 and not self.aiC.stopped:   #center on orange
                                angle = min(20, abs(orangeDirection) / 10)
                                motors.turnAngleFunction(angle = angle)
                                motors.waitOnAction()
                                time.sleep(0.5)
                                orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")

                    #second.2 GW > 20
                    else:
                        print("Green width > 20")
                        #determine which side of approach
                        side = self.whichSide(greenDirs, orangeDirection)
                        #if leftside
                        if side == self.GREEN_LEFT:
                            print("left side case")
                            #turn right until orange centered
                            print("Turn right until orange centered")
                            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                            while abs(orangeDirection) >= 40 and not self.aiC.stopped:
                                angle = min(20, abs(orangeDirection) / 10)
                                motors.turnAngleFunction(angle = -angle)
                                motors.waitOnAction()
                                time.sleep(0.5)
                                orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                        #else if rightside
                        else:
                            print("right side case")
                            #turn left until orange centered
                            print("Turn left until orange centered")
                            orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")

                            while abs(orangeDirection) >= 40 and not self.aiC.stopped:
                                angle = min(20, abs(orangeDirection) / 10)  #set turning angle
                                motors.turnAngleFunction(angle = angle)
                                motors.waitOnAction()
                                time.sleep(0.5)
                                orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")

            elif not lineAbsent and numGreen == 2:          #orange and only 2 green visible (Case5)
                print("I see orange and 2 green")
                speech.speak("orange, and, both, green, tapes, found")
                greenNum = self.smallToLarge(greenAreaL, greenAreaR)[0]  #smaller of the green areas
                greenDenom = self.smallToLarge(greenAreaL, greenAreaR)[1] #larger of the green areas
                if (abs(greenNum / greenDenom)) >= 0.85:  #proportion of green areas tells us we are roughly centered between them
                    print("Green roughly the same size")
                    #turn direction of odir
                    print("lining up with orange")
                    orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                    while abs(orangeDirection) >= 40 and not self.aiC.stopped:   #center on orange between the green tape
                        angle = min(20, abs(orangeDirection) / 10)
                        if orangeDirection >= 0:    #turn left if orange tape is to our left
                            motors.turnAngleFunction(angle = angle)
                            motors.waitOnAction()
                        elif orangeDirection < 0:   #turn right if orange tape is to our right
                            motors.turnAngleFunction(angle = -angle)
                            motors.waitOnAction()
                        time.sleep(0.5)
                        orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                    #move forwardFunction
                    print("Forward 10 cm")
                    motors.forwardFunction(distance = 10)  #if centered on orange, go forward
                    motors.waitOnAction()
                else:   #the proportion of green areas, lets us know that we are not in the center of them
                    print("Green size diff > 15 percent")
                    angleConstant = 10 #calibrate

                    #We need to decide which direction to turn, based on the
                    #proportion of left gap to right gap
                    orangeDirection = rCV.getCurrentVisionFunctionValue("orangeDirection")
                    if abs(greenDirL - orangeDirection) < abs(greenDirR - orangeDirection): #we are more to the left of base
                        print("Distance from left to orange < dist from righ to orange")
                        #Find the numerator term of distances (smaller gap)
                        numerator = self.smallToLarge((greenDirR - orangeDirection), (greenDirL - orangeDirection))[0]
                        #Find larger gap distance
                        denom = self.smallToLarge((greenDirR - orangeDirection), (greenDirL - orangeDirection))[1]
                        #Turn some angle proportional to the gap distance proportion
                        turnAngle = angleConstant * numerator/denom
                        print("turning right " + str(turnAngle))
                        motors.turnAngleFunction(angle = -0.85 * turnAngle)  #turn right scaled turnAngleFunction
                        motors.waitOnAction()
                    else:
                        print("Distance from left to orange >= dist from right to orange")
                        numerator = self.smallToLarge((greenDirL - orangeDirection), (greenDirR - orangeDirection))[0]
                        denom = self.smallToLarge((greenDirL - orangeDirection), (greenDirR - orangeDirection))[1]
                        #Same as previous case, in the opposite direction
                        turnAngle = angleConstant * numerator/denom
                        #turnAngle = angleConstant * abs((greenDirL - orangeDirection) / (greenDirR - orangeDirection))
                        print("Turning left " + str(turnAngle))
                        motors.turnAngleFunction(angle = 0.85 * turnAngle)  #turn left scaled turnAngleFunction
                        motors.waitOnAction()

            #If we detect charge at some point, lets go to charge state.
            if self.detectCharge(sensors):

                motors.halt()
                self.aiC.popState()                                 #pop avoidanceState off stack
                self.pushState(chargeState(self.aiC))

    '''
    ****************************************************
    *Function: smallToLarge
    *Description: takes in 2 numbers, and returns them as a tuple
    that is order from smallest to largest
    *Parameters: self, float, float
    *Returns: tuple[float, float]
    *
    ****************************************************
    '''
    def smallToLarge(self, num1, num2):                     #takes in 2 numbers, orders from abs of least to greatest
        if abs(num1) < abs(num2):
            return (abs(num1),  abs(num2))                             #returns the absolute value'd tuple
        elif abs(num2) <= abs(num1):
            return (abs(num2), abs(num1))

    '''
    ****************************************************
    *Function: detectCharge
    *Description: Calls isCharging() function within the sensorsController
    to detect if there is a charge, until the current motor operation is completed
    *Parameters: self, sensors
    *Returns: True, False
    *
    ****************************************************
    '''
    def detectCharge(self, sensors):
        print("inside of detectCharge")
        time.sleep(0.1)
        speech = self.getController(self.aiC.speechString)
        motors = self.aiC.getController(self.aiC.motorsString) #get access to motorsController

        while not motors.currentOperation.complete and not self.aiC.stopped:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            if sensors.isCharging():   #poll to detect charge
                speech.speak("charge, detected")
                return True

        if sensors.isCharging():
            speech.speak("charge, detected")
            return True

        return False

    '''
    ****************************************************
    *Function: dock
    *Description: Alternative approach to docking into base station,
    similar to wiggleIn
    *Parameters: self,motors, sensors
    *
    *Returns:
    *
    ****************************************************
    '''
    def dock(self, motors, sensors):                                 #docking procedure

        print("in dock")
        self.dockFlag = True   #flag used so that we cannot revert back to IR or CV methods of approaching base
        chargeDetected = False
        time.sleep(2)
        motors.forwardFunction(distance = 10) #move forward, at this point we are centered with orange tape
        motors.waitOnAction()

        chargeDetected = self.detectCharge(sensors)
        toggleBit = 1

        while not chargeDetected and not self.aiC.stopped:  #loops while charge is not detected
            wheelToUse = motors.LEFT if (toggleBit == 1) else motors.RIGHT
            angle = -38 if (toggleBit == 1) else 38   #turn 45 degrees
            motors.turnAngleFunction(angle = angle, wheel = wheelToUse)
            if self.detectCharge(sensors):
                chargeDetected = True
                break

            toggleBit ^= 1           #toggle between turning left or turning right
            #wiggle up
        motors.halt()               #stop motors, charge detected
        self.aiC.popState()                                 #pop findHomeState off stack
        self.pushState(chargeState(self.aiC))               #push chargeState onto stack

    '''
    ****************************************************
    *Function: getAvgIRAngle
    *Description: Creates a stack and populates it with 5 IR values, that can either be 90 degrees
    (robot is aligned directly with transmitter), 135 (transmitter is 45 degrees to our right),
     180 (90 degrees to our right), 45 (45 degrees to our left), 0 (90 to our left), 200
     (a value signifying an invalid value), 300 (signifies that at least 3 IR
    sensors are receiving a signal). Processes the array to take average of only valid values, or reutrns
    a value, if that value occurs more than thrice in the capacity 5 array
    *Parameters: self, sensors
    *Returns: integer
    *
    ****************************************************
    '''
    def getAvgIRAngle (self, sensors):

        for i in xrange(5):                                  #get 5 IR angle values
            self.irAngle = sensors.getIRAngle()                     #get IR angle
            self.angleStack.appendleft(self.irAngle)         #populate stack (max size 5) with IR angle values
            time.sleep(0.03)

        #print ("stack elements: " + str(self.angleStack))    #check if properly populating
        count200 = 0                                         #initialize count variables to zero
        count300 = 0                                         #initialize count variables to zero
        countA = 0                                           #initialize count variables to zero
        countB = 0                                           #initialize count variables to zero
        countC = 0                                           #initialize count variables to zero
        total = 0                                            #initialize count variables to zero
        a = self.angleStack[0]                               #compare to first value value of stack
        b = self.angleStack[1]
        c = self.angleStack[2]

        for j in xrange(5):
            time.sleep(0.03)                                #iterage through each element of stack
            if self.angleStack[j] == 200:                    #check for not valid value ()'200')
                count200 += 1                                #count occurences of 200
                self.angleStack[j] = 0                      #replace invalid value with zero for averaging purposes
            elif self.angleStack[j] == 300:                  #check for 3+ IR sensors returning values ('300')
                count300 += 1
                self.angleStack[j] = 0
            else:                                            #if not '200' or '300'
                if self.angleStack[j] == a:                  #count occurences of first element
                    countA += 1
                elif self.angleStack[j] == b:
                    countB += 1                              #count occurences of second element
                elif self.angleStack[j] == c:
                    countC += 1                             #count occurences of third element
            total = total + self.angleStack[j]              #keep running total of valid IR angles

        if (count200 + count300) > 2:                       #check if invalid number has median/majority value
            self.irAngle = 200                              #set irAngle to invalid number if majority value
        elif count300 > 2:                                  #check if 300 is majority/medianvalue
            self.irAngle = 300
        elif countA > 2:                                    #check if value A is majority/median value
            self.irAngle = a                                #if so, set irAngle to a
        elif countB > 2:                                    #check if value A is majority/median value
            self.irAngle = b
        elif countC > 2:                                    #check if value A is majority/median value
            self.irAngle = c
        else:                                               #if there is no single angle that has a majority value
            self.irAngle = total / (5 - count200 - count300)  #set irAngle as the average of non-200 or non-300 values

    '''
    ****************************************************
    *Function: update
    *Description: Runs the overall protocol to navigate
    the robot back to the base station
    *Parameters: self
    *
    *Returns:
    *
    ****************************************************
    '''
    def update(self):
        print("in findHomeState")
        print("scanning for IR signal")
        speech = self.getController(self.aiC.speechString)

        sensors = self.getController(self.aiC.sensorString)  #for easier access to sensorController's functions
        motors = self.getController(self.aiC.motorsString)   #set variable for easier access to MotorController
        speech = self.getController(self.aiC.speechString)
        invalidCount = 0                                     #used to determine how many successive times invalid ir angle polled

        self.irBeamLocked = False
        speech.speak("scanning, eye, are, signal")
        while not self.irBeamLocked and not self.dockFlag and not self.aiC.stopped:                         #corrects orientation until lined up with transmitter
            time.sleep( 0.1 )

            self.getAvgIRAngle(sensors)                      #gets average or majority value of polled IR angles

            print("IR average value: " + str(self.irAngle))

            if self.irAngle is None:
                print("ir Angle equals none")

            elif self.irAngle == 90:                           #robot is lined up with base station
                self.irBeamLocked = True

            elif self.irAngle < 90:                                  #transmitter is offcenter to the right
                self.correctionAngle = ((self.irAngle - 90) * 0.80)        #calculate angle needed to line up with ir transmitter
                print("Angle < 90")
                print ("Correction angle is: " + str(self.correctionAngle))                                                     #extra 20 degrees added to compensate for turnAngleFunction
                                                                    #may need recalibration
                motors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.waitOnAction()                                    #wait for turn

            elif self.irAngle > 90 and self.irAngle != 200 and self.irAngle != 300:                                  #transmitter is off center to the left
                #self.correctionAngle = ((self.irAngle*0.80) - 90)        #calculate angle needed to line up with ir transmittermotors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                self.correctionAngle = ((self.irAngle - 90) * 0.80)
                print("angle > 90")
                print ("Correction angle is: " + str(self.correctionAngle))
                motors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.waitOnAction()                                    #wait for turn

            elif self.irAngle == 300:                               #it is possible that robot is in very close proximity to transmitter
                print("ir angle = 300")
                check300 = 2                                        #check '300' value three times, to better verify that we are close to transmitter
                while check300 != 0 and not self.aiC.stopped:
                    print("inside second while loop")
                    print ("Check300 is: " + str(check300))
                    time.sleep(0.1)
                    print("moving forward 2 cm")
                    motors.forwardFunction(distance = 2)                #move slightly forward to reassess if '300' is a legitimate value
                    motors.waitOnAction()
                    self.getAvgIRAngle(sensors)                         #gets average or majority value of polled IR angles
                    if self.irAngle != 300:                             #check IR agnle again
                        check300 = 0                                    #if 300 value is not confirmed, reset counter and break
                        break
                    else:
                        check300 -= 1                                   #iterate again to again verify that value is 300

                if self.irAngle == 300:                                        #cvScan function will set cvInRange as 'True' if CV is in range
                    print("calling cvScan from elif 300")
                    self.cvScan()                                                #check proximity of ir transmitter, using cvScan function
                    if self.cvInRange:
                        self.irBeamLocked = False
                        break

            elif self.irAngle == 200:
                print("ir Angle = 200")
                print ("invalidCount: " + str(invalidCount))
                print("invalid IR value")              #invalid angle value received
                if invalidCount < 6:
                    print("turning to search for IR signal")                             #if invalid, try turning 90 degrees twice before moving lovation
                    motors.turnAngleFunction(angle = 70)             #turn 90 degrees left and rescan
                    motors.waitOnAction()
                    invalidCount += 1                                #increment invalid angle value Counter
                else:
                    print("moving to new location in search of IR signal")
                    motors.forwardFunction(distance = 25)            #if invalid angle value detected thru 270 degrees
                    motors.waitOnAction()                            #move forward to  change location
                    motors.turnAngleFunction(angle = 30)             #turn 45 degrees
                    motors.waitOnAction()

            print("end of while loop")

        print("outside of while loop")
        print("IR average value: " + str(self.irAngle))

        if self.irBeamLocked == True and not self.dockFlag:                               #outside of while loop, double check that IR beam is locked
            print("irBeam is locked")
            speech.speak("eye. are. beam. centered.")
            self.cvInRange = False
            while self.irAngle == 90 and self.cvInRange == False and not self.aiC.stopped:   #keeps moving forward as long as aligned with transmitter,
                motors.forwardFunction(distance = 30)               # and CV is not in range to assist yet
                if self.detectCharge(sensors):
                    self.aiC.popstate()
                    self.pushState(chargeState(self.aiC))
                print("moving forward")
                motors.halt()
                time.sleep(0.2)
                self.getAvgIRAngle(sensors)                         #gets average or majority value of polled IR angles
                self.cvScan()                                      #cvScan function
                print("executing cvScan")

        if self.cvInRange == True and not self.dockFlag:                                  #within close enough proximity to transmitter to use CV to align
            while not self.alignedWithCenter and not self.aiC.stopped:
                self.aiC.collisionCBenable = False
                self.alignCenter()                                      #call alignCenter function, to align with center of base station

        if self.alignedWithCenter == True and not self.dockFlag:                          #if aligned with center of base station, begin docking routine
            self.dock(motors, sensors)

        if sensors.isCharging():
            self.aiC.popState()                                 #pop findHome off stack
            self.pushState(chargeState(self.aiC))

                                     # set self.irBeamLocked back to 'False' and self.cvInRange back to 'False' before ending function
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

    '''
    ****************************************************
    *Function: update
    *Description: runs the protocol for the robot to stay in
    charging state and switch back to lineFollowState when done charging
    *Parameters: self
    *
    *Returns:
    *
    ****************************************************
    '''
    def update(self):
        print ("in chargeState's update")
        speech = self.getController(self.aiC.speechString)
        speech.speak("Charging. system.")

        sensors = self.getController(self.aiC.sensorString)  #variable for easier access to sensors controller

        while not chargeComplete and not self.aiC.stopped:   #loops until chargeComplete is True
            time.sleep(120)
            battVoltage = sensors.getBatteryVoltage()     #polls battery voltage from sensors thread
            if battVoltage == 3.40:
                speech.sayVoltage(battVoltage)
            elif battVoltage == 3.50:
                speech.sayVoltage(battVoltage)
            elif battVoltage == 3.60:
                speech.sayVoltage(battVoltage)
            elif battVoltage == 3.70:
                speech.sayVoltage(battVoltage)
            elif battVoltage == 3.80:
                speech.sayVoltage(battVoltage)
            elif battVoltage == 3.90:
                speech.sayVoltage(battVoltage)
            elif battVoltage == 4.00:
                speech.sayVoltage(battVoltage)

            if battVoltage >= 4.00:                       #charge complete at battery voltage = 4 volts
                chargeComplete = True                     #set flag to end while loop

        motors = self.getController(self.aiC.motorsString)
        motors.reverseFunction(distance = 40)             #reverse out of base station
        motors.waitOnAction()                             #wait until reverse function has completed
        motors.turnAngleFunction(angle = -153)            #reorient in opposite direction
        motors.waitOnAction()                             #wait for turn

        self.aiC.voltageCBtriggered = False
        self.aiC.collisionCBenable = True                 #reenable avoidanceState
        self.aiC.clearStack()
        self.pushState(lineFollowState(self.aiC))
                                   #pop chargeState off stack

        return

'''
*	Class: AvoidanceState
*	Description: lineFollowState is a child class of aiState. After receiving a critical value alert from the ultrasonic sensor,
    sensorsController calls the collisionCB function within aiController class which will push AvoidanceState onto the stack.
    AvoidanceState directs the robot to reorient itself away from the obstacle that the ultrasonic sensor has detected.
*	Author:       Michael Lee and Austin Dibble
*	Date Created:	2/21/18
'''
#TODO fix this freeze again
class avoidanceState(aiState):

    def __init__(self, aiC):                            #constructor for avoidanceState
        aiState.__init__(self, aiC)
        self.turnObstruct = False                       #default value for variable representing if a turn attempt was obstructed

    '''
    ****************************************************
    *Function: detectTurnStall
    *Description: detects if there was a stall during the current motor operation.
    Sets the turnObstruct variable to be true if so.
    *Parameters: self,motors, pivotWheel
    *
    *Returns:
    *
    ****************************************************
    '''
    def detectTurnStall(self, motors, pivotWheel):      #function to determine if there was turn obstruction
        print("inside of detectTurnStall")
        self.turnObstruct = False
        motors.setStallState([False, False])            #reset the stall state of both wheels to false
        time.sleep(0.25)

        while not motors.currentOperation.complete and not self.aiC.stopped:     #loops while turnAngleFunction command has not completed
            time.sleep(0.2)
            stallState = motors.getStallState()         #polls the stall state using motor controller for both wheels
            print("Stallstate: " + str(stallState))
            print("Pivotwheel: " + str(pivotWheel))

            if (stallState[0] == True and pivotWheel == motors.RIGHT) or (stallState[1] == True and pivotWheel == motors.LEFT):
                if not motors.currentOperation.complete:    #checks if a non-pivot wheel stalled during turn
                    print ("stall detected during turn")
                    self.turnObstruct = True                #sets variable to true, signifying stall has occured
                    motors.halt()                           #cancels turn function by halting motors
                    break

        #motors.forwardFunction(distance = 25)               #move forward 25 cm
        #motors.waitOnAction()

    '''
    ****************************************************
    *Function: oneEighty
    *Description: Reverses the alignment of the robot by 180 degrees
    *Parameters: self,motors
    *
    *Returns:
    *
    ****************************************************
    '''
    def oneEighty(self, motors):                            #reverse and turn around 180 degrees
        motors.reverseFunction(distance = 30)               #possible corner, reverse
        motors.waitOnAction()                               #wait untill reverse function has completed
        motors.turnAngleFunction(angle = -153)              #reorient in opposite direction
        motors.waitOnAction()

    '''
    ****************************************************
    *Function: update
    *Description: Runs the procedure for avoidance state to be
    able to avoid an obstacle
    *Parameters: self
    *
    *Returns:
    *
    ****************************************************
    '''
    def update(self):                                       #sets routine for obstacle avoidance
        print ("in avoidanceState's update")
        speech = self.getController(self.aiC.speechString)
        speech.speak("Obstacle, avoidance, mode")
        self.aiC.collisionCBtriggered = True
        motors = self.getController(self.aiC.motorsString)  #set variable for easier access to MotorController
        print("turning left 70 degrees, pivot on right wheel")
        motors.turnAngleFunction(angle = 60, wheel = motors.LEFT) #turn command for a 70 degree left turn, pivoting on right wheel

        self.detectTurnStall(motors, motors.RIGHT)          #calls function to determine if stall occurs during turn


        if self.turnObstruct == True:                       #if stall has occurred during turn
            print("stall detected during first turn")
            #speech.speak("Stall. detected")
            time.sleep(0.1)
            self.turnObstruct = False                       #reset variable to default
            motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)  #turn 120 degrees in the other direction, pivot on left
            self.detectTurnStall(motors, motors.LEFT)       #detect if stall occurs during this second turn

            if self.turnObstruct == True:                   #stall detected on right turn, obstacle must be a corner
                print("stall detected during second turn")
                #speech.speak("Stall. detected.")
                self.oneEighty(motors)                         #reverse alignment
            else:                                           #figure out if wall or obstacle
                self.turnObstruct = False
                motors.forwardFunction(distance = 90)       #move forward to clear obstacle
                self.detectTurnStall(motors, motors.LEFT)
                if self.turnObstruct == True:               # case that we bumped into a wall, close to a corner
                    #speech.speak("Stall. detected.")
                    motors.turnAngleFunction(angle = -115, wheel = motors.RIGHT)
                    motors.waitOnAction()
                else:
                    self.turnObstruct = False
                    motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)  #case that obstacle is an obstacle away from wall
                    motors.waitOnAction()                                       #turn left to to clear it
                    motors.forwardFunction(distance = 60)                       #move forward to get back towards original course
                    self.detectTurnStall(motors, motors.LEFT)

                    if self.turnObstruct == True:                               #obstacle larger than expected
                        #speech.speak("Stall. detected.")
                        motors.turnAngleFunction(angle = -38, wheel = motors.LEFT)      #turn right again, and continue to try to clear obstacle
                        motors.waitOnAction()
                        motors.forwardFunction(distance = 60)
                        motors.waitOnAction()
                        motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)     #turn back to get back on coarse
                        motors.waitOnAction()
                        motors.forwardFunction(distance = 60)
                        motors.waitOnAction()
                        motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)
                        motors.waitOnAction()
                    else:
                        self.turnObstruct = False
                        motors.turnAngleFunction(angle = -38, wheel = motors.LEFT)
                        motors.waitOnAction()

            self.turnObstruct = True

        #if stall did not occur on first left turn , don't forget to set self.turnObstruct to TRUE for top if statement
        elif self.turnObstruct == False:                    #if stall did not occur during first turn
            #speech.speak("Avoiding. Obstacle")
            motors.forwardFunction(distance = 90)           #move forward 2 ft to try and clear obstacle
            self.detectTurnStall(motors, motors.LEFT)
            if self.turnObstruct == True:                   #stall detected
                motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)  #avoid corner
                motors.waitOnAction()
            else:
                self.turnObstruct = False
                motors.turnAngleFunction(angle = -60, wheel = motors.RIGHT)  #turn to parallel course with obstacle
                self.detectTurnStall(motors, motors.LEFT)
                if self.turnObstruct == True:               # corner detected
                    self.oneEighty(motors)
                else:
                    self.turnObstruct = False
                    motors.forwardFunction(distance = 60)           #move forward 2 ft to try and clear obstacle
                    self.detectTurnStall(motors, motors.LEFT)
                    if self.turnObstruct == True:                       #wall detected
                        motors.turnAngleFunction(angle = 60, wheel = motors.LEFT)
                        motors.waitOnAction()
                    else:                               #regular obstacle, turn to avoid
                        self.turnObstruct = False
                        motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)
                        motors.waitOnAction()
                        motors.forwardFunction(distance = 85)   #go back towards original course
                        self.detectTurnStall(motors, motors.LEFT)

                        if self.turnObstruct == True:       #obstacle larger than expected
                            #speech.speak("Stall. detected.")
                            motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                            motors.waitOnAction()       #turn back to left, and go further before turning back in
                            motors.forwardFunction(distance = 60)
                            motors.waitOnAction()
                            motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)
                            motors.waitOnAction()
                            motors.forwardFunction(distance = 60)
                            motors.waitOnAction()
                            motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                            motors.waitOnAction()
                        else:
                            motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)  #turn to restore original alignment
                            motors.waitOnAction()

        self.aiC.collisionCBtriggered = False               #clear collisionCBtriggered flag before popping
        self.turnObstruct = False
        self.aiC.popState()                                 #pop avoidanceState off stack

        return
