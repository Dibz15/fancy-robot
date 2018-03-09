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

    def updateDisplay(self):
        cv2.imshow("Frame", self.rCV.getCurrentVisionFunctionValue("modified"))
    	#Show our binary color-detection image
    	cv2.imshow("Line Image", self.rCV.getCurrentVisionFunctionValue("lineImage"))
    	key = cv2.waitKey(1) & 0xFF

    	# if the `q` key was pressed, break from the loop
    	if key == ord("q"):
    		self.aiC.popState()

    def updateServo(self, direction, angle, ptAngle):
        self.ptC.pointDown()
        if self.moveServo:
            ptAngleDelta = (direction * 0.02)
            directionDelta = (direction - self.prevDirection) * 0.04

            ptAngle = max(70, min(110, ptAngle + ptAngleDelta + directionDelta))
            self.ptC.setServo(self.ptC.pan, ptAngle)
            time.sleep(0.1) #Make sure our servo has a bit of time to get in position

            self.prevDirection = direction

        if self.showDisplay:
            self.updateDisplay()

    def detectTurnStall(self, motors, pivotWheel):      #function to determine if there was turn obstruction
        self.turnObstruct = False
        self.motors.setStallState([False, False])            #reset the stall state of both wheels to false

        while not self.motors.currentOperation.complete:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            stallState = self.motors.getStallState()         #polls the stall state using motor controller for both wheels

            if (stallState[0] == True and pivotWheel == motors.RIGHT) or (stallState[1] == True and pivotWheel == motors.LEFT):
                if not self.motors.currentOperation.complete:    #checks if a non-pivot wheel stalled during turn
                    self.turnObstruct = True                #sets variable to true, signifying stall has occured
                    self.motors.halt()                           #cancels turn function by halting motors
                    break

    def linePresent(self):
        return (self.rCV.getCurrentVisionFunctionValue("missingContours") <= 3)

    def linePresentLow(self):
        return (self.rCV.getCurrentVisionFunctionValue("missingContours") < 5)

    def detectLinePresent(self):
        print("inside of detectLinePresent")
        time.sleep(0.2)

        while not self.motors.currentOperation.complete:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            if self.linePresentLow():
                return True

        return False

    def lineSweep(self):
        if not self.linePresentLow():
            #print("Looking left")
            self.ptC.setServo(self.ptC.getPanPin(), 90)
            for i in range(19):
                time.sleep(0.1)
                self.ptC.incServo(self.ptC.getPanPin(), 2)
                if self.linePresentLow():
                    self.motors.turnAngleFunction(angle = 15)
                    self.motors.waitOnAction()
                    return True

        if self.aiC.collisionCBtriggered:
            return False

        if not self.linePresentLow():
            #print("Looking right")
            self.ptC.setServo(self.ptC.getPanPin(), 90)
            for i in range(19):
                time.sleep(0.1)
                self.ptC.incServo(self.ptC.getPanPin(), -2)
                if self.linePresentLow():
                    self.motors.turnAngleFunction(angle = -15)
                    self.motors.waitOnAction()
                    return True

        return False

    def searchForLine(self, motors):                       #drive around in expanding concentric circles
            #drive in circles
        lineFound = False
        n = 1                                              #number of hexagon iterations
        hexRate = 3                                        #rate of increase for perimeter of hexagon (hexrate*6 = increase in perimeter)

        while not lineFound:
            time.sleep(0.1)

            if self.linePresentLow():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward first leg of hexagon
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward second leg of hexagon
            motors.waitOnAction()
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward third leg of hexagon
            motors.waitOnAction()
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward fourth leg of hexagon
            motors.waitOnAction()
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward fifth leg of hexagon
            motors.waitOnAction()
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -51, wheel = motors.LEFT) #turn command for a 60 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            motors.forwardFunction(distance = (15+ (n * hexRate)))       #move forward sixth leg of hexagon
            motors.waitOnAction()
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = 51, wheel = motors.RIGHT) #turn command for a 60 degree left turn, pivoting on left wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            motors.forwardFunction(distance = hexRate)       #travel to top left vertex of 2 * perimeter hexagon
            motors.waitOnAction()
            if self.detectLinePresent():                        #calls function to determine if line detected during forward movement
                lineFound = True
                break
            motors.turnAngleFunction(angle = -102, wheel = motors.LEFT) #turn command for a 120 degree right turn, pivoting on right wheel
            motors.waitOnAction()
            if self.lineSweep():
                lineFound = True
                break
            if self.aiC.collisionCBtriggered:
                return
            n  += 1

    def update(self):

        if self.showDisplay:
            self.updateDisplay()

        defaultSpeed = 5

        self.ptC.pointDown()

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

        #If line is not present
        else:
            self.motors.halt()
            time.sleep( 1.0 / 10.0 )

            if not self.hasSearched:

                if self.aiC.collisionCBtriggered:
                    return

                if not self.linePresent():
                    #print("Looking left")
                    self.ptC.setServo(self.ptC.getPanPin(), 90)
                    for i in range(19):
                        time.sleep(0.1)
                        self.ptC.incServo(self.ptC.getPanPin(), 2)
                        if self.linePresent():
                            self.motors.turnAngleFunction(angle = 15)
                            self.motors.waitOnAction()
                            break

                if self.aiC.collisionCBtriggered:
                    return

                if not self.linePresent():
                    #print("Looking right")
                    self.ptC.setServo(self.ptC.getPanPin(), 90)
                    for i in range(19):
                        time.sleep(0.1)
                        self.ptC.incServo(self.ptC.getPanPin(), -2)
                        if self.linePresent():
                            self.motors.turnAngleFunction(angle = -15)
                            self.motors.waitOnAction()
                            break

                if self.aiC.collisionCBtriggered:
                    return

                #Start looking here
                self.searchForLine(self.motors)

            self.ptC.setServo(self.ptC.getPanPin(), 90)

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
        self.irBeamLocked = False                       #flag set when ir value of '90' reliably polled
        self.cvInRange = False                          #flag set when computer vision becomes available as navigation tool
        self.alignedWithCenter = False                  #flag set when robot is ready to begin docking procedure
        self.dockFlag = False
        self.correctionAngle = 0
        self.angleStack = collections.deque(5*[0], 5)

    def cvScan(self):
        print(" first iteration of CVScan")
        #pans if needed to attempt to recognize secondary recognition color adhered
        #to center of base station. Sets 'cvInRange' to 'True' if recognizable
        rCV = self.aiC.getController(self.aiC.rCVString)
        ptC = self.aiC.getController(self.aiC.panTiltString)

        rCV.setCurrentVisionFunction("BaseFinder")
        while rCV.getCurrentVisionFunctionValue("binarized") is None:
            time.sleep(1.0 / 10.0)
            #print("Waiting for binarized image")
            continue

        if self.aiC.collisionCBtriggered:
            return 0.0

        if not rCV.getCurrentVisionFunctionValue("lineAbsent"):
            self.cvInRange = True
            return ptC.getCurrentAngle(ptC.getPanPin())

        #If we don't see a line, scan left
        if rCV.getCurrentVisionFunctionValue("lineAbsent"):
            #print("Looking left")
            ptC.setServo(ptC.getPanPin(), 90)
            for i in range(19):
                time.sleep(0.1)
                ptC.incServo(ptC.getPanPin(), 2)
                if not rCV.getCurrentVisionFunctionValue("lineAbsent"):
                    self.cvInRange = True
                    return ptC.getCurrentAngle(ptC.getPanPin())

        if self.aiC.collisionCBtriggered:
            return 0.0

        #If we still don't see a line, scan right
        if rCV.getCurrentVisionFunctionValue("lineAbsent"):
            #print("Looking right")
            ptC.setServo(ptC.getPanPin(), 90)
            for i in range(19):
                time.sleep(0.1)
                ptC.incServo(ptC.getPanPin(), -2)
                if not rCV.getCurrentVisionFunctionValue("lineAbsent"):
                    self.cvInRange = True
                    return ptC.getCurrentAngle(ptC.getPanPin())

        if self.aiC.collisionCBtriggered:
            return 0.0


    def alignCenter(self):                         #uses CV to align robot with secondary recognition color in center of
        print("Align center")
        self.aiC.collisionCBenable = False         #disable collsionCB
        rCV = self.aiC.getController(self.aiC.rCVString)
        ptC = self.aiC.getController(self.aiC.panTiltString)
        motors = self.aiC.getController(self.aiC.motorsString)

        rCV.setCurrentVisionFunction("BaseFinder")
        while rCV.getCurrentVisionFunctionValue("binarized") is None:
            time.sleep(1.0 / 10.0)
            #print("Waiting for binarized image")
            continue
        print("setting angle and calling cvScan")
        angle = self.cvScan()
        print("Initial angle: " + str(angle))

        ptC.setServo(ptC.getPanPin(), 90)
        motors.turnAngleFunction(angle = (angle - 91.0))
        motors.waitOnAction()

        directionThresh = 45
        distanceThresh = 40
        areaThresh = 2400

        if self.aiC.collisionCBtriggered:
            return

        lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
        direction = rCV.getCurrentVisionFunctionValue("direction")
        distance = rCV.getCurrentVisionFunctionValue("distance")

        print("LA: " + str(lineAbsent) + "Dir: " + str(direction) + "Dist: " + str(distance))

        rect_area = rCV.getCurrentVisionFunctionValue("averages")[1]
        while (not lineAbsent) and abs(direction) > directionThresh and (distance > distanceThresh or rect_area < areaThresh):
            time.sleep(0.3)
            lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
            averageQueue = rCV.getCurrentVisionFunctionValue("averageQueue")
            if len(averageQueue) >= 5:
                averages = rCV.getCurrentVisionFunctionValue("averages")
                direction = averages[0]
                distance = averages[2]
            else:
                direction = rCV.getCurrentVisionFunctionValue("direction")
                distance = rCV.getCurrentVisionFunctionValue("distance")

            if self.aiC.collisionCBtriggered:
                return

            angleDelta = direction * 0.2
            if ( direction < 1 ):
                #print("Line is on the right")
                angleDelta = min(-5, angleDelta)
            else:
                #print("Line is on the left")
                angleDelta = max(5, angleDelta)

            #Stop the motors
            motors.halt()

            #Figure out which wheel to turn with
            wheelToUse = motors.LEFT if (angleDelta < 0) else motors.RIGHT

            #Make our turn
            motors.turnAngleFunction(angle = ( angleDelta ), wheel = wheelToUse)
            motors.waitOnAction()

        if self.aiC.collisionCBtriggered:
            return

        lineAbsent = rCV.getCurrentVisionFunctionValue("lineAbsent")
        averageQueue = rCV.getCurrentVisionFunctionValue("averageQueue")
        if len(averageQueue) >= 5:
            averages = rCV.getCurrentVisionFunctionValue("averages")
            direction = averages[0]
            distance = averages[2]
        else:
            direction = rCV.getCurrentVisionFunctionValue("direction")
            distance = rCV.getCurrentVisionFunctionValue("distance")

        rect_area = rCV.getCurrentVisionFunctionValue("averages")[1]

        if abs(direction) <= directionThresh and (distance > distanceThresh or rect_area <= areaThresh):
            motors.forwardFunction(speed = 5, distance = 5)
            motors.waitOnAction()

        if self.aiC.collisionCBtriggered:
            return

        if (distance <= distanceThresh or rect_area > areaThresh) and abs(direction) <= directionThresh:
            motors.halt()
            self.alignedWithCenter = True

    def detectCharge(self, sensors):
        print("inside of detectCharge")
        time.sleep(0.1)

        while not self.motors.currentOperation.complete:     #loops while turnAngleFunction command has not completed
            time.sleep(0.1)
            if sensors.isCharging():
                return True

        if sensors.isCharging():
            return True

        return False

    def dock(self, motors, sensors):                                 #docking procedure

        print("in dock")
        self.dockFlag = True
        chargeDetected = False
        time.sleep(2)
        motors.forwardFunction(distance = 10)
        motors.waitOnAction()

        chargeDetected = self.detectCharge(sensors)
        toggleBit = 1

        while not chargeDetected:
            wheelToUse = motors.LEFT if (toggleBit == 1) else motors.RIGHT
            angle = -38 if (toggleBit == 1) else 38
            motors.turnAngleFunction(angle = angle, wheel = wheelToUse)
            if self.detectCharge(sensors):
                chargeDetected = True
                break

            toggleBit ^= 1
            #wiggle up
        motors.halt()
        self.aiC.popState()                                 #pop avoidanceState off stack
        self.pushState(chargeState(self.aiC))


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
                self.angleStack[j] == 0                      #replace invalid value with zero for averaging purposes
            elif self.angleStack[j] == 300:                  #check for 3+ IR sensors returning values ('300')
                count300 += 1
                self.angleStack[j] == 0
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

    def update(self):
        print("in findHomeState")
        print("scanning for IR signal")
        sensors = self.getController(self.aiC.sensorString)  #for easier access to sensorController's functions
        motors = self.getController(self.aiC.motorsString)   #set variable for easier access to MotorController
        speech = self.getController(self.aiC.speechString)
        invalidCount = 0                                     #used to determine how many successive times invalid ir angle polled

        self.irBeamLocked = False
        while not self.irBeamLocked and not self.dockFlag:                         #corrects orientation until lined up with transmitter
            time.sleep( 0.1 )

            self.getAvgIRAngle(sensors)                      #gets average or majority value of polled IR angles

            print("IR average value: " + str(self.irAngle))

            if self.irAngle is None:
                print("ir Angle equals none")

            elif self.irAngle == 90:                           #robot is lined up with base station
                self.irBeamLocked = True

            elif self.irAngle < 90:                                  #transmitter is offcenter to the right
                self.correctionAngle = ((self.irAngle * 1.15) - 90)        #calculate angle needed to line up with ir transmitter
                print("Angle < 90")
                print ("Correction angle is: " + str(self.correctionAngle))                                                     #extra 20 degrees added to compensate for turnAngleFunction
                                                                    #may need recalibration
                motors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.waitOnAction()                                    #wait for turn

            elif self.irAngle > 90 and self.irAngle != 200 and self.irAngle != 300:                                  #transmitter is off center to the left
                self.correctionAngle = ((self.irAngle*0.85) - 90)        #calculate angle needed to line up with ir transmittermotors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                print("angle > 90")
                print ("Correction angle is: " + str(self.correctionAngle))
                motors.turnAngleFunction(angle = self.correctionAngle)   #turn to line up with transmitter
                motors.waitOnAction()                                    #wait for turn

            elif self.irAngle == 300:                               #it is possible that robot is in very close proximity to transmitter
                print("ir angle = 300")
                check300 = 2                                        #check '300' value three times, to better verify that we are close to transmitter
                while check300 != 0:
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
                if invalidCount < 3:
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
            self.cvInRange = False
            while self.irAngle == 90 and self.cvInRange == False:   #keeps moving forward as long as aligned with transmitter,
                motors.forwardFunction(distance = 60)               # and CV is not in range to assist yet
                motors.waitOnAction()
                print("moving forward")
                motors.halt()
                time.sleep(0.2)
                self.getAvgIRAngle(sensors)                         #gets average or majority value of polled IR angles
                self.cvScan()                                      #cvScan function
                print("executing cvScan")

        if self.cvInRange == True and not self.dockFlag:                                  #within close enough proximity to transmitter to use CV to align
            while not self.alignedWithCenter:
                self.alignCenter()                                      #call alignCenter function, to align with center of base station

        if self.alignedWithCenter == True and not self.dockFlag:                          #if aligned with center of base station, begin docking routine
            self.dock(motors, sensors)


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

class avoidanceState(aiState):

    def __init__(self, aiC):                            #constructor for avoidanceState
        aiState.__init__(self, aiC)
        self.turnObstruct = False                       #default value for variable representing if a turn attempt was obstructed

    def detectTurnStall(self, motors, pivotWheel):      #function to determine if there was turn obstruction
        print("inside of detectTurnStall")
        self.turnObstruct = False
        motors.setStallState([False, False])            #reset the stall state of both wheels to false
        time.sleep(0.25)

        while not motors.currentOperation.complete:     #loops while turnAngleFunction command has not completed
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

    def oneEighty(self, motors):                            #reverse and turn around 180 degrees
        motors.reverseFunction(distance = 30)               #possible corner, reverse
        motors.waitOnAction()                               #wait untill reverse function has completed
        motors.turnAngleFunction(angle = -153)              #reorient in opposite direction
        motors.waitOnAction()

    def update(self):                                       #sets routine for obstacle avoidance
        print ("in avoidanceState's update")
        self.aiC.collisionCBtriggered = True
        motors = self.getController(self.aiC.motorsString)  #set variable for easier access to MotorController
        print("turning left 70 degrees, pivot on right wheel")
        motors.turnAngleFunction(angle = 60, wheel = motors.LEFT) #turn command for a 70 degree left turn, pivoting on right wheel

        self.detectTurnStall(motors, motors.RIGHT)          #calls function to determine if stall occurs during turn


        if self.turnObstruct == True:                       #if stall has occurred during turn
            print("stall detected during first turn")
            time.sleep(0.1)
            self.turnObstruct = False                       #reset variable to default
            motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)  #turn 120 degrees in the other direction, pivot on left
            self.detectTurnStall(motors, motors.LEFT)       #detect if stall occurs during this second turn

            if self.turnObstruct == True:
                print("stall detected during second turn")
                self.oneEighty(motors)
            else:
                self.turnObstruct = False
                motors.forwardFunction(distance = 90)
                self.detectTurnStall(motors, motors.LEFT)
                if self.turnObstruct == True:
                    motors.turnAngleFunction(angle = -115, wheel = motors.RIGHT)
                    motors.waitOnAction()
                else:
                    self.turnObstruct = False
                    motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                    motors.waitOnAction()
                    motors.forwardFunction(distance = 60)
                    self.detectTurnStall(motors, motors.LEFT)

                    if self.turnObstruct == True:
                        motors.turnAngleFunction(angle = -38, wheel = motors.LEFT)
                        motors.waitOnAction()
                        motors.forwardFunction(distance = 60)
                        motors.waitOnAction()
                        motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)
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
            motors.forwardFunction(distance = 90)           #move forward 2 ft to try and clear obstacle
            self.detectTurnStall(motors, motors.LEFT)
            if self.turnObstruct == True:
                motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                motors.waitOnAction()
            else:
                self.turnObstruct = False
                motors.turnAngleFunction(angle = -60, wheel = motors.RIGHT)
                self.detectTurnStall(motors, motors.LEFT)
                if self.turnObstruct == True:
                    self.oneEighty(motors)
                else:
                    self.turnObstruct = False
                    motors.forwardFunction(distance = 60)           #move forward 2 ft to try and clear obstacle
                    self.detectTurnStall(motors, motors.LEFT)
                    if self.turnObstruct == True:
                        motors.turnAngleFunction(angle = 60, wheel = motors.LEFT)
                        motors.waitOnAction()
                    else:
                        self.turnObstruct = False
                        motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)
                        motors.waitOnAction()
                        motors.forwardFunction(distance = 60)
                        self.detectTurnStall(motors, motors.LEFT)

                        if self.turnObstruct == True:
                            motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                            motors.waitOnAction()
                            motors.forwardFunction(distance = 60)
                            motors.waitOnAction()
                            motors.turnAngleFunction(angle = -38, wheel = motors.RIGHT)
                            motors.waitOnAction()
                            motors.forwardFunction(distance = 60)
                            motors.waitOnAction()
                            motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                            motors.waitOnAction()
                        else:
                            motors.turnAngleFunction(angle = 38, wheel = motors.LEFT)
                            motors.waitOnAction()

        self.aiC.collisionCBtriggered = False               #clear collisionCBtriggered flag before popping
        self.turnObstruct = False
        self.aiC.popState()                                 #pop avoidanceState off stack

        return
