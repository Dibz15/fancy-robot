'''
*	File: MotorController.py
*	Description:  This module holds the MotorController class, which assists
*       in controlling our motors through PWM. A separate thread is devoted to
*       a PID loop which controls motor speed and angle through feedback.
*	Author(s):		Austin Dibble
*	Date Created:	12/28/17
'''
import pigpio
import time
from collections import deque
import Constants
from threading import Thread
import math
from PID import *

'''
*	Class: MotorController.py
*	Description:	MotorController is a threaded class which takes care of
*       our motor control as an abstraction interface. Internally, it uses PID
*       controls and callbacks to provide a simple interface for movement.
*	Author(s):		Austin Dibble
*	Date Created:	12/28/18
'''
class MotorController:

    def __init__(self, pi, leftForward = Constants.leftMotorForward, leftReverse = Constants.leftMotorReverse, rightForward = Constants.rightMotorForward, rightReverse = Constants.rightMotorReverse, freq = 50, decoder = None):
        self.pi = pi
        self.leftForward = int(leftForward)
        self.leftReverse = int(leftReverse)
        self.rightForward = int(rightForward)
        self.rightReverse = int(rightReverse)
        self.duty = {}
        self.duty[self.leftForward] = 0
        self.duty[self.leftReverse] = 0
        self.duty[self.rightForward] = 0
        self.duty[self.rightReverse] = 0
        self.freq = 50

        self.currentOperation = None

        self.decoder = decoder                          #Decoder for callback on decoder ticks

        #Set decoder callback, if a decoder is provided
        if self.decoder is not None:
            self.decoder.setCallbackFunction(self.decoderCallback)

        self.stopped = True
        self.manualControl = False
        self.halted = False


        #============Motor Controller Function Labels==================
        self.FORWARD       = 0
        self.REVERSE       = 1
        self.TURN_ANGLE    = 2
        self.RIGHT_TURN    = 3
        self.LEFT_TURN     = 4
        self.HALT          = 5

        #Wheel contants?
        self.LEFT = 0
        self.RIGHT = 1

        #Note, since the right wheel is bigger, it will have a greater linear speed
        #at the same rotational speed.
        self.RIGHT_DIAM_MM = 67.0 #mm
        self.LEFT_DIAM_MM = 66.0 #mm

        self.RIGHT_RADIUS_MM = self.RIGHT_DIAM_MM / 2.0
        self.LEFT_RADIUS_MM = self.LEFT_DIAM_MM / 2.0

        self.RIGHT_CIRC_CM = self._mmToCm(float(self.RIGHT_DIAM_MM)) * math.pi  #21.0486708 cm
        self.LEFT_CIRC_CM = self._mmToCm(float(self.LEFT_DIAM_MM)) * math.pi   #20.7345115 cm

        self.TICKS_PER_REV = 20.0   #Encoder ticks per revolution

        self.SINGLE_TICK_ANGLE = 360.0 / self.TICKS_PER_REV #18 degrees turn per tick
        self.SINGLE_TICK_ANGLE_RAD = math.radians(self.SINGLE_TICK_ANGLE)   #Angle in radians, for physics calcs

        #Width of the chassis in cm.
        self.CHASSIS_WIDTH_CM = 14.9 #diameter of wheel->wheel distance

    '''
    ****************************************************
    *Function: start
    *Description: Starts everything up for motors, including the PID thread
    *Parameters:
    *   self -
    *Returns:
    *   nothing
    ****************************************************
    '''
    def start(self):
        print("Motors started")
        self.pi.set_mode(self.leftForward, pigpio.OUTPUT)           #Set leftForward motor pin to output
        self.pi.set_mode(self.leftReverse, pigpio.OUTPUT)           #Set leftReverse motor pin to output
        self.pi.set_mode(self.rightForward, pigpio.OUTPUT)          #Set rightForward motor pin to output
        self.pi.set_mode(self.rightReverse, pigpio.OUTPUT)          #Set rightReverse motor pin to output
        self.pi.set_PWM_frequency(self.leftForward, self.freq)      #Set PWM frequencies (default 50Hz)
        self.pi.set_PWM_frequency(self.leftReverse, self.freq)
        self.pi.set_PWM_frequency(self.rightForward, self.freq)
        self.pi.set_PWM_frequency(self.rightReverse, self.freq)

        self.stopped = False
        self.manualControl = False

        #Start thread for updating current operation
        Thread(target = self.update, args = ()).start()

    #Stop everything!
    def stop(self):
        print("Stopping motors")
        self.stopped = True
        self.halted = True
        time.sleep(1)
        self._set_motor_duty(self.leftForward, 0)       #Turn off all motors
        self._set_motor_duty(self.leftReverse, 0)       #Turn off all motors
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, 0)

    def update(self):
        while not self.stopped:
            time.sleep(1.0 / 15.0)

            #print(str(self.manualControl))

            if not self.manualControl and self.currentOperation is not None and not self.halted:
                self.currentOperation.operate()


    def getCurrentFunction(self):
        return self.currentOperation

    def decoderCallback(self, gpioPin):

        #Do something?
        #print("Callback")
        pass

    def _motorPower(self, motorPin, pwr):
        duty = pwr * 2.55
        self._set_motor_duty(motorPin, duty)

    def _set_motor_duty(self, motorPin, duty):
        duty = max(0, min(255, duty))
        self.duty[motorPin] = duty
    	self.pi.set_PWM_dutycycle(motorPin, duty)

    def forward(self, Lpwr, Rpwr):
        self.halted = False
        #print("Motors forward")
        #Lduty = (Lpwr / 100.0) * 255
        Lduty = Lpwr * 2.55
        #Rduty = (Rpwr / 100.0) * 255
        Rduty = Rpwr * 2.55
        self._set_motor_duty(self.leftForward, Lduty)
        self._set_motor_duty(self.leftReverse, 0)
        self._set_motor_duty(self.rightForward, Rduty)
        self._set_motor_duty(self.rightReverse, 0)


    '''
    ****************************************************
    *Function: reverse
    *Description:
    *   Puts motors into reverse
    *Parameters:
    *   self -
    *   pwr - percent power, as a decimal. Ex: 50% would be 50
    *Returns:
    *   nothing
    ****************************************************
    '''
    def reverse(self, Lpwr, Rpwr):
        self.halted = False
        #Lduty = (Lpwr / 100.0) * 255
        Lduty = (Lpwr * 2.55)
        #Rduty = (Rpwr / 100.0) * 255
        Rduty = (Rpwr * 2.55)
        self._set_motor_duty(self.leftForward, 0)
        self._set_motor_duty(self.leftReverse, Lduty)
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, Rduty)

    '''
    ****************************************************
    *Function: rightTurn
    *Description: Right turn with some amount of "power"
    * Power here is simply the percentage of the duty cycle.
    *Parameters:
    *   self -
    *   pwr - percent power, as a decimal. Ex: 50% would be 50
    *Returns:
    *   nothing
    ****************************************************
    '''
    def rightTurn(self, Lpwr, Rpwr):
        self.halted = False
        #Lduty = (Lpwr / 100.0) * 255
        Lduty = (Lpwr * 2.55)
        #Rduty = (Rpwr / 100.0) * 255
        Rduty = (Rpwr * 2.55)
        self._set_motor_duty(self.leftForward, Lduty)
        self._set_motor_duty(self.leftReverse, 0)
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, Rduty)

    '''
    ****************************************************
    *Function: leftTurn
    *Description: Left turn with some amount of "power"
    * Power here is simply the percentage of the duty cycle.
    *Parameters:
    *   self -
    *   pwr - percent power, as a decimal. Ex: 50% would be 50
    *Returns:
    *   nothing
    ****************************************************
    '''
    def leftTurn(self, Lpwr, Rpwr):
        self.halted = False
        Lduty = (Lpwr * 2.55)
        Rduty = (Rpwr * 2.55)
        self._set_motor_duty(self.leftForward, 0)
        self._set_motor_duty(self.leftReverse, Lduty)
        self._set_motor_duty(self.rightForward, Rduty)
        self._set_motor_duty(self.rightReverse, 0)


    def takeControl(self):
        print("Obtaining manual control of motors...")
        self.manualControl = True
        return

    def releaseControl(self):
        print("Releasing control of motors...")
        self.manualControl = False
        return

    #Halt the motors (same as stop, but stop we use at the end of main)
    def halt(self):
        self.halted = True
        #print("break!")
        self._set_motor_duty(self.leftForward, 0)       #Turn off all motors
        self._set_motor_duty(self.leftReverse, 0)       #Turn off all motors
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, 0)
        time.sleep(0.2)

    def bumpbot(self):
        self.halt()
        self.reverse(80, 80)
        time.sleep(2)
        self.leftTurn(80, 80)
        time.sleep(2)
        self.halted = False

    def forwardFunction(self, distance = -1, speed = -1):
        self.halted = False
        self.stallState = [False, False]
        self.currentOperation = LinearFunction(self.decoder, self, distance = distance, targetSpeed = speed, direction = 0)

    def reverseFunction(self, distance = -1, speed = -1):
        self.halted = False
        self.stallState = [False, False]
        self.currentOperation = LinearFunction(self.decoder, self, distance = distance, targetSpeed = speed, direction = 1)

    def turnAngleFunction(self, angle, wheel = None, targetSpeed = -1):
        self.halted = False
        self.stallState = [False, False]
        self.currentOperation = TurnAngleFunction(self.decoder, self, angle, wheel, targetSpeed)

    def waitOnAction(self):
        while not self.currentOperation.complete:
            time.sleep(0.2)

        return

    def getStallState(self):
        return self.currentOperation.stallState

    def setStallState(self, state):
        self.currentOperation.stallState = state

    def getCurrentDutyCycle(self, motorPin):
        return self.duty[motorPin]

    def dutyToPower(self, duty):
        return (duty / 255.0) * 100.0


    '''
    Given the desired turn angle (in degrees) of the entire chassis, find the
    number of ticks required for each wheel to turn the chassis to that angle.

    Formula is as follows:
    Chassis turn arc length (S_c) = chassis_radius * angle
    Wheel radius = r_w
    Wheel turn arc length (S_w) = ~S_c
    Wheel turn angle (theta_w) = S_w / r_w
    Wheel ticks (ticks_w) = theta_w / SINGLE_TICK_ANGLE_RAD

    Returns a tuple for left and right wheel ticks

    Note: if oneWheel parameter is set as True, this is calculated for a one-wheeled turn.
    A tuple is still passed, but the info for only one wheel should be used.

    '''
    def _chassisAngleToTicks(self, angle, oneWheel = False):
        angle = float(angle)                                #make sure it's a float
        angle = math.radians(angle)                         #Convert angle to radians for physics

        if oneWheel:
            radius = self._cmToMm(self.CHASSIS_WIDTH_CM)         #Turn radius in mm
        else:
            radius = self._cmToMm(self.CHASSIS_WIDTH_CM) / 2.0

        chassisArcLength = self._arcLengthFromAngle(radius, angle)               #Chassis arc length in mm
        LwheelAngle = self._angleFromArc( chassisArcLength, self.LEFT_RADIUS_MM) #Left wheel angle delta, in radians
        RwheelAngle = self._angleFromArc( chassisArcLength, self.RIGHT_RADIUS_MM)

        LwheelTicks = self._wheelAngleToTicks(LwheelAngle)
        RwheelTicks = self._wheelAngleToTicks(RwheelAngle)

        return (LwheelTicks, RwheelTicks)


    '''
    Given the wheel angle (in radians), this returns the approximate required
    number of ticks (rounded) to achieve that angle
    '''
    def _wheelAngleToTicks(self, angle):
        return round(abs(angle) / self.SINGLE_TICK_ANGLE_RAD, 0)


    '''
    ****************
    Gives the theta (angle in rads) given by an arc length and the radius of the arc.
    ****************
    '''
    def _angleFromArc(self, arcLen, rad):
        return arcLen / rad

    '''*********************************************************************************
    Given a radius and some angle (in radians), this finds the corresponding arc lenght.
     |--
     |      \
    r|\         \  arc length
     |    \        \
     |theta  \       \
     |_________\______|

    ********************************************************************************
    '''
    def _arcLengthFromAngle(self, rad, theta):
        return rad * abs(theta)


    '''
    ************************************************************************************
    Takes a tick delta (say 0.01 s) and converts it to angular velocity (in rads/sec), using
    our known tick angle delta of 18 degrees
    ***********************************************************************************
    '''
    def _deltaToAngularVelocity(self, delta, numTicks = 1):
        if (delta == 0):
            return -1
        singleTickDelta = float(delta / numTicks)
        w = self.SINGLE_TICK_ANGLE_RAD / singleTickDelta
        return w

    '''
    Takes a tick delta (like 0.01s), and a radius (7 cm), and converts
    it to linear velocity in {radius_unit}/s
    '''
    def _deltaToLinearVelocity(self, delta, radius, numTicks = 1):
        w = self._deltaToAngularVelocity(delta, numTicks)
        v = w * radius
        return v

    '''
    Convert cm to mm
    '''
    def _cmToMm(self, cm):
        return cm * 10.0

    '''
    Convert mm to cm
    '''
    def _mmToCm(self, mm):
        return mm / 10.0

    '''
    Convert ms to S
    '''
    def _msToS(self, ms):
        return ms / 1000.0


class MotorFunction:
    def __init__(self, decoder, motors, targetSpeed = -1):
        self.decoder = decoder
        self.complete = False
        self.targetSpeed = targetSpeed
        self.motors = motors

        self.stallState = [False, False]

        self.decoder.clearDeltas()
        self.decoder.resetCounters()

    def isComplete(self):
        return self.complete

    def operate(self):
        pass


class LinearFunction(MotorFunction):
    def __init__(self, decoder, motors, distance = -1, targetSpeed = -1, direction = 0):
        MotorFunction.__init__(self, decoder, motors, targetSpeed)

        self.FORWARD = 0
        self.REVERSE = 1

        self.targetDistance = distance
        self.angleL = self.motors._angleFromArc( distance, self.motors._mmToCm(self.motors.LEFT_RADIUS_MM) )
        self.angleR = self.motors._angleFromArc( distance, self.motors._mmToCm(self.motors.RIGHT_RADIUS_MM) )
        self.desiredDistanceTicks = ( self.motors._wheelAngleToTicks(self.angleL), self.motors._wheelAngleToTicks(self.angleR) )

        self.direction = direction

        self.targetPwr = 80

        self.dPID = PID(initSetpoint = 0.0, Kp = 8.0, Ki = 1.0, Kd = 0.09)
        self.sPID = PID(initSetpoint = self.targetSpeed, Kp = 2.5, Ki = 0.3, Kd = 0.12)
        #TODO
        self.mDPID = MotorDistancePID(self.decoder, self.dPID, self.targetPwr)




    def operate(self):
        if self.complete:
            return

        self.mDPID.update()

        lTargetPwr = self.mDPID.getOutput()[0]
        rTargetPwr = self.mDPID.getOutput()[1]

        #Get the current number of ticks for each wheel
        self.Lticks = self.decoder.getCounter()[0]
        self.Rticks = self.decoder.getCounter()[1]

        #If neither distance nor speed were specified, just go at a steady rate.
        if self.targetDistance == -1 and self.targetSpeed == -1:
            self.runUpdate()

        #If we specified distance
        if self.targetDistance != -1:
            self.distanceUpdate()

        #If we specified speed.
        if self.targetSpeed != -1:
            self.speedUpdate()

        if (((time.time() * 1000) - self.decoder.getLastTickTime()[0])) > 200:
            self.stallState[0] = True
        else:
            self.stallState[0] = False

        if (((time.time() * 1000) - self.decoder.getLastTickTime()[1])) > 200:
            self.stallState[1] = True
        else:
            self.stallState[1] = False

        return

    def runUpdate(self):
            lTargetPwr = self.mDPID.getOutput()[0]
            rTargetPwr = self.mDPID.getOutput()[1]

            if self.direction == self.FORWARD:
                self.motors.forward(lTargetPwr, rTargetPwr)
            elif self.direction == self.REVERSE:
                self.motors.reverse(lTargetPwr, rTargetPwr)


    def distanceUpdate(self):
        lTargetPwr = self.mDPID.getOutput()[0]
        rTargetPwr = self.mDPID.getOutput()[1]

        #If we haven't yet reached our tick target
        if (self.Lticks < self.desiredDistanceTicks[0]) and (self.Rticks < self.desiredDistanceTicks[1]):
            #print("RCP:" + str(rCurrentPwr) + ",RTP:" + str(rTargetPwr) + ",LCP:" + str(lCurrentPwr) + ",LTP:" + str(lTargetPwr) + ",dP:" + str(deltaPwr))
            if self.direction == self.FORWARD:
                self.motors.forward(lTargetPwr, rTargetPwr)
            elif self.direction == self.REVERSE:
                self.motors.reverse(lTargetPwr, rTargetPwr)

            #If the right wheel is still going
        elif (self.Lticks >= self.desiredDistanceTicks[0]) and (self.Rticks < self.desiredDistanceTicks[1]):
            if self.direction == self.FORWARD:
                self.motors.forward(0, rTargetPwr)
            elif self.direction == self.REVERSE:
                self.motors.reverse(0, rTargetPwr)

            #If the left wheel is still going
        elif (self.Lticks < self.desiredDistanceTicks[0]) and (self.Rticks >= self.desiredDistanceTicks[1]):

            if self.direction == self.FORWARD:
                self.motors.forward(lTargetPwr, 0)
            elif self.direction == self.REVERSE:
                self.motors.reverse(lTargetPwr, 0)

        #If both motors have reached their ticks
        elif (self.Lticks >= self.desiredDistanceTicks[0]) and (self.Rticks >= self.desiredDistanceTicks[1]):
                #Let everybody know we're done
                self.motors.halt()
                self.complete = True

    def speedUpdate(self):
        lTargetPwr = self.mDPID.getOutput()[0]
        rTargetPwr = self.mDPID.getOutput()[1]

        averages = self.decoder.getDeltaAverages()
        #print("Avg: " + str(averages))


        lV = self.motors._deltaToLinearVelocity(self.motors._msToS(averages[0]), self.motors._mmToCm(self.motors.LEFT_RADIUS_MM))
        rV = self.motors._deltaToLinearVelocity(self.motors._msToS(averages[0]), self.motors._mmToCm(self.motors.RIGHT_RADIUS_MM))

        #print("Velocities: (" + str(lV) + ", " + str(rV) + ")")

        if (((time.time() * 1000) - self.decoder.getLastTickTime()[0])) > 200:
            #print("Left stalled")
            lV = 0.0

        if (((time.time() * 1000) - self.decoder.getLastTickTime()[1])) > 200:
            #print("Right stalled")
            rV = 0.0

        lV = max(lV, 0.0)
        rV = max(rV, 0.0)
        #The wheels should be going a similar speed, due to our other PID loop, so it's safe to
        #use the average as the input into our speed PID.
        aV = (lV + rV) * 0.5
        #print("AV: " + str(aV))
        #print("Target speed: " + str(self.targetSpeed))

        #Update our speed PID loop, given the real speed.
        #It will compare its setpoint (targetSpeed) to the given speed, and calculate the new output power delta for us
        #print("SPID update")
        self.sPID.update(aV)

        #Get our calculated wheel power delta
        newPwrDelta = self.sPID.getOutput()
        #print("Pwr delta: " + str(newPwrDelta))

        #print("Old pwr: " + str(self.targetPwr))
        #Clamp our targetPower to 99%, since 100% is absolute max.
        newTargetPwr = min(self.targetPwr + newPwrDelta, 99.0)
        #print("New pwr: " + str(newTargetPwr))
        #Make sure our distance PID knows of the new target power.
        self.mDPID.setTargetPwr(newTargetPwr)

        #Update our motor power
        if self.direction == self.FORWARD:
            self.motors.forward(lTargetPwr, rTargetPwr)
        elif self.direction == self.REVERSE:
            self.motors.reverse(lTargetPwr, rTargetPwr)

class TurnAngleFunction(MotorFunction):

    def __init__(self, decoder, motors, angle, wheel = None, targetSpeed = -1):
        MotorFunction.__init__(self, decoder, motors, targetSpeed)
        self.oneWheel = False
        self.wheel = wheel

        if self.wheel is not None:
            self.oneWheel = True

        self.desiredAngleTicks = self.motors._chassisAngleToTicks(angle, self.oneWheel)
        self.angle = angle
        self.targetSpeed = targetSpeed

        self.targetPwr = 60

        self.dPID = PID(initSetpoint = 0.0, Kp = 10.0, Ki = 1.0, Kd = 0.05)
        self.mDPID = MotorDistancePID(self.decoder, self.dPID, self.targetPwr)

    def operate(self):
            #if 0, is given, we don't need to do anything
            if (self.angle == 0) or self.complete:
                return

            if (((time.time() * 1000) - self.decoder.getLastTickTime()[0])) > 200:
                self.stallState[0] = True
            else:
                self.stallState[0] = False

            if (((time.time() * 1000) - self.decoder.getLastTickTime()[1])) > 200:
                self.stallState[1] = True
            else:
                self.stallState[1] = False

            #print("DesiredTicks: " + str(self.desiredAngleTicks) + ", ticks: " + str(self.decoder.getCounter()))

            #Whether or not we have a speed to keep
            if self.targetSpeed != -1:
                #TODO PID speed control
                pass
            else:

                self.mDPID.update()

                lTargetPwr = self.mDPID.getOutput()[0]
                rTargetPwr = self.mDPID.getOutput()[1]

                #If this is a 2-wheel turn
                if self.wheel is None:
                    #If both wheels haven't reach their mark
                    if (self.decoder.getCounter()[0] < self.desiredAngleTicks[0]) and (self.decoder.getCounter()[1] < self.desiredAngleTicks[1]):
                        if self.angle > 0:
                            self.motors.leftTurn(lTargetPwr, rTargetPwr)
                        elif self.angle < 0:
                            self.motors.rightTurn(lTargetPwr, rTargetPwr)

                    #If the right wheel is still going
                    elif (self.decoder.getCounter()[0] >= self.desiredAngleTicks[0]) and (self.decoder.getCounter()[1] < self.desiredAngleTicks[1]):
                        if self.angle > 0:
                            self.motors.leftTurn(0, rTargetPwr)
                        elif self.angle < 0:
                            self.motors.rightTurn(0, rTargetPwr)

                    #If the left wheel is still going
                    elif (self.decoder.getCounter()[0] < self.desiredAngleTicks[0]) and (self.decoder.getCounter()[1] >= self.desiredAngleTicks[1]):
                        if self.angle > 0:
                            self.motors.leftTurn(lTargetPwr, 0)
                        elif self.angle < 0:
                            self.motors.rightTurn(lTargetPwr, 0)

                    #If both motors have reached their ticks
                    elif (self.decoder.getCounter()[0] >= self.desiredAngleTicks[0]) and (self.decoder.getCounter()[1] >= self.desiredAngleTicks[1]):
                        #Let everybody know we're done
                        self.motors.halt()
                        self.complete = True

                #If this is a 1-wheel turn
                else:
                    self.wheel = int(self.wheel)

                    #If we're using the left wheel
                    if self.wheel == self.motors.LEFT:
                        #If we haven't yet reached our mark
                        if (self.decoder.getCounter()[0] < self.desiredAngleTicks[0]):
                            if self.angle > 0:
                                self.motors.leftTurn(self.targetPwr, 0)
                            elif self.angle < 0:
                                self.motors.rightTurn(self.targetPwr, 0)
                        #If we have reached our mark
                        elif (self.decoder.getCounter()[0] >= self.desiredAngleTicks[0]):
                            #We're done now
                            self.motors.halt()
                            self.complete = True

                    #If we're using the right wheel
                    elif self.wheel == self.motors.RIGHT:
                        #If we haven't reached our mark
                        if (self.decoder.getCounter()[1] < self.desiredAngleTicks[1]):
                            if self.angle > 0:
                                self.motors.leftTurn(0, self.targetPwr)
                            elif self.angle < 0:
                                self.motors.rightTurn(0, self.targetPwr)
                        #If we have reached it
                        elif (self.decoder.getCounter()[1] >= self.desiredAngleTicks[1]):
                            #Signal we're done
                            self.motors.halt()
                            self.complete = True
            return




class MotorDistancePID:
    def __init__(self, decoder, PID, targetPwr, maxPwr = 100):
        self.dPID = PID
        self.decoder = decoder
        self.targetPwr = targetPwr
        self.rTargetPwr = targetPwr
        self.lTargetPwr = targetPwr
        self.maxPwr = maxPwr

    def update(self):
        self.Lticks = self.decoder.getCounter()[0]
        self.Rticks = self.decoder.getCounter()[1]

        highestTick = max(self.Lticks, self.Rticks)

        #Lowest tick amount so far
        lowestTick = min(self.Lticks, self.Rticks)

        #Difference between our highest and lowest
        tickDelta = highestTick - lowestTick

        #Update our PID loop with the current error
        self.dPID.update(tickDelta)

        #Get the new power adjustment
        deltaPwr = abs(self.dPID.getOutput())

        #rCurrentPwr = self.motors.dutyToPower(self.motors.getCurrentDutyCycle(self.motors.rightForward))
        #lCurrentPwr = self.motors.dutyToPower(self.motors.getCurrentDutyCycle(self.motors.leftForward))

        if (self.Lticks < self.Rticks):
            if (self.lTargetPwr + deltaPwr >= self.maxPwr):
                self.rTargetPwr = self.targetPwr - deltaPwr
                self.lTargetPwr = self.lTargetPwr

            elif (self.lTargetPwr + deltaPwr < self.maxPwr):
                self.rTargetPwr = self.targetPwr
                self.lTargetPwr = self.targetPwr + deltaPwr

        elif (self.Lticks > self.Rticks):
            if (self.rTargetPwr + deltaPwr >= self.maxPwr):
                self.lTargetPwr = self.targetPwr - deltaPwr
                self.rTargetPwr = self.rTargetPwr

            elif (self.rTargetPwr + deltaPwr < self.maxPwr):
                self.lTargetPwr = self.lTargetPwr
                self.rTargetPwr = self.rTargetPwr + deltaPwr

        elif (self.Lticks == self.Rticks):
            self.lTargetPwr = self.lTargetPwr
            self.rTargetPwr = self.rTargetPwr

    def getOutput(self):
        return (self.lTargetPwr, self.rTargetPwr)

    def setTargetPwr(self, pwr):
        self.targetPwr = pwr

    def setMaxPwr(self, pwr):
        self.maxPwr = pwr
