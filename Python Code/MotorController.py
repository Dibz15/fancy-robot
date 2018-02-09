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
from threading import Thread
import math

'''
*	Class: MotorController.py
*	Description:	MotorController is a threaded class which takes care of
*       our motor control as an abstraction interface. Internally, it uses PID
*       controls and callbacks to provide a simple interface for movement.
*	Author(s):		Austin Dibble
*	Date Created:	12/28/18
'''
class MotorController:

    def __init__(self, pi, leftForward, leftReverse, rightForward, rightReverse, freq = 50, decoder = None):
        self.pi = pi
        self.leftForward = leftForward
        self.leftReverse = leftReverse
        self.rightForward = rightForward
        self.rightReverse = rightReverse
        self.duty = []
        self.freq = 50

        self.currentOperation = self.stop               #Initialize the controller with the motors stopped
        self.currentOperationArgs = []                  #Current parameters for our operation?

        self.decoder = decoder                          #Decoder for callback on decoder ticks

        #Set decoder callback, if a decoder is provided
        if self.decoder is not None:
            self.decoder.setCallbackFunction(self.decoderCallback)

        self.stopped = True

        #============Motor Controller Function Labels==================
        self.FORWARD       = 0
        self.REVERSE       = 1
        self.TURN_ANGLE    = 2
        self.RIGHT_TURN    = 3
        self.LEFT_TURN     = 4

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

        self.TICKS_PER_REV = 20.0

        self.SINGLE_TICK_ANGLE = 360.0 / self.TICKS_PER_REV #18 degrees turn per tick
        self.SINGLE_TICK_ANGLE_RAD = math.radians(self.SINGLE_TICK_ANGLE)

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

        #Start thread for updating current operation
        Thread(target = self.update, args = self.currentOperationArgs).start()

    #Stop everything!
    def stop(self):
        #print("Stopping motors")
        self._set_motor_duty(self.leftForward, 0)       #Turn off all motors
        self._set_motor_duty(self.leftReverse, 0)       #Turn off all motors
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, 0)

    def update(self, *args):
        self.currentOperation(*args)

    def decoderCallback(self, gpioPin):

        #Do something?
        #print("Callback")
        pass

    def leftStep(self, pwr):
        self._motorPower(self.rightForward, 0)
        self._motorPower(self.rightReverse, 0)
        self._motorPower(self.leftForward, pwr)
        self._motorPower(self.leftReverse, 0)

    def rightStep(self, pwr):
        self._motorPower(self.leftForward, 0)
        self._motorPower(self.leftReverse, 0)
        self._motorPower(self.rightForward, pwr)
        self._motorPower(self.rightReverse, 0)

    def _motorPower(self, motorPin, pwr):
        duty = pwr * 2.55
        self._set_motor_duty(motorPin, duty)

    def _set_motor_duty(self, motorPin, duty):
        duty = max(0, min(255, duty))
        self.duty[motorPin] = duty
    	self.pi.set_PWM_dutycycle(motorPin, duty)

    def forward(self, Lpwr, Rpwr):
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
    *Function: _forwardPower
    *Description:
    *   Forward motion, with different duty for each motor
    *Parameters:
    *   self -
    *   pwr - percent power, as a decimal. Ex: 50% would be 50
    *Returns:
    *   nothing
    ****************************************************
    '''
    def _forwardPower(self, leftPower, rightPower):
        self._motorPower(self.leftForward, leftPower)
        self._motorPower(self.leftReverse, 0)
        self._motorPower(self.rightForward, rightPower)
        self._motorPower(self.rightReverse, 0)

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
        Lduty = (Lpwr * 2.55)
        Rduty = (Rpwr * 2.55)
        self._set_motor_duty(self.leftForward, 0)
        self._set_motor_duty(self.leftReverse, Lduty)
        self._set_motor_duty(self.rightForward, Rduty)
        self._set_motor_duty(self.rightReverse, 0)

    #
    def getCurrentDutyCycle(self, motorPin):
        return self.duty[motorPin]

    '''
    ****************************************************
    *Function: motorFunction
    *Description: This operates on the currently set motor
    *   function pointer, with the given array of parameters.
    *Parameters:
    *   function - function constant, used to set the current
    *       function
    *   *parameters - variable parameter list of parameters, since
    *       these may vary depending on the function
    *Returns:
    *   nothing
    ****************************************************
    '''
    def motorFunction(function, *parameters):
        #if function ==
        pass

    #Halt the motors (same as stop, but stop we use at the end of main)
    def halt(self):
        self.stop()

    '''
    Given the wheel angle in radians, this returns the approximate required
    number of ticks (rounded) to achieve that angle
    '''
    def _wheelAngleToTicks(self, angle):
        #TODO
        return

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
        return rad * theta


    '''
    ************************************************************************************
    Takes a tick delta (say 0.01 s) and converts it to angular velocity (in rads/sec), using
    our known tick angle delta of 18 degrees
    ***********************************************************************************
    '''
    def _deltaToAngularVelocity(self, delta, numTicks = 1):
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
    Convert mm to cm
    '''
    def _mmToCm(self, mm):
        return mm / 10.0

    '''
    Convert ms to S
    '''
    def _msToS(self, ms):
        return ms / 1000.0
