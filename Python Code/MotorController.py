import pigpio
import time
from collections import deque
from threading import Thread
import math

class MotorController:

    def __init__(self, pi, leftForward, leftReverse, rightForward, rightReverse, freq = 50, decoder = None):
        self.pi = pi
        self.leftForward = leftForward
        self.leftReverse = leftReverse
        self.rightForward = rightForward
        self.rightReverse = rightReverse
        self.duty = {}
        self.freq = 50

        self.currentOperation = self.stop
        self.currentOperationArgs = []

        self.decoder = decoder                          #Decoder for callback on decoder ticks

        if self.decoder is not None:
            self.decoder.setCallbackFunction(self.decoderCallback)

        self.stopped = True

        #============Motor Controller Function Labels==================
        self.FORWARD_SPEED = 0
        self.FORWARD_STEP  = 1
        self.REVERSE_SPEED = 2
        self.REVERSE_STEP  = 3
        self.TURN_ANGLE    = 4
        self.RIGHT_TURN    = 5
        self.LEFT_TURN     = 6

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


    def start(self):
        print("Motors started")
        self.pi.set_mode(self.leftForward, pigpio.OUTPUT)
        self.pi.set_mode(self.leftReverse, pigpio.OUTPUT)
        self.pi.set_mode(self.rightForward, pigpio.OUTPUT)
        self.pi.set_mode(self.rightReverse, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.leftForward, self.freq)
        self.pi.set_PWM_frequency(self.leftReverse, self.freq)
        self.pi.set_PWM_frequency(self.rightForward, self.freq)
        self.pi.set_PWM_frequency(self.rightReverse, self.freq)

        self.stopped = False
        Thread(target = self.update, args = self.currentOperationArgs).start()

    def stop(self):
        #print("Stopping motors")
        self._set_motor_duty(self.leftForward, 0)
        self._set_motor_duty(self.leftReverse, 0)
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
        duty = (pwr / 100.0) * 255
        self._set_motor_duty(motorPin, duty)

    def _set_motor_duty(self, motorPin, duty):
        duty = max(0, min(255, duty))
        self.duty[motorPin] = duty
    	self.pi.set_PWM_dutycycle(motorPin, duty)

    def forward(self, pwr):
        duty = (pwr / 100.0) * 255
        self._set_motor_duty(self.leftForward, duty)
        self._set_motor_duty(self.leftReverse, 0)
        self._set_motor_duty(self.rightForward, duty)
        self._set_motor_duty(self.rightReverse, 0)

    def _forwardPower(self, leftPower, rightPower):
        self._motorPower(self.leftForward, leftPower)
        self._motorPower(self.leftReverse, 0)
        self._motorPower(self.rightForward, rightPower)
        self._motorPower(self.rightReverse, 0)

    def reverse(self, pwr):
        duty = (pwr / 100.0) * 255
        self._set_motor_duty(self.leftForward, 0)
        self._set_motor_duty(self.leftReverse, duty)
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, duty)

    def rightTurn(self, pwr):
        duty = (pwr / 100.0) * 255
        self._set_motor_duty(self.leftForward, duty)
        self._set_motor_duty(self.leftReverse, 0)
        self._set_motor_duty(self.rightForward, 0)
        self._set_motor_duty(self.rightReverse, duty)

    def leftTurn(self, pwr):
        duty = (pwr / 100.0) * 255
        self._set_motor_duty(self.leftForward, 0)
        self._set_motor_duty(self.leftReverse, duty)
        self._set_motor_duty(self.rightForward, duty)
        self._set_motor_duty(self.rightReverse, 0)

    def getCurrentDutyCycle(self, motorPin):
        return self.duty[motorPin]

    def motorFunction(function, *parameters):
        #if function ==
        pass

    '''
    Takes a tick delta (say 0.01 s) and converts it to angular velocity (in rads/sec), using
    our known tick angle delta of 18 degrees
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

    def _mmToCm(self, mm):
        return mm / 10.0

    def _msToS(self, ms):
        return ms / 1000.0
