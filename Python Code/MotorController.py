import pigpio
import time
from collections import deque
from threading import Thread

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
        print("Callback")
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
