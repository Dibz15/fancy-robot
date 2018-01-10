import pigpio
import time

class MotorController:

    def __init__(self, pi, leftForward, leftReverse, rightForward, rightReverse, freq = 50):
        self.pi = pi
        self.leftForward = leftForward
        self.leftReverse = leftReverse
        self.rightForward = rightForward
        self.rightReverse = rightReverse
        self.duty = {}
        self.freq = 50

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

    def stop(self):
        #print("Motors stopped")
        self.set_motor_duty(self.leftForward, 0)
        self.set_motor_duty(self.leftReverse, 0)
        self.set_motor_duty(self.rightForward, 0)
        self.set_motor_duty(self.rightReverse, 0)

    def leftStep(self, pwr):
        self.motorPower(self.rightForward, 0)
        self.motorPower(self.rightReverse, 0)
        self.motorPower(self.leftForward, pwr)
        self.motorPower(self.leftReverse, 0)

    def rightStep(self, pwr):
        self.motorPower(self.leftForward, 0)
        self.motorPower(self.leftReverse, 0)
        self.motorPower(self.rightForward, pwr)
        self.motorPower(self.rightReverse, 0)

    def motorPower(self, motorPin, pwr):
        duty = (pwr / 100.0) * 255
        self.set_motor_duty(motorPin, duty)

    def set_motor_duty(self, motorPin, duty):
        duty = max(0, min(255, duty))
        self.duty[motorPin] = duty
    	self.pi.set_PWM_dutycycle(motorPin, duty)

    def forward(self, pwr):
        duty = (pwr / 100.0) * 255
        self.set_motor_duty(self.leftForward, duty)
        self.set_motor_duty(self.leftReverse, 0)
        self.set_motor_duty(self.rightForward, duty)
        self.set_motor_duty(self.rightReverse, 0)

    def forwardPower(self, leftPower, rightPower):
        self.motorPower(self.leftForward, leftPower)
        self.motorPower(self.leftReverse, 0)
        self.motorPower(self.rightForward, rightPower)
        self.motorPower(self.rightReverse, 0)

    def reverse(self, pwr):
        duty = (pwr / 100.0) * 255
        self.set_motor_duty(self.leftForward, 0)
        self.set_motor_duty(self.leftReverse, duty)
        self.set_motor_duty(self.rightForward, 0)
        self.set_motor_duty(self.rightReverse, duty)

    def rightTurn(self, pwr):
        duty = (pwr / 100.0) * 255
        self.set_motor_duty(self.leftForward, duty)
        self.set_motor_duty(self.leftReverse, 0)
        self.set_motor_duty(self.rightForward, 0)
        self.set_motor_duty(self.rightReverse, duty)

    def leftTurn(self, pwr):
        duty = (pwr / 100.0) * 255
        self.set_motor_duty(self.leftForward, 0)
        self.set_motor_duty(self.leftReverse, duty)
        self.set_motor_duty(self.rightForward, duty)
        self.set_motor_duty(self.rightReverse, 0)

    def getCurrentDutyCycle(self, motorPin):
        return self.duty[motorPin]
