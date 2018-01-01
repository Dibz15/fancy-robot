import pigpio
import time

class MotorController:

    def __init__(self, pi, leftForward, leftReverse, rightForward, rightReverse):
        self.pi = pi
        self.leftForward = leftForward
        self.leftReverse = leftReverse
        self.rightForward = rightForward
        self.rightReverse = rightReverse

    def start(self):
        self.pi.set_mode(self.leftForward, pigpio.OUTPUT)
        self.pi.set_mode(self.leftReverse, pigpio.OUTPUT)
        self.pi.set_mode(self.rightForward, pigpio.OUTPUT)
        self.pi.set_mode(self.rightReverse, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.leftForward, 20)
        self.pi.set_PWM_frequency(self.leftReverse, 20)
        self.pi.set_PWM_frequency(self.rightForward, 20)
        self.pi.set_PWM_frequency(self.rightReverse, 20)

    def stop(self):
        set_motor_duty(self.leftForward, 0)
        set_motor_duty(self.leftReverse, 0)
        set_motor_duty(self.rightForward, 0)
        set_motor_duty(self.rightReverse, 0)

    def set_motor_duty(self, motorPin, duty):
    	self.pi.set_PWM_dutycycle(motorPin, duty)

    def forward(self, pwr):
        duty = (pwr / 100.0) * 255
        set_motor_duty(self.leftForward, duty)
        set_motor_duty(self.leftReverse, 0)
        set_motor_duty(self.rightForward, duty)
        set_motor_duty(self.rightReverse, 0)

    def reverse(self, pwr):
        duty = (pwr / 100.0) * 255
        set_motor_duty(self.leftForward, 0)
        set_motor_duty(self.leftReverse, duty)
        set_motor_duty(self.rightForward, 0)
        set_motor_duty(self.rightReverse, duty)

    def rightTurn(self, pwr):
        duty = (pwr / 100.0) * 255
        set_motor_duty(self.leftForward, duty)
        set_motor_duty(self.leftReverse, 0)
        set_motor_duty(self.rightForward, 0)
        set_motor_duty(self.rightReverse, duty)

    def leftTurn(self, pwr):
        duty = (pwr / 100.0) * 255
        set_motor_duty(self.leftForward, 0)
        set_motor_duty(self.leftReverse, duty)
        set_motor_duty(self.rightForward, duty)
        set_motor_duty(self.rightReverse, 0)
