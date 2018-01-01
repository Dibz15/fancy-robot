import pigpio
import time

class PanTiltControl:
    def __init__(self, panPin, tiltPin, pi):
        self.panPin = panPin
        self.tiltPin = tiltPin
        self.pi = pi
        self.angle = 45

        return

    def start(self) :
        print("Starting pan and tilt servos")
        self.pi.set_servo_pulsewidth(self.tiltPin, 1500)
        self.pi.set_servo_pulsewidth(self.panPin, 1500)
        time.sleep(0.5)
        print("Servos started")

    def stop(self) :
        print("Stopping servos")
        self.pi.set_servo_pulsewidth(self.tiltPin, 0)
        self.pi.set_servo_pulsewidth(self.panPin, 0)
        print("Servos stopped")

    def center(self):
        self.pi.set_servo_pulsewidth(self.tiltPin, 1500)
        self.pi.set_servo_pulsewidth(self.panPin, 1500)
        self.angle = 45

    def setServo(self, servoPin, angle):
        angle = max(0, min(90, angle))
        pulsewidth = int((angle / 90.0 * 1000.0) + 1000)
        self.pi.set_servo_pulsewidth(servoPin, pulsewidth)
        self.angle = angle

    def incServo(self, servoPin, amount):
        self.angle = self.angle + amount
        self.angle = max(0, min(90, self.angle))
        self.pi.set_servo_pulsewidth(servoPin, (self.angle / 90 * 1000) + 1000)

    def getTiltPin(self):
        return self.tiltPin

    def getPanPin(self):
        return self.panPin

    def getCurrentAngle(self):
        return self.angle
