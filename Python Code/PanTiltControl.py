from RPIO import PWM

class PanTiltControl:
    def __init__(self, panPin, tiltPin):
        self.servo = PWM.Servo()
        self.panPin = panPin
        self.tiltPin = tiltPin

        return

    def start(self) :
        self.servo.set_servo(self.tiltPin, 1500)
        self.servo.set_servo(self.panPin, 1500)

    def stop(self) :
        self.servo.stop_servo(self.tiltPin)
        self.servo.stop_servo(self.panPin)
        PWM.cleanup()

    def center(self):
        self.servo.set_servo(self.tiltPin, 1500)
        self.servo.set_servo(self.panPin, 1500)

    def setServo(self, servoPin, angle):
        angle = max(0, min(90, angle))
        self.servo.set_servo(servoPin, (angle / 90 * 1000) + 1000)

    def getTiltPin(self):
        return self.tiltPin

    def getPanPin(self):
        return self.panPin
