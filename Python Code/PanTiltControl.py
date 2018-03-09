'''
*	File: PanTiltControl.py
*	Description:	This module holds the PanTiltControl class
*	Author(s):		Austin Dibble
*	Date Created:	12/24/18
'''
import pigpio
import time

'''
*	Class: PanTiltControl.py
*	Description: PanTiltControl is a class used as an abstraction layer between
*       the user and the servo angle controls.
*	Author(s):		Austin Dibble
*	Date Created:	12/24/18
'''
class PanTiltControl:
    def __init__(self, panPin, tiltPin, pi):
        self.panPin = panPin
        self.tiltPin = tiltPin
        self.pi = pi
        self.angle = {}

        self.tilt = tiltPin
        self.pan = panPin

        return

    '''
    ****************************************************
    *Function: pointDown
    *Description:
    *   Points the camera down
    *Parameters:
    *   self -
    *Returns:
    *   nothing
    ****************************************************
    '''
    def pointDown(self):
        #print("Pointing down")
        self.setServo(self.tilt, 135)

    '''
    ****************************************************
    *Function: start
    *Description:
    *   Sets the pulsewidth of the servos to 50%, and waits
    *   0.5 seconds
    *Parameters:
    *   self -
    *Returns:
    *   nothing
    ****************************************************
    '''
    def start(self) :
        print("Starting pan and tilt servos")
        self.pi.set_servo_pulsewidth(self.tiltPin, 1500)
        self.pi.set_servo_pulsewidth(self.panPin, 1500)
        self.angle[self.pan] = 90
        self.angle[self.tilt] = 90
        time.sleep(0.5)

    '''
    ****************************************************
    *Function: stop
    *Description:
    *   Stops the servos
    *Parameters:
    *   self -
    *Returns:
    *   nothing
    ****************************************************
    '''
    def stop(self) :
        print("Stopping servos")
        self.pi.set_servo_pulsewidth(self.tiltPin, 0)
        self.pi.set_servo_pulsewidth(self.panPin, 0)

    '''
    ****************************************************
    *Function: center
    *Description:
    *   Puts both servos at 45 degrees
    *Parameters:
    *   self -
    *Returns:
    *   nothing
    ****************************************************
    '''
    def center(self):
        self.pi.set_servo_pulsewidth(self.tiltPin, 1500)
        self.pi.set_servo_pulsewidth(self.panPin, 1500)
        self.angle[self.tilt] = 90
        self.angle[self.pan] = 90

    '''
    ****************************************************
    *Function: setServo
    *Description:
    *   Allows for setting the servo's angle.
    *Parameters:
    *   self -
    *   servo - const refering to tilt or pan (pin)
    *   angle - angle to set it to (0-90*)
    *Returns:
    *   nothing
    ****************************************************
    '''
    def setServo(self, servo, angle):
        angle = max(0, min(180, angle))
        pulsewidth = int((angle / 180.0 * 2000.0) + 500)
        self.pi.set_servo_pulsewidth(servo, pulsewidth)
        self.angle[servo] = angle

    '''
    ****************************************************
    *Function: incServo
    *Description:
    *   Convenience function to increment/decrement a servo's
    *   angle by some amount
    *Parameters:
    *   self -
    *   servo - servo pin
    *   amount - amount in degrees to increment by
    *Returns:
    *   nothing
    ****************************************************
    '''
    def incServo(self, servo, amount):
        self.angle[servo] = int(self.angle[servo]) + amount
        self.angle[servo] = max(0, min(180, self.angle[servo]))
        self.setServo(servo, self.angle[servo])

    def getTiltPin(self):
        return self.tiltPin

    def getPanPin(self):
        return self.panPin

    def getCurrentAngle(self, servo):
        return self.angle[servo]
