'''
*	File: PID
*	Description: Creado por el control de sistemas que
*       estan describido aqui https://en.wikipedia.org/wiki/PID_controller
*	Author(s):		Austin Dibble
*	Date Created:	2/15/18
'''
import time

'''
*	Class: PID
*	Description:	Este clase se utiliza para controlar sistemas
*       en manera "PID". Ejemplos de uso: motores, velocidad, direccion, etc.
*	Author(s):		Austin Dibble
*	Date Created:	2/15/18
'''
class PID:
    def __init__(self, initSetpoint = 0.0, Kp = 0.2, Ki = 0.0, Kd = 0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.currentTime = time.time()
        self.lastTime = self.currentTime

        self.setPoint = initSetpoint

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.lastError = 0.0

        # Windup Guard
        self.Imax = 20.0

        self.output = 0.0

        return

    '''
    ****************************************************
    *Function: update
    *Description: Given a value to compare to, this produces a new output value
        based on the PID equation. The error is the supplied value subtracted
        from the setpoint
    *Parameters:
    *   self -
    *   actualValue - the actual value to use for calculating PID error
    *Returns:
    *   nothing
    ****************************************************
    '''
    def update(self, actualValue):
        error = self.setPoint - actualValue

        errorDelta = error - self.lastError
        #print("Error: " + str(error))
        timeDelta = time.time() - self.lastTime

        #Set the p term = E
        self.P = error

        #Set the I term = E*dt
        self.I += error * timeDelta

        #Make sure I doesn't grow too large
        self.I = min( max(self.I, -self.Imax), self.Imax )

        #Set the D term = dE / dt
        self.D = 0.0
        if timeDelta > 0:
            self.D = errorDelta / timeDelta

        self.lastError = error
        self.lastTime = time.time()

        #Our output = P*Kp + I*Ki + D * Kd
        self.output = (self.P * self.Kp) + (self.I * self.Ki) + (self.D * self.Kd)


    #Clears the variables to restart the PID loop.
    def clear(self):
        self.setPoint = 0.0

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.lastError = 0.0

        # Windup Guard
        self.Imax = 20.0

        self.output = 0.0

    #Set a max value for the I term for use as a windup guard.
    def setIMax(self, Imax):
        self.Imax = Imax

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setSetpoint(self, setpoint):
        self.setPoint = setpoint

    def getSetpoint(self):
        return self.setPoint

    def getOutput(self):
        return self.output

"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline


pid = PID(0.0, Kp = 0.9, Ki = 0.2, Kd = 0.002)
END = 100
feedback = 0

feedback_list = []
time_list = []
setpoint_list = []

for i in range(1, END):
    pid.update(feedback)
    output = pid.getOutput()

    if pid.getSetpoint() > 0:
        feedback += (output - (5 / i))

    if i > 9:
        pid.setSetpoint(10)

    if i > 15:
        pid.setSetpoint(12)

    if i > 25:
        pid.setSetpoint(2)

    time.sleep(0.02)

    feedback_list.append(feedback)
    setpoint_list.append(pid.getSetpoint())
    time_list.append(i)

time_sm = np.array(time_list)
time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
feedback_smooth = spline(time_list, feedback_list, time_smooth)

plt.plot(time_smooth, feedback_smooth)
plt.plot(time_list, setpoint_list)
plt.xlim((0, END))
plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
plt.xlabel('time (s)')
plt.ylabel('PID (PV)')
plt.title('TEST PID')

#plt.ylim((1-0.5, 1+0.5))

plt.grid(True)
#plt.show()

plt.savefig("PID.jpg")
"""
