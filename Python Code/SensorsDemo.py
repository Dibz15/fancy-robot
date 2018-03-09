'''
*	File: SensorsDemo.py
*	Description:	This module is for demoing the function of the sensors systems.
*	Author(s):		Austin Dibble
*	Date Created:	02/01/18
'''

#Imports
from SensorsController import *
from MotorController import *
import pigpio
from SpeechController import *

#Initialize our pigpio reference
pi = pigpio.pi()

#Start the speech controller
#speech = SpeechController()
##.start()

#Say what we're doing here
#speech.speak("Program starting.")

#speech.speak("Booting sensors.")

#Boot sensors module
sensors = SensorsController(pi)
sensors.start()

#Start our motor controller
#speech.speak("Preparing motors.")
time.sleep(1)
try:

    motors = MotorController(pi, decoder = sensors.getDecoder())
    motors.start()

except NameError:
    print("Problem importing motor controller.")
    quit()

time.sleep(5)





#speech.speak("Shutdown sequence commencing.")

#Close up our resources
motors.stop()
sensors.stop()
#speech.stop()
pi.stop()
