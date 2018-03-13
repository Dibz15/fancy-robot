'''
*	File: SensorsController.py
*	Description:	This module holds classes SensorsController and
*       Decoder.
*	Author(s):		Austin Dibble
*	Date Created:	1/28/18
'''
import serial
import Constants
import pigpio
from collections import deque
import time
from threading import Thread
import re


'''
*	Class: SensorsController
*	Description:	SensorsController is a threaded class which
*       takes care of the robot's serial communication and wheel decoder callbacks.
*       Through serial, it collects information from the ultrasonic sensor,
*       the IR sensor array, and the battery.
*	Author(s):		Austin Dibble
*	Date Created:	1/28/18
'''
class SensorsController:

    #__init__ is called whenever our class is instantiated.
    #ie. sensors = SensorsController() is actually calling this __init__ class, implicitly
    #parameters with default arguments are specified optionally
    def __init__(self, pi, ARDUINO_PORT = '/dev/ttyACM0', ARDUINO_BAUD = 9600, DECODER_CALLBACK = None):
        self.ARDUINO_PORT = ARDUINO_PORT                #Port for Arduino UNO/Leonardo is often /dev/ttyACM0 on RPi
        self.ARDUINO_BAUD = ARDUINO_BAUD                #Baud rate we set to 9600
        self.stopped = True                             #start out stopped
        self.readSerialString = ""                      #Place to read in current string
        self.dataQueue = deque(maxlen = 5)  #Buffer to hold last 5 recieved data packets
        #Data packets are in this order [Distance, IR Angle, Batt voltage]
        self.dataAverage = [5, 5, 5, 0]
        self.decoder = Decoder(pi, Constants.leftEncoder, Constants.rightEncoder, DECODER_CALLBACK)
        self.averageCounter = 0
        self.ser = None

        self.attempt = 0

        self.distanceCallback = None
        self.voltageCallback = None

        self.distanceCallbackDistance = 30
        self.voltageCallbackVoltage = 3.60 #3.50
        self.chargingCallbackVoltage = 3.50

        self.numUpdates = 0


    #Function to start up the sensors controller.
    #This must be used in order to use the class's data...
    def start(self):
        while self.attempt < 3:
            time.sleep(0.5)
            try:
                print("Attempting to open serial port " + self.ARDUINO_PORT)
                self.ser = serial.Serial(self.ARDUINO_PORT, self.ARDUINO_BAUD, timeout = 1)

                print("Serial interface opened.")

                time.sleep(0.2)
                matcher = re.compile('\n')
                tic     = time.time()
                buff    = self.ser.read(20)

                # you can use if not ('\n' in buff) too if you don't like re
                while ((time.time() - tic) < 1.2) and (not matcher.search(buff)):
                    buff += self.ser.read(20)

                #print("Buffer: " + str(buff))
                if len(buff) == 0:
                    print("Empty recieved buffer")
                    raise Exception('No contents read from serial port.')

                self.ser.flushInput()
                Thread(target = self.updateSerial, args=()).start()             #Start new thread on update function
                self.attempt = 10
            except:
                self.stopped = True
                if self.attempt == 0:
                    self.ARDUINO_PORT = '/dev/ttyAMA0'
                elif self.attempt == 1:
                    self.ARDUINO_PORT = '/dev/ttyUSB0'
                print("Could not open serial interface. Sensors unconnected.")
                self.attempt += 1


        self.stopped = False                                            #We aren't stopped now :)

        self.decoder.start()
        time.sleep(1)          #Start our decoder
        print("Sensors Controller started.")

    #This function runs in its own thread
    '''
    ****************************************************
    *Function: updateSerial
    *Description: Runs in its own thread, and cycles through and parses
    *   the serial data that is being recieved from the Arduino.
    *Parameters:
    *   self -
    *Returns:
    *   nothing
    ****************************************************
    '''
    def updateSerial(self):
        print("Serial thread started.")
        while not self.stopped:
            time.sleep(1 / 15.0 )
            #print("Reading serial string")
            self.readSerialString = self.ser.readline()                 #Read until newline
            #print("Serial string: " + self.readSerialString)
            splitString = self.readSerialString.split(',')              #Parse the data from between the commas

            #Add current parsed data into queue. Order is: Distance, IRAngle, BatteryVoltage
            #print("Data: " + self.readSerialString)

            try:
                #Make sure we have our data
                if splitString[0] != '' and splitString[1] != '' and splitString[2] != '' and splitString[3] != '':
                    #Add our parsed data into the data queue
                    self.dataQueue.appendleft([int(splitString[0]), int(splitString[1]), float(splitString[2]), float(splitString[3])])
                    #Count used for averaging
                    self.averageCounter += 1

                    #print("Distance: " + str(self.dataQueue[0][0]))
                    #Check for a critical distance value
                    if self.dataQueue[0][0] < self.distanceCallbackDistance and self.dataQueue[0][0] > 0:
                        #If we have a callback function
                        if self.distanceCallback is not None:
                            #print("Calling distance CB")
                            self.distanceCallback()

                    #Check for a critical voltage value
                    self.numUpdates += 1
                    compareVoltage = 0
                    if self.numUpdates > 20:
                        compareVoltage = self.dataAverage[2]
                    else:
                        compareVoltage = self.dataQueue[0][2]

                    #Voltage to compare with TEMP TODO for testing
                    compareVoltage = 3.5
                    #print("voltage is:" + str(self.dataAverage[2]))
                    if compareVoltage < self.voltageCallbackVoltage and self.dataQueue[0][2] > 0:
                        if self.voltageCallback is not None:
                            self.voltageCallback()

                #Make sure we have a full data queue, and we have been counting for averages
                if (len(self.dataQueue) >= 5) and self.averageCounter >= 3:
                    self.averageCounter = 0
                    totalUS = 0.0
                    totalIRAngle = 0.0
                    totalVoltage = 0.0
                    totalCharge = 0.0

                    dataQueueLen = len(self.dataQueue)

                    #Cycle through and sum data for averaging
                    for i in range(dataQueueLen):
                        d = self.dataQueue[i]
                        totalUS += d[0]
                        totalIRAngle += d[1]
                        totalVoltage += d[2]
                        totalCharge += d[3]

                    #Calculate the average from the sums
                    self.dataAverage = [int(totalUS / dataQueueLen), totalIRAngle / dataQueueLen, totalVoltage / dataQueueLen, totalCharge / dataQueueLen]
                    #print("Arduino data average: " + str(self.dataAverage))
            except:
                #If we had a serial error, just skip this packet
                pass

    #Return the IR angle stored in the dataAverage array
    def getIRAngle(self):
        return self.dataAverage[1]

    #Return the distance sensor distance stored in the dataAverage array
    def getUSDistance(self):
        if self.dataAverage[0] == 0:
            return -1
        else:
            return self.dataAverage[0]

    #Return the battery voltage stored in the dataAverage array
    def getBatteryVoltage(self):
        return self.dataAverage[2]

    def getChargeVoltage(self):
        return self.dataAverage[3]

    def isCharging(self):
        return (self.getChargeVoltage() > self.chargingCallbackVoltage)

    #You gotta call this to stop the thread
    #Otherwise it may not stop when you go to close the whole program :D
    #Plus, who wants resource leaks, right?
    def stop(self):
        self.stopped = True
        if self.ser is not None:
            time.sleep(0.5)
            self.ser.close()
        self.decoder.stop()
        print("Stopped sensors controller.")

    #Set the callback function of the decoder
    def setDecoderCallback(self, callbackFunction):
        self.decoder.setCallbackFunction(callbackFunction)

    #Get a reference to the decoder object
    def getDecoder(self):
        return self.decoder

    def setDistanceCallback(self, callback):
        self.distanceCallback = callback

    def setVoltageCallback(self, callback):
        self.voltageCallback = callback

'''
*	Class: Decoder.py
*	Description:	Decoder is a class that sets up callback functions in order
*       to recieve accurate timing information from the wheel encoders.
*	Author(s):		Austin Dibble
*	Date Created:	1/28/18
'''
class Decoder:

    #Note: Callback function should have a parameter that expects an int for the pin number
    def __init__(self, pi, leftEncoderPin, rightEncoderPin, callbackFunction = None):

        #Set the variables given in the constructor
        self.pi = pi
        self.leftEncoderPin = int(leftEncoderPin)
        self.rightEncoderPin = int(rightEncoderPin)
        self.callbackFunction = callbackFunction

        #self.leftEncoderPin = int(Constants.leftEncoder)
        #self.rightEncoderPin = int(Constants.rightEncoder)

        #Set the pin mode of each of these GPIO to input
        self.pi.set_mode(self.leftEncoderPin, pigpio.INPUT)
        self.pi.set_mode(self.rightEncoderPin, pigpio.INPUT)

        #Make sure the pins are floating (they'll be driven by the decoder)
        self.pi.set_pull_up_down(self.leftEncoderPin, pigpio.PUD_OFF)
        self.pi.set_pull_up_down(self.rightEncoderPin, pigpio.PUD_OFF)

        #Max number of deltas to store (for averaging), or for memory
        self.deltaStore = 5

        #Queue for left wheel deltas
        self.Ldeltas = deque(maxlen = self.deltaStore)          #Deque to show the deltas for the last 5 ticks (could be for averaging)

        #Queue for right wheel deltas
        self.Rdeltas = deque(maxlen = self.deltaStore)

        #initialize the queues
        self.Ldeltas.appendleft(0)
        self.Rdeltas.appendleft(0)

        self.counter = [0, 0]                                    #Counter to use for number of ticks in a certain time

        #Used to calculate the delta time of each wheel
        self.lastTickTime = [time.time() * 1000, time.time() * 1000]

        #Ints to designate which wheel we're talking about
        self.LEFT = 0
        self.RIGHT = 1

    '''
    ****************************************************
    *Function: getDeltaAverages
    *Description: Calculates the average of all the deltas.
    *   If the memory is too large, this may be inaccurate.
    *Parameters:
    *   self -
    *Returns:
    *   left and right wheel averages tuple
    ****************************************************
    '''
    def getDeltaAverages(self):
        dSums = [0, 0]

        if (len(self.Ldeltas) == 0) or (len(self.Rdeltas) == 0):
            return dSums

        rnge = min(len(self.Rdeltas), len(self.Ldeltas))

        for i in range(rnge - 1):
            dSums[self.LEFT] += self.Ldeltas[i]
            dSums[self.RIGHT] += self.Rdeltas[i]

        dSums[self.LEFT] /= len(self.Ldeltas)
        dSums[self.RIGHT] /= len(self.Rdeltas)

        return dSums

    #Start up the class
    def start(self):
        #Set the inner callback of the left wheel pin to trigger on a falling edge
        self.cbLeft = self.pi.callback(self.leftEncoderPin, pigpio.FALLING_EDGE, self._pulse)
        #Set the inner callback of the right wheel pin to trigger on a falling edge
        self.cbRight = self.pi.callback(self.rightEncoderPin, pigpio.FALLING_EDGE, self._pulse)
        print("Decoder started.")

    #in case we want to change the callback function sometime
    def setCallbackFunction(self, callbackFunction):
        self.callbackFunction = callbackFunction
        print("Decoder callback defined.")

    #Get the last time there was a tick
    def getLastTickTime(self):
        return self.lastTickTime

    #Get the current tick counter values
    def getCounter(self):
        return self.counter

    #Get the current raw delta values for each wheels
    def getDeltas(self):
        return (self.Ldeltas, self.Rdeltas)

    #Clear out the deltas
    def clearDeltas(self):
        self.Ldeltas.clear()
        self.Rdeltas.clear()

    #Reset a counter for a
    def resetCounter(self, side):
        self.counter[side] = 0

    #Reset the counter for both wheels
    def resetCounters(self):
        self.resetCounter(self.LEFT)
        self.resetCounter(self.RIGHT)

    #Callback trigger function
    #gpio gives the pin that triggered this callback
    #level:
    #   0 - falling edge
    #   1 - rising edge
    #   2 - no edge change (watchdog timeout)
    #Tick:
    #Number of microseconds since boot
    def _pulse(self, gpio, level, tick):
        """
        Decode the encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1
        """
        #print("Pin " + str(gpio) + ": " + str(level))
        #If we have a callback function set
        if self.callbackFunction is not None:

            #If the call was the left wheel
            if gpio == self.leftEncoderPin:
                #Increment our count
                self.counter[self.LEFT] += 1
                #Call the callback, with the pin number
                self.callbackFunction(int(gpio))
                #Append our delta
                self.Ldeltas.appendleft( (time.time() * 1000) - self.lastTickTime[self.LEFT] )
                #Update our last tick time
                self.lastTickTime[self.LEFT] = time.time() * 1000

            #If the call was the right wheel
            elif gpio == self.rightEncoderPin:
                self.counter[self.RIGHT] += 1
                self.callbackFunction(int(gpio))
                self.Rdeltas.appendleft((time.time() * 1000) - self.lastTickTime[self.RIGHT])
                self.lastTickTime[self.RIGHT] = time.time() * 1000

            #print("D: (" + str(self.Ldeltas[0]) + ", " + str(self.Rdeltas[0]) + ")")

    #Stop everthing!
    #Disassociate the callbacks, basically.
    #That's it.
    def stop(self):
        """
        Cancel the decoder's callbacks.
        """
        self.cbLeft.cancel()
        self.cbLeft.cancel()
        print("Stopped Decoder.")
