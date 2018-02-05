import serial
import Constants
import pigpio
from collections import deque
import time
from threading import Thread

class SensorsController:
    #__init__ is called whenever our class is instantiated.
    #ie. sensors = SensorsController() is actually calling this __init__ class, implicitly
    #parameters with default arguments are specified optionally
    def __init__(self, pi, ARDUINO_PORT = "/dev/ttyACM0", ARDUINO_BAUD = 9600, DECODER_CALLBACK = None):
        self.ARDUINO_PORT = ARDUINO_PORT                #Port for Arduino UNO/Leonardo is often /dev/ttyACM0 on RPi
        self.ARDUINO_BAUD = ARDUINO_BAUD                #Baud rate we set to 9600
        self.stopped = True                             #start out stopped
        self.readSerialString = ""                      #Place to read in current string
        self.dataQueue = deque(maxlen = 5)  #Buffer to hold last 5 recieved data packets
        #Data packets are in this order [Distance, IR Angle, Batt voltage]
        self.dataAverage = [0, 0, 0]
        self.decoder = Decoder(pi, DECODER_CALLBACK)
        self.averageCounter = 0

    #Function to start up the sensors controller.
    #This must be used in order to use the class's data...
    def start(self):
        self.ser = serial.Serial(self.ARDUINO_PORT, self.ARDUINO_BAUD, timeout = 1)  #Open Serial port
        self.ser.flushInput()                                           #Flush serial buffer

        self.stopped = False                                            #We aren't stopped now :)

        Thread(target = self.updateSerial, args=()).start()                   #Start new thread on update function
        self.decoder.start()
        print("Sensors Controller started.")

    #This function runs in its own thread
    def updateSerial(self):
        while not self.stopped:
            self.readSerialString = self.ser.readline()                 #Read until newline
            splitString = self.readSerialString.split(',')              #Parse the data from between the commas

            #Add current parsed data into queue. Order is: Distance, IRAngle, BatteryVoltage

            try:
                if splitString[0] != '' and splitString[1] != '' and splitString[2] != '':
                    self.dataQueue.appendleft([int(splitString[0]), int(splitString[1]), float(splitString[2])])
                    #print("Arduino Data: " + str(self.dataQueue[0]))
                    self.averageCounter += 1

                if (len(self.dataQueue) >= 5) and self.averageCounter >= 3:
                    self.averageCounter = 0
                    totalUS = 0.0
                    totalIRAngle = 0.0
                    totalVoltage = 0.0

                    for i in range(len(self.dataQueue)):
                        d = self.dataQueue[i]
                        totalUS += d[0]
                        totalIRAngle += d[1]
                        totalVoltage += d[2]

                    self.dataAverage = [int(totalUS / 5.0), totalIRAngle / 5.0, totalVoltage / 5.0]
                    #print("Arduino data average: " + str(self.dataAverage))
            except:
                pass

    def getIRAngle(self):
        return self.dataAverage[1]

    def getUSDistance(self):
        return self.dataAverage[0]

    def getBatteryVoltage(self):
        return self.dataAverage[2]

    #You gotta call this to stop the thread
    #Otherwise it may not stop when you go to close the whole program :D
    #Plus, who wants resource leaks, right?
    def stop(self):
        self.stopped = True
        self.ser.close()
        self.decoder.stop()
        print("Stopped sensors controller.")

    def setDecoderCallback(self, callbackFunction):
        self.decoder.setCallbackFunction(callbackFunction)

    def getDecoder(self):
        return self.decoder


class Decoder:

    #Note: Callback function should have a parameter that expects an int for the pin number
    def __init__(self, pi, leftEncoderPin = Constants.leftEncoder, rightEncoderPin = Constants.rightEncoder, callbackFunction = None):
        """
        Instantiate the class with the pi and gpios connected to
        rotary encoder contacts A and B.  The common contact
        should be connected to ground.  The callback is
        called when the rotary encoder is turned.
        """

        self.pi = pi
        self.leftEncoderPin = leftEncoderPin
        self.rightEncoderPin = rightEncoderPin
        self.callbackFunction = callbackFunction

        self.leftEncoderPin = int(Constants.leftEncoder)
        self.rightEncoderPin = int(Constants.rightEncoder)

        self.pi.set_mode(self.leftEncoderPin, pigpio.INPUT)
        self.pi.set_mode(self.rightEncoderPin, pigpio.INPUT)

        self.pi.set_pull_up_down(self.leftEncoderPin, pigpio.PUD_OFF)
        self.pi.set_pull_up_down(self.rightEncoderPin, pigpio.PUD_OFF)

        self.deltaStore = 15

        self.Ldeltas = deque(maxlen = self.deltaStore)          #Deque to show the deltas for the last 5 ticks (could be for averaging)
        self.Rdeltas = deque(maxlen = self.deltaStore)
        self.Ldeltas.appendleft(0)
        self.Rdeltas.appendleft(0)

        self.counter = [0, 0]                        #Counter to use for number of ticks in a certain time
        self.lastTickTime = [time.time() * 1000, time.time() * 1000]       #Time in milliseconds

        self.LEFT = 0
        self.RIGHT = 1


    def getDeltaAverages(self):
        dSums = [0, 0]
        for i in range(len(self.Rdeltas) - 1):
            dSums[self.LEFT] += self.Ldeltas[i]
            dSums[self.RIGHT] += self.Rdeltas[i]

        dSums[self.LEFT] /= len(self.Ldeltas)
        dSums[self.RIGHT] /= len(self.Rdeltas)

        return dSums


    def start(self):
        self.cbLeft = self.pi.callback(self.leftEncoderPin, pigpio.FALLING_EDGE, self._pulse)
        self.cbRight = self.pi.callback(self.rightEncoderPin, pigpio.FALLING_EDGE, self._pulse)
        print("Decoder started.")

    def setCallbackFunction(self, callbackFunction):
        self.callbackFunction = callbackFunction
        print("Decoder callback defined.")

    def getLastTickTime(self):
        return self.lastTickTime

    def getCounter(self):
        return self.counter

    def getDeltas(self):
        return self.deltas

    def resetCounter(self, side):
        self.counter[side] = 0

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
        if self.callbackFunction is not None:
            if gpio == self.leftEncoderPin:
                self.counter[self.LEFT] += 1
                self.callbackFunction(int(gpio))
                self.Ldeltas.appendleft( (time.time() * 1000) - self.lastTickTime[self.LEFT] )
                self.lastTickTime[self.LEFT] = time.time() * 1000
            elif gpio == self.rightEncoderPin:
                self.counter[self.RIGHT] += 1
                self.callbackFunction(int(gpio))
                self.Rdeltas.appendleft((time.time() * 1000) - self.lastTickTime[self.RIGHT])
                self.lastTickTime[self.RIGHT] = time.time() * 1000

            #print("D: (" + str(self.Ldeltas[0]) + ", " + str(self.Rdeltas[0]) + ")")

    def stop(self):
        """
        Cancel the decoder's callbacks.
        """
        self.cbLeft.cancel()
        self.cbLeft.cancel()
        print("Stopped Decoder.")
