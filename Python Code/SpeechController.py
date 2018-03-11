'''
*	File: SpeechController.py
*	Description:  This module holds the SpeechController class. This is simply
*       an interface on top of some audio system calls.
*	Author(s):		Austin Dibble
*	Date Created:	2/2/18
'''
import pyttsx
import os
import time

'''
*	Class: SpeechController.py
*	Description:	SpeechController is a simple python interface on top of the
*       pico2wave program. It uses system calls to do TTS.
*	Author(s):		Austin Dibble
*	Date Created:	2/2/18
'''
class SpeechController:
    def __init__(self, voiceType = 1, output = 1, speechRate = 70, volume = 100):
        self.voiceType = voiceType
        self.HDMI = 2
        self.JACK = 1
        self.output = output
        self.speechRate = speechRate
        self.volume = volume

    #Start the voice engine
    def start(self):
        print ("Starting voice controller with voice engine " + str(self.voiceType) + " and output " + str(self.output))

        #If voiceType is set to 1, then we'll use pyttsx library.
        if self.voiceType == 0:
            self.engine = pyttsx.init()
            self.engine.setProperty('rate', self.speechRate)

        #Set the output type
        if self.output != 3:
            os.system("amixer cset numid=3 " + str(self.output) + " 2>/dev/null 1>/dev/null")

        #Set the volume
        os.system("amixer cset numid=3 " + str(self.volume) + "% 2>/dev/null 1>/dev/null")

    #Resource cleaning, we want to remove the temp file
    def stop(self):
        if self.voiceType == 1:
            try:
                os.remove("voice.wav")
            except:
                #In case we didn't make a file in the first place
                pass

    def speak(self, speechString):
        print("Saying \"" + speechString + "\"")
        #If voicetype is 0, use the speech engine
        #TODO fix this

        if self.voiceType == 0:
            self.engine.say(speechString)
            self.engine.runAndWait()
        #Else, let's build a couple of system calls
        else:
            os.system("pico2wave -w voice.wav \"" + speechString + "\" 2>/dev/null")
            time.sleep(0.5)
            os.system("aplay voice.wav 2>/dev/null")


    #Change the output. Options are HDMI, audio jack, or ?
    def changeOutput(self, newOutput):
        print("Changing speech output to output " + str(newOutput))
        self.output = newOutput
        os.system("amixer cset numid=3 " + str(self.output) + " 2>/dev/null 1>/dev/null")

    #Set the volume of the output.
    def setVolume(self, volume):
        self.volume = volume
        #OS call
        os.system("amixer cset numid=3 " + str(self.volume) + "% 2>/dev/null 1>/dev/null")

    #Convenience function that reads out a a given 3 digit voltage. Format is #.##
    def sayVoltage(self, voltage):
        #Get rid of the decimal
        voltage *= 100
        #Truncate the value
        voltage = int(voltage)
        #Get the hundreds place, and truncate the decimal
        hundreds = int(voltage / 100)
        #Get the tens place, and truncate the ones and decimals
        tens = int((voltage - (hundreds * 100)) / 10)
        #Get the ones.
        ones = int(voltage - (hundreds * 100) - (tens * 10))
        #Build the string and say it. "Battery is # point # # volts."
        self.speak("Battery is " + str(hundreds) + " point " + str(tens) + " " + str(ones) + " volts.")
