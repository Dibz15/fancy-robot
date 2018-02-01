import pyttsx
import os
import time

class SpeechController:
    def __init__(self, voiceType = 1, output = 1):
        self.voiceType = voiceType
        self.HDMI = 2
        self.JACK = 1
        self.output = output

    def start(self):
        print ("Starting voice controller with voice engine " + str(self.voiceType) + " and output " + str(self.output))

        if self.voiceType == 0:
            engine = pyttsx.init()
            engine.setProperty('rate', 70)

        os.system("amixer cset numid=3 " + str(self.output) + " 2>/dev/null 1>/dev/null")

    def stop(self):
        if self.voiceType == 1:
            os.remove("voice.wav")

    def speak(self, speechString):
        print("Saying \"" + speechString + "\"")
        if self.voiceType == 0:
            engine.say(speechString)
            engine.runAndWait()
        else:
            os.system("pico2wave -w voice.wav \"" + speechString + "\" 2>/dev/null")
            os.system("aplay voice.wav 2>/dev/null")

    def changeOutput(self, newOutput):
        print("Changing speech output to output " + str(newOutput))
        self.output = newOutput
        os.system("amixer cset numid=3 " + str(self.output) + " 2>/dev/null 1>/dev/null")

    def sayVoltage(self, voltage):
        voltage *= 100
        voltage = int(voltage)
        for j in range(3):
            if j == 0:
                hundreds = int(voltage / 100)
            if j == 1:
                tens = int((voltage - (hundreds * 100)) / 10)
            if j == 2:
                ones = int(voltage - (hundreds * 100) - (tens * 10))
                self.speak("Battery is " + str(hundreds) + " point " + str(tens) + " " + str(ones) + " volts")