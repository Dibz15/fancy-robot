

import signal

class BreakHandler:

    def __init__(self, motors, sensors, panTilt, rCV, ai, speech, pi):
            self.motors = motors
            self.sensors = sensors
            self.panTilt = panTilt
            self.rCV = rCV
            self.ai = ai
            self.speech = speech
            self.pi = pi

            signal.signal(signal.SIGINT, self)

    def __call__(self, signame, sf):
        #We recieved a CTRL^C command
        print("Break called, stopping everything.")
        self.ai.stop()
        self.rCV.stop()
