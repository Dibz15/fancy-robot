import pigpio
import time
import Constants

from MotorController import *
from SensorsController import *
from SpeechController import *
from aiController import *
from aiState import *
from PanTiltControl import *
from threading import Thread
from OpenCV.RoboCV import *

rCV = RoboCV()
rCV.start()

rCV.setCurrentVisionFunction("LineFollower")

while rCV.getCurrentVisionFunctionValue("lineImage") is None:
    print("Waiting for image")
    time.sleep(0.2)
    continue

rCV.stop()
