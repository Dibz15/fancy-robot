from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

width = 640
height = 480

#initialize pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24
camera.vflip = True
camera.hflip = True
rawCapture = PiRGBArray(camera, size=(640, 480))

#allow camera to turn on
time.sleep(0.1)

#grab image from the PiCamera
#camera.capture(rawCapture, format="bgr")
#image = rawCapture.array

#Capture a single image
#camera.capture('image.jpg')

#grab video from the PiCamera
#camera.start_recording('video.h264')
#time.sleep(5)
#camera.stop_recording()

#display image and wait for keypress
#cv2.imshow("Image", image)
#cv2.waitkey(0)

'''
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

'''
