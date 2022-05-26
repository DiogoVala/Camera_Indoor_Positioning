
import io
import time
import threading
import picamera
import datetime
import math
import cv2
import numpy as np
import sys
from picamera.array import PiYUVArray
from picamera.array import PiRGBArray

# Camera Settings
camera_resolution = (1280, 720)

####### MAIN ####### 
print("Starting server camera.")

# Camera startup
camera = picamera.PiCamera()
camera.resolution = camera_resolution
camera.exposure_mode = 'sports'
camera.iso 	= 1600
camera.shutter_speed = 10000
camera.framerate = 15
time.sleep(1)
capture = PiRGBArray(camera, size=camera_resolution)

while True:
	camera.capture(capture, use_video_port=True, format='bgr')
	stamp = time.time()
	print(stamp)
	frame=capture.array
	capture.truncate(0)
	cv2.imshow("frame", frame)
	
	key=cv2.waitKey(1)
