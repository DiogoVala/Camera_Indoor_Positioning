
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

# Camera Settings
camera_resolution = (4056, 3040)
camera_resolution = (1280, 720)

####### MAIN ####### 
print("Starting server camera.")

# Camera startup
camera = picamera.PiCamera()
camera.resolution = camera_resolution
#camera.exposure_mode = 'sports'
#camera.iso 	= 1600
#camera.shutter_speed = 10000
camera.framerate = 30
time.sleep(1)
capture = PiYUVArray(camera, size=camera_resolution)

cnt=0
start = time.time()
for frame in camera.capture_continuous(capture, use_video_port=True, format='yuv'):
	frame=capture.array
	capture.truncate(0)
	if cnt == 50:
		fps=(time.time()-start)/50
		print(fps)
		break
	cnt+=1

