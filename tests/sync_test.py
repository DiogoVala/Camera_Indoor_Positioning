
import io
import time
import threading
import picamera
import datetime
import math
import cv2
import numpy as np
import sys
from numpy.linalg import inv
from numpy import array, cross
from numpy.linalg import solve, norm
from scipy.spatial.transform import Rotation
import csv

# Add common modules path
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/common')

import image_processor as imgp
# Camera Settings
camera_resolution = (2016, 1520)

# Processing pipeline for each frame
def frame_processor(frame):
	
	name=str(datetime.datetime.now())+'.jpg'
	
	rgb=cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)
	cv2.imwrite(name, rgb)
	
	return


####### MAIN ####### 
print("Starting server camera.")


# Camera startup
camera = picamera.PiCamera()
camera.resolution = camera_resolution
camera.exposure_mode = 'sports'
camera.iso 	= 1600
camera.shutter_speed = 5000
camera.framerate = 60
time.sleep(1)

# Initialize pool of threads to process each frame
imgp.ImgProcessorPool = [imgp.ImageProcessor(frame_processor, camera, camera_resolution) for i in range(imgp.nProcess)]

print("Starting tracking.\n")
camera.capture_sequence(imgp.getStream(), use_video_port=True, format='yuv')

while imgp.ImgProcessorPool :
	with imgp.ImgProcessorLock:
		processor = imgp.ImgProcessorPool.pop()
	processor.terminated = True
	processor.join()
print("Terminating program.")
