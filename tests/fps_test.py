# Fast reading from the raspberry camera with Python, Numpy, and OpenCV
# Allows to process grayscale video up to 124 FPS (tested in Raspberry Zero Wifi with V2.1 camera)
#
# Made by @CarlosGS in May 2017
# Club de Robotica - Universidad Autonoma de Madrid
# http://crm.ii.uam.es/
# License: Public Domain, attribution appreciated

import cv2
import numpy as np
import subprocess as sp
import time
from cv2 import aruco
import atexit

max_frames = 10

N_frames = 0

# Video capture parameters
(w,h) = (2016, 1520)
bytesPerFrame = w * h
fps = 5 # setting to 250 will request the maximum framerate possible

# "raspividyuv" is the command that provides camera frames in YUV format
#  "--output -" specifies stdout as the output
#  "--timeout 0" specifies continuous video
#  "--luma" discards chroma channels, only luminance is sent through the pipeline
# see "raspividyuv --help" for more information on the parameters
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview -ex sports -ISO 100"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string

#cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE) # start the camera
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=1)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly

# wait for the first frame and discard it (only done to measure time more accurately)q
#rawStream = cameraProcess.stdout.read(bytesPerFrame)

print("Recording...")

start_time = time.time()

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
parameters =  aruco.DetectorParameters_create()

while True:
	cameraProcess.stdout.flush() # discard any frames that we were not able to process in time

	try:
		frame_yuv = np.frombuffer(cameraProcess.stdout.read(w*h*3//2), np.uint8).reshape(h*3//2,w)
	except:
		pass

	N_frames += 1
	if N_frames > max_frames: break

end_time = time.time()
cameraProcess.terminate() # stop the camera

elapsed_seconds = end_time-start_time
print("Done! Result: "+str(N_frames/elapsed_seconds)+" fps")

