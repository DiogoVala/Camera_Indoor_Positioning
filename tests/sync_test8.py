
import cv2
import numpy as np
import subprocess as sp
import time
import atexit

frames = [] # stores the video sequence for the demo
max_frames = 100

N_frames = 0

# Video capture parameters
(w,h) = (2016, 1520)

videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate 250 --nopreview -ex night"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string

cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=1)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly

cameraProcess.stdout.flush() 
while True:
    frame_yuv = np.frombuffer(cameraProcess.stdout.read(w*h*3//2), np.uint8).reshape(h*3//2,w)
    
cameraProcess.terminate() # stop the camera

