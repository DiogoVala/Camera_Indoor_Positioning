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

frames = [] # stores the video sequence for the demo
max_frames = 100000

N_frames = 0

# Video capture parameters
(w,h) = (1280, 960)
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
    # Parse the raw stream into a numpy array
    # YUV: w'h' intensities(Y) + (w'/2)(h'/2) U colour values + (w'/2)*(h'/2) V colour values
    frame_yuv = np.frombuffer(cameraProcess.stdout.read(w*h*3//2), np.uint8).reshape(h*3//2,w)

    #convert to rgb
    frame_rgb = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV420p2RGB);
    
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    
    markers, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame_rgb.copy(), markers, ids)

    #cv2.imshow('webcam',frame_markers)
    # Convert YUV to BGR
    #bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420);

    #frame1.shape = (h,w) # set the correct dimensions for the numpy array
    #rgb = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2RGB);
    
    cv2.imshow("rgb", cv2.resize(frame_markers, (0,0), fx=1, fy=1))
    
    key=cv2.waitKey(1) # request maximum refresh rate
    if key == ord('q'):
        break

    # The frame can be processed here using any function in the OpenCV library.

    # Full image processing will slow down the pipeline, so the requested FPS should be set accordingly.
    #frame = cv2.Canny(frame, 50,150)
    # For instance, in this example you can enable the Canny edge function above.
    # You will see that the frame rate drops to ~35fps and video playback is erratic.
    # If you then set fps = 30 at the beginning of the script, there will be enough cycle time between frames to provide accurate video.
    
    # One optimization could be to work with a decimated (downscaled) version of the image: deci = frame[::2, ::2]
    
    #frames.append(rgb) # save the frame (for the demo)
    #del frame # free the allocated memory
    N_frames += 1
    if N_frames > max_frames: break

end_time = time.time()
cameraProcess.terminate() # stop the camera


elapsed_seconds = end_time-start_time
print("Done! Result: "+str(N_frames/elapsed_seconds)+" fps")

'''
print("Writing frames to disk...")
#out = cv2.VideoWriter("slow_motion.avi", cv2.cv.CV_FOURCC(*"MJPG"), 30, (w,h))
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
out = cv2.VideoWriter("slow_motion.avi", fourcc, 30, (w,h))
for n in range(N_frames):
    #cv2.imwrite("frame"+str(n)+".png", frames[n]) # save frame as a PNG image
    frame_rgb = cv2.cvtColor(frames[n],cv2.COLOR_GRAY2RGB) # video codec requires RGB image
    out.write(frame_rgb)
out.release()

print("Display frames with OpenCV...")
for frame in frames:
    cv2.imshow("Slow Motion", frame)
    cv2.waitKey(1) # request maximum refresh rate


'''
cv2.destroyAllWindows()
