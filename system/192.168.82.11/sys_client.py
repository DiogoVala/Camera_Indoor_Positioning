import io
import time
import threading
#import picamera
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
import subprocess as sp
import atexit

# Add common modules path
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/common')
from sys_connection import *
import image_processor as imgp
import blob_detector as blob
import sys_calibration_bare as cal

# Camera Settings
w = 2016
h = 1520
fps = 1

# Returns (x,y) real world coordinates at height z.
def getWorldCoordsAtZ(image_point, z, mtx, rmat, tvec):

	camMat = np.asarray(mtx)
	iRot = inv(rmat.T)
	iCam = inv(camMat)

	uvPoint = np.ones((3, 1))

	# Image point
	uvPoint[0, 0] = image_point[0]
	uvPoint[1, 0] = image_point[1]

	tempMat = np.matmul(np.matmul(iRot, iCam), uvPoint)
	tempMat2 = np.matmul(iRot, tvec)

	s = (z + tempMat2[2, 0]) / tempMat[2, 0]
	wcPoint = np.matmul(iRot, (np.matmul(s * iCam, uvPoint) - tvec))

	# wcPoint[2] will not be exactly equal to z, but very close to it
	assert int(abs(wcPoint[2] - z) * (10 ** 8)) == 0
	wcPoint[2] = z

	return wcPoint

# Processing pipeline for each frame
def frame_processor(frameID, frame):
	frame = frame.reshape(h*3//2,w) # Reshape frame into planar YUV420
	frame = cv2.cvtColor(frame, cv2.COLOR_YUV420p2BGR) # Convert to RGB
	frame = cv2.cvtColor(frame, cv2.COLOR_RGB2YUV) # Convert back to YUV
	
	# Converting twice is faster than manually building the YUV frame from the planar frame	
	keypoints = [] # List of detected keypoints in the frame
	keypoints_sizes = []
	keypoint = None
	posData = None 

	# Resize high resolution to low resolution
	frame_low = cv2.resize(frame, (w//blob.rescale_factor,h//blob.rescale_factor),interpolation = cv2.INTER_NEAREST)

	# Filter low resolution frame by YUV components
	mask_low = cv2.inRange(frame_low, blob.lower_range, blob.upper_range)
	
	# Blob detector using low resolution parameters
	keypoints_low = blob.detectBlob_LowRes(mask_low)
	
	# Get rough LED position from low resolution mask
	if keypoints_low:
		pts_rough = [keypoint.pt for keypoint in keypoints_low] # List of keypoint coordinates in low resolution
		pts_rough = [(round(x,0)*blob.rescale_factor, round(y,0)*blob.rescale_factor) for x,y in pts_rough] # Rescale rough coordinates
		
		for pt in pts_rough:
			# Round rough coordinates to the nearest integers
			pt_x=int(round(pt[0],0))
			pt_y=int(round(pt[1],0))
			
			keypoints_tmp=[]
			# Crop frame around each estimated position
			try: # Cropping near the edges of the frame was not tested
				yuv_crop = frame[(pt_y-blob.crop_window):(pt_y+blob.crop_window), (pt_x-blob.crop_window):(pt_x+blob.crop_window)]
				mask_high_crop = cv2.inRange(yuv_crop, blob.lower_range, blob.upper_range)
				
				name=str(time.time())+".jpg"
				#cv2.imwrite(name, mask_high_crop)
				cv2.imshow("frame", mask_high_crop)
				cv2.waitKey(1)
				
				# Blob detector using high resolution parameters to get accurate keypoints for each window
				keypoints_tmp = blob.detectBlob_HighRes(mask_high_crop)
			except Exception as e:
				#print(e)
				break

			for keypoint_tmp in keypoints_tmp:
				# Adjust keypoint coordinates according to the crop window's position within the frame
				x = keypoint_tmp.pt[0]+pt_x-blob.crop_window
				y = keypoint_tmp.pt[1]+pt_y-blob.crop_window
				keypoints.append(tuple((x,y)))
				# Save the keypoint's size
				keypoints_sizes.append(cv2.countNonZero(mask_high_crop)) # Number of white pixels
	
	# Process existing keypoints	
	if keypoints:
		# Select the largest blob
		keypoint = keypoints[np.argmin(keypoints_sizes)] # Argmin because we have the number of empty pixels 
		keypoint = np.array(keypoint, dtype=np.float32).reshape(1,1,2) # Reshape to make it suitable for undistortPoints

		# Correct for camera distortion  
		keypoint = cv2.fisheye.undistortPoints(keypoint, cameraMatrix, cameraDistortion, None, cameraMatrix)
		keypoint = keypoint[0][0] 
		
		# Get projection coordinates in the real world
		keypoint_realWorld = getWorldCoordsAtZ(keypoint, 0.0, cameraMatrix, rmat, tvec).tolist()
		
		# Final data for this frame 
		posData=[(keypoint_realWorld[0][0], keypoint_realWorld[1][0]), (camera_pos[0][0],camera_pos[1][0],camera_pos[2][0])]
	
	# Send location data to the server
	socket_clt.txdata=(frameID,posData)
	socket_clt.event.set()

	return

####### MAIN ####### 
print("Starting client camera.")

# Initialize Socket Server
socket_clt = Socket_Client()

# Run system calibration before starting camera (Must be done before creating a PiCamera instance)
numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec = cal.runCalibration()
if(numDetectedMarkers < cal.MinMarkerCount):
	print("Exiting program.")
	quit()

# Start raspividyuv subprocess to capture frames
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview -ex sports -ISO 100"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=1)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly

# Initialize pool of threads to process each frame
imgp.ImgProcessorPool = [imgp.ImageProcessor(frame_processor) for i in range(imgp.nProcess)]

cameraProcess.stdout.flush() # Flush whatever was sent by the subprocess in order to get a clean start
while True:
	#print("Threads in use: ", (imgp.nProcess-len(imgp.ImgProcessorPool)))
	try:
		frame = np.frombuffer(cameraProcess.stdout.read(w*h*3//2), np.uint8)
	except:
		break
	if frame is not None:
		frameID=time.time()
		processor = imgp.ImgProcessorPool.pop()
		processor.frameID = frameID
		processor.frame = frame
		processor.event.set()
	else:
		break

cameraProcess.terminate() # stop the camera
while imgp.ImgProcessorPool :
	with imgp.ImgProcessorLock:
		processor = imgp.ImgProcessorPool.pop()
	processor.terminated = True
	processor.join()
socket_clt.join()
print("Terminating program.")

