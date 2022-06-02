# Camera-based Positioning System - Diogo Vala 2022

import io
import time
import threading
import datetime
import math
import cv2
import numpy as np
import sys
from numpy.linalg import inv
from numpy import array, cross
from numpy.linalg import solve, norm
from scipy.spatial.transform import Rotation
import subprocess as sp
import atexit
import heapq
import os
os.nice(10)

# Add common modules path
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/common')
from sys_connection import *
import image_processor as imgp
import blob_detector as blob
import sys_calibration_bare as cal

# Camera Settings
w = 2016
h = 1520
fps = 5

# Minimum number of ArUco markers required for an acceptable initial calibration
MinMarkerCount = 0

# Piority queues to store calculated data
# Higher priority is given to oldest data
sv_DataQ = list([])
cl_DataQ = list([])
heapq.heapify(sv_DataQ)
heapq.heapify(cl_DataQ)

# Maximum time difference between cameras (Values are discarded if timing is off)
timing_threshhold = 0.1

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
	frame = cv2.cvtColor(frame, cv2.COLOR_YUV420p2RGB) # Convert to RGB
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

			# Crop frame around each estimated position
			try: # Cropping near the edges of the frame was not tested
				yuv_crop = frame[(pt_y-blob.crop_window):(pt_y+blob.crop_window), (pt_x-blob.crop_window):(pt_x+blob.crop_window)]
				mask_high_crop = cv2.inRange(yuv_crop, blob.lower_range, blob.upper_range)
			except Exception as e:
				print(e)
				break

			# Blob detector using high resolution parameters to get accurate keypoints for each window
			keypoints_tmp = blob.detectBlob_HighRes(mask_high_crop)
			
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
		#keypoint_realWorld = getWorldCoordsAtZ(keypoint, 0.0, cameraMatrix, rmat, tvec).tolist()
		
		# Final data for this frame 
		#posData=[(keypoint_realWorld[0][0], keypoint_realWorld[1][0]), (camera_pos[0][0],camera_pos[1][0],camera_pos[2][0])]
	
	# Save data in a heap queue. posData = None is stored if no blob is detected
	heapq.heappush(sv_DataQ,(frameID, posData))
	return

# Calculates closest approach of two lines
prev_time = time.time()
def intersect(svData, clData):
		
	try:
		svCamPos = [svData[1][0], svData[1][1], svData[1][2]] # Server camera position
		svProj   = [svData[0][0], svData[0][1], 0.0]		  # Target projection from server's perspective 
		clCamPos = [clData[1][0], clData[1][1], clData[1][2]] # Client camera position
		clProj   = [clData[0][0], clData[0][1], 0.0]		  # Target projection from client's perspective 
		P0 = np.array([svCamPos, clCamPos])
		P1 = np.array([svProj,   clProj])
	except Exception as e:
		print(e)
		return
		
	"""P0 and P1 are NxD arrays defining N lines.
	D is the dimension of the space. This function 
	returns the least squares intersection of the N
	lines from the system given by eq. 13 in 
	http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf.
	"""
	# generate all line direction vectors 
	n = (P1-P0)/np.linalg.norm(P1-P0,axis=1)[:,np.newaxis] # normalized

	# generate the array of all projectors 
	projs = np.eye(n.shape[1]) - n[:,:,np.newaxis]*n[:,np.newaxis]  # I - n*n.T

	# generate R matrix and q vector
	R = projs.sum(axis=0)
	q = (projs @ P0[:,:,np.newaxis]).sum(axis=0)

	# solve the least squares problem for the 
	# intersection point p: Rp = q
	solution = np.linalg.lstsq(R,q,rcond=None)
	p = solution[0]
	
	try:
		a0=np.array(svProj)
		a1=np.array(svCamPos)
		
		b0=np.array(clProj)
		b1=np.array(clCamData)
		d=closestDistanceBetweenLines(a0,a1,b0,b1)	
	except Exception as e:
		print(e)
		d=-1
		
	print(f"Server at: (%8.2f, %8.2f, %8.2f)mm" % (this_cam_data[1][0], this_cam_data[1][1], this_cam_data[1][2]) )
	print(f"Client at: (%8.2f, %8.2f, %8.2f)mm" % (other_cam_data[1][0], other_cam_data[1][1], other_cam_data[1][2]) )
	print(f"Target at: (%8.2f, %8.2f, %8.2f)mm" % (round(p[0][0],2), round(p[1][0],2), round(p[2][0],2)) )
	print(f"Distance between lines at closest approach: %.fmm" % (d) )
	print("\x1b[5A\r")
	'''
	with open('pos.csv', 'a', newline='') as f:
		writer = csv.writer(f)
		row=[(time.time()-prev_time), round(p[0][0],2), round(p[1][0],2), round(p[2][0],2)]
		writer.writerow(row)
	'''
	return None

def closestDistanceBetweenLines(a0,a1,b0,b1):

	''' Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
		Return the closest points on each segment and their distance
	'''
	# Calculate denomitator
	A = a1 - a0
	B = b1 - b0
	magA = np.linalg.norm(A)
	magB = np.linalg.norm(B)
	
	_A = A / magA
	_B = B / magB
	
	cross = np.cross(_A, _B);
	denom = np.linalg.norm(cross)**2
	
	# Lines criss-cross: Calculate the projected closest points
	t = (b0 - a0);
	detA = np.linalg.det([t, _B, cross])
	detB = np.linalg.det([t, _A, cross])

	t0 = detA/denom;
	t1 = detB/denom;

	pA = a0 + (_A * t0) # Projected closest point on segment A
	pB = b0 + (_B * t1) # Projected closest point on segment B
	
	return np.linalg.norm(pA-pB)
	
####### MAIN ####### 
print("Starting server camera.")

# Initialize Socket Server
socket_sv = Socket_Server(intersect, cl_DataQ)

time.sleep(0.2)

# Run system calibration before starting camera (Must be done before creating a PiCamera instance)
numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec = cal.runCalibration()
if(numDetectedMarkers < MinMarkerCount):
	print("Exiting program.")
	quit()

# Start raspividyuv subprocess to capture frames
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview -ex sports -ISO 150"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=1)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly

# Initialize pool of threads to process each frame
imgp.ImgProcessorPool = [imgp.ImageProcessor(frame_processor) for i in range(imgp.nProcess)]

def test():
	threading.Timer(1/fps, test).start()
	
	global sv_DataQ, cl_DataQ
	
	try:
		svData=heapq.heappop(sv_DataQ)
		clData=heapq.heappop(cl_DataQ)
			
		timedif=svData[0]-clData[0]
		
		if(abs(timedif) < timing_threshhold):
			print(timedif)
			intersect(svData, clData)

	except Exception as e: 
		sv_DataQ = list([])
		cl_DataQ = list([])	
		print(e)
	
		
new_thread = threading.Thread(target=test)
new_thread.start()

time.sleep(0.3) # Give the client some time to reach this point
cameraProcess.stdout.flush() # Flush whatever was sent by the subprocess in order to get a clean start

while True:
	#print("Threads in use: ", (imgp.nProcess-len(imgp.ImgProcessorPool)))
	frame = np.frombuffer(cameraProcess.stdout.read(w*h*3//2), np.uint8)
	if frame is not None:
		try:
			frameID=time.time()
			processor = imgp.ImgProcessorPool.pop()
			processor.frameID = frameID
			processor.frame = frame
			processor.event.set()
		except Exception as e:
			print(e)
			print("Terminating program.")
			cameraProcess.terminate() # stop the camera
			while imgp.ImgProcessorPool :
				processor = imgp.ImgProcessorPool.pop()
				processor.terminated = True
				processor.event.set()
			socket_sv.join()
			break
			
