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
from sys_calibration_bare import *

# Add common modules path
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/common')
from sys_connection import *
import image_processor as imgp
import blob_detector as blob

# Camera Settings
camera_resolution = (2016, 1520)

# LED position data from this camera
this_cam_data=None

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
def frame_processor(frame):
	global this_cam_data

	keypoints = [] # List of detected keypoints in the frame
	keypoints_sizes = []
	keypoint = None # Target LED keypoint

	# Resize high resolution to low resolution
	frame_low = cv2.resize(frame, (int(RESOLUTION[0]/blob.rescale_factor),int(RESOLUTION[1]/blob.rescale_factor)),interpolation = cv2.INTER_NEAREST)

	# Filter low resolution frame by color
	mask_low = cv2.inRange(frame_low, blob.lower_range, blob.upper_range)

	# Blob detector
	keypoints_low = blob.detectBlob_LowRes(mask_low)
	
	# Get rough LED position from low resolution mask
	if keypoints_low:
		pts_rough = [keypoint.pt for keypoint in keypoints_low] # List of keypoint coordinates in low resolution
		pts_rough = [(int(x)*blob.rescale_factor, int(y)*blob.rescale_factor) for x,y in pts_rough] # Rescale coordinates

		for pt in pts_rough:
			pt_x=int(pt[0])
			pt_y=int(pt[1])
			# Crop frame around each estimated position
			yuv_crop = frame[(pt_y-blob.crop_window):(pt_y+blob.crop_window), (pt_x-blob.crop_window):(pt_x+blob.crop_window)]
			try:
				mask_high = cv2.inRange(yuv_crop, blob.lower_range, blob.upper_range)
			except:
				break

			# Detect blobs in each cropped region
			keypoints_tmp = blob.detectBlob_HighRes(mask_high)
			
			for keypoint_tmp in keypoints_tmp:
				# Adjust keypoint coordinates according to the crop window position
				x = keypoint_tmp.pt[0]+pt_x-blob.crop_window
				y = keypoint_tmp.pt[1]+pt_y-blob.crop_window
				keypoints.append(tuple((x,y)))
				keypoints_sizes.append(cv2.countNonZero(mask_high)) # Number of white pixels
				
	
	if keypoints:
		# Select the largest blob
		keypoint = keypoints[np.argmin(keypoints_sizes)] # Argmin because we have the number of empty pixels 
		keypoint = np.array(keypoint, dtype=np.float32).reshape(1,1,2)

		# Correct for camera distortion  
		keypoint = cv2.fisheye.undistortPoints(keypoint, cameraMatrix, cameraDistortion, None, cameraMatrix)
		
		keypoint = keypoint[0][0]
		
		# Get projection coordinates in the real world
		keypoint_realWorld = getWorldCoordsAtZ(keypoint, 0, cameraMatrix, rmat, tvec).tolist()

		this_cam_data=[(keypoint_realWorld[0][0], keypoint_realWorld[1][0]), (camera_pos[0][0],camera_pos[1][0],camera_pos[2][0])]
		#print(time.time(), this_cam_data)
	# Send location data to the server
	socket_clt.txdata=this_cam_data
	socket_clt.event.set()

	return

####### MAIN ####### 
print("Starting client camera.")

# Initialize Socket Server
socket_clt = Socket_Client()

# Run system calibration before starting camera (Must be done before creating a PiCamera instance)
numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec = runCalibration()
if(numDetectedMarkers == 0):
	print("Exiting program.")
	quit()

# Camera startup
camera = picamera.PiCamera()
camera.resolution = camera_resolution
camera.exposure_mode = 'sports'
camera.iso 	= 100
print("Camera warming up.")
time.sleep(1)

# Initialize pool of threads to process each frame
imgp.ImgProcessorPool = [imgp.ImageProcessor(frame_processor, camera, camera_resolution) for i in range(imgp.nProcess)]

print("Starting capture.")
camera.capture_sequence(imgp.getStream(), use_video_port=True, format='yuv')

while imgp.ImgProcessorPool :
	with imgp.ImgProcessorLock:
		processor = imgp.ImgProcessorPool.pop()
	processor.terminated = True
	processor.join()
socket_clt.join()
print("Terminating program.")

