import io
import time
import threading
import picamera
import datetime
import math
import cv2
import numpy as np
from numpy.linalg import inv
from numpy import array, cross
from numpy.linalg import solve, norm
from scipy.spatial.transform import Rotation
from sys_calibration_bare import *
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

	return

# Calculates closest approach of two lines
def intersect(other_cam_data):
	try:
		P0=np.array([[this_cam_data[1][0], this_cam_data[1][1], this_cam_data[1][2]], [other_cam_data[1][0], other_cam_data[1][1], other_cam_data[1][2]]])
		P1=np.array([[this_cam_data[0][0], this_cam_data[0][1], 0.0], [other_cam_data[0][0], other_cam_data[0][1], 0.0]])
		
		
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
		#d = solution[1]
		
		print(this_cam_data)
		print(f"LED at (%.2f, %.2f, %.2f)" % (round(p[0][0],2), round(p[1][0],2), round(p[2][0],2)) )
		#print(f"Distance between lines at closest approach: %.f" % (d) )
		return p,d
	except:
		print("Invalid data")
		return None


####### MAIN ####### 
print("Starting server camera.")

# Initialize Socket Server
socket_sv = Socket_Server(intersect)

# Run system calibration before starting camera (Must be done before creating a PiCamera instance)
numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec = runCalibration()
if(numDetectedMarkers == 0):
	print("Exiting program.")
	quit()

# Camera startup
camera = picamera.PiCamera()
camera.resolution = camera_resolution
camera.exposure_mode = 'sports'
camera.iso 	= 150
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
socket_sv.join()
print("Terminating program.")
