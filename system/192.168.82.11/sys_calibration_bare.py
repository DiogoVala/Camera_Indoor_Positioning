import numpy as np
import cv2
import cv2.aruco as aruco
import picamera
from picamera.array import PiRGBArray
import time
from scipy.spatial.transform import Rotation

# Camera Settings
RESOLUTION = (2016, 1520)
camera = picamera.PiCamera()
camera.resolution 	 = RESOLUTION
camera.exposure_mode = 'night'
camera.iso 			 = 1600

# Camera matrix and cameraDistortion vector
fname = "camera_intrinsics_%dx%d.npz" % (RESOLUTION[0], RESOLUTION[1])
calib_file = np.load(fname)
cameraMatrix=calib_file['mtx']
cameraDistortion=calib_file['dist']

#ArUco Settings
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
parameters =  aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

# Real world position of corners (in millimeters)
# Marker IDs (top to bottom) = 0, 1, 2 and 3
################  	  Corner 0 	 #####     Corner 1   #####     Corner 2 #######    Corner 3 #####
objp=np.array([[   0.0,    0.0, 0.0], [ 173.0,    0.0, 0.0], [ 173.0,  172.5, 0.0], [   0.0,  172.5,  0],\
			   [1832.0,    0.0, 0.0], [2005.0,    0.0, 0.0], [2005.0,  172.5, 0.0], [1832.0,  172.5,  0],\
			   [   0.0, 1475.5, 0.0], [ 173.0, 1475.5, 0.0], [ 173.0, 1648.5, 0.0], [   0.0, 1648.5,  0],\
			   [1832.0, 1475.5, 0.0], [2005.0, 1475.5, 0.0], [2005.0, 1648.5, 0.0], [1832.0, 1648.5,  0]],\
			   dtype = np.float32)

def flattenList(list):
	return [item for sublist in list for item in sublist]

def detectArucos(frame):
	corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

	return len(corners), corners, ids

def undistortFrame(frame):
	
	# Get undistortion maps
	mapx, mapy = cv2.fisheye.initUndistortRectifyMap(cameraMatrix, cameraDistortion, np.eye(3), cameraMatrix, RESOLUTION, cv2.CV_16SC2)
	
	# Remaps the frame pixels to their new positions 
	frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	return frame

def organizeObjpp(markers, ids):
	# Flatten lists to make them more maneageable 
	markers = flattenList(markers)
	ids = flattenList(ids)

	# Create a list of found marker corners.
	imgPts=[]
	for m_id in sorted(ids):
		
		idx = ids.index(m_id)
		marker = markers[idx]
		
		corners=[]
		for i, corner in enumerate(marker):
			x=int(corner[0])
			y=int(corner[1])
			corners.append([x, y])
			
		imgPts.append(corners) # Ordered list of corners
			
	
	# List of pixel coordinate for each found marker corner 
	imgPts=np.array(flattenList(imgPts), dtype = np.float32)
	
	# Get corresponding objp of the detected imgPts
	objpp=[]
	for m_id in sorted(ids):
		ii=m_id*4
		for i in range(4):
			objpp.append(list(objp[ii]))
			ii+=1
	objpp=np.array(objpp)

	return objpp, imgPts

def runCalibration():
	tic = time.perf_counter()
	
	camera_pos = None
	camera_ori = None
	
	# Variable to store frame
	capture = PiRGBArray(camera, size=RESOLUTION)

	# Capture frame from PiCamera
	camera.capture(capture, 'rgb')
	frame = capture.array
	capture.truncate(0)
	
	# ArUco detection is faster in grayscale
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Undistort frame
	frame = undistortFrame(frame)
	
	# Look for ArUco markers
	numDetectedMarkers, markers, ids = detectArucos(frame)
	
	if numDetectedMarkers: # Number of detected ArUco Markers
		
		# Sort detected corners to match reference data
		objpp, imgPts = organizeObjpp(markers, ids)
		
		# Find the rotation and translation vectors
		ret, rvecs, tvecs = cv2.solvePnP(objpp, imgPts, cameraMatrix, np.zeros((1,5)))
		
		tvec = np.array(tvecs) # Translation Vector
		rmat,_= cv2.Rodrigues(rvecs) # Rotation matrix
		rmat = np.array(rmat)
		rmat = rmat.T # Transpose Rotation matrix because we want the reference to be the world plane and not the camera
		R = Rotation.from_matrix(rmat)
		
		# Camera pose calculation 
		camera_pos = -rmat @ tvec
		camera_ori= R.as_euler('xyz', degrees=True)
		
		toc = time.perf_counter()
		
		print("Camera pos:\nx: %dmm\ny: %dmm\nz: %dmm" % (camera_pos[0], camera_pos[1], camera_pos[2]))
		#print("Camera ori:\nx: %.2fº\ny: %.2fº\nz: %.2fº" % (camera_ori[0], camera_ori[1], camera_ori[2]))
		
		print("Calibration Complete.")
		print(f"Calibration time: {toc - tic:0.4f} seconds")
		print("Number of markers detected:", numDetectedMarkers)
		
		camera.close()
		return numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec
	else: 
		print("Calibration Failed.")
		print("Number of markers detected:", numDetectedMarkers)
		camera.close()
		return numDetectedMarkers, None, None, cameraMatrix, cameraDistortion, None, None
		
