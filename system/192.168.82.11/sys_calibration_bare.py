import numpy as np
import cv2
import cv2.aruco as aruco
import picamera
from picamera.array import PiRGBArray
import time
from scipy.spatial.transform import Rotation
import subprocess as sp

# Calibration Settings
MinMarkerCount = 4

# Camera Settings
RESOLUTION = (2016, 1520)

# Camera matrix and cameraDistortion vector
fname = "camera_intrinsics_%dx%d.npz" % (RESOLUTION[0], RESOLUTION[1])
calib_file = np.load(fname)
cameraMatrix=calib_file['mtx']
cameraDistortion=calib_file['dist']

# ArUco Settings
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
parameters =  aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

# Real world position of corners (in millimeters)
# Marker IDs (top to bottom) = 0, 1, 2, 3, etc.
################  	  Corner 0 	 #####     Corner 1   #####     Corner 2 #######    Corner 3 #####
objp=np.array([[ 125, 125, 0.0], [ 275, 125, 0.0], [ 275, 275, 0.0], [ 125, 275, 0.0],\
			   [2025, 125, 0.0], [2175, 125, 0.0], [2175, 275, 0.0], [2025, 275, 0.0],\
			   [ 125,1125, 0.0], [ 275,1125, 0.0], [ 275,1275, 0.0], [ 125,1275, 0.0],\
			   [2025,1125, 0.0], [2175,1125, 0.0], [2175,1275, 0.0], [2025,1275, 0.0],\
			   [ 125,2521, 0.0], [ 275,2521, 0.0], [ 275,2671, 0.0], [ 125,2671, 0.0],\
			   [2025,2521, 0.0], [2175,2521, 0.0], [2175,2671, 0.0], [2025,2671, 0.0],\
			   [ 125,3518, 0.0], [ 275,3518, 0.0], [ 275,3668, 0.0], [ 125,3668, 0.0],\
			   [2025,3518, 0.0], [2175,3518, 0.0], [2175,3668, 0.0], [2025,3668, 0.0],\
			   [1075, 625, 0.0], [1225, 625, 0.0], [1225, 775, 0.0], [1075, 775, 0.0],\
			   [1075,1820, 0.0], [1225,1820, 0.0], [1225,1970, 0.0], [1075,1970, 0.0],\
			   [1075,3018, 0.0], [1225,3018, 0.0], [1225,3168, 0.0], [1075,3168, 0.0]],\
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
	
	# Start subprocess to capture still
	videoCmd = "raspistill -o ~/Camera_Indoor_Positioning/system/192.168.82.11/cal.bmp -w 2016 -h 1520 -ex night -ISO 1600"
	sp.call(videoCmd, shell=True)
	
	# Read captured image
	frame=cv2.imread("/home/pi/Camera_Indoor_Positioning/system/192.168.82.11/cal.bmp")
	
	# Show calibration image
	#cv2.imshow("Calibration", cv2.resize(frame, (0,0), fx=0.5, fy=0.5))
	
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

		print("Calibration Complete.")
		print(f"Calibration time: {toc - tic:0.4f} seconds")
		print("Number of markers detected:", numDetectedMarkers)
		return numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec
	else: 
		print("Calibration Failed.")
		print("Number of markers detected:", numDetectedMarkers)
		return numDetectedMarkers, None, None, cameraMatrix, cameraDistortion, None, None
		
