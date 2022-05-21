import numpy as np
import cv2
import cv2.aruco as aruco
import picamera
from picamera.array import PiRGBArray
import time
import math
from scipy.spatial.transform import Rotation
from numpy.linalg import inv
import csv

# GPIO for light
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)

# CV2 Text settings
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 2
color = (0, 0, 255)
color1 = (255, 0, 0)
thickness = 3

# Camera Settings
RESOLUTION = (2016, 1520)
#RESOLUTION = (4032, 3040)
rescale_factor=4
crop_window = 100

# Blob detector (High Resolution)
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 10
params.maxArea = 6000
params.minDistBetweenBlobs = 80
params.filterByCircularity = True
params.minCircularity = 0
params.filterByConvexity = True
params.minConvexity = 0
params.filterByInertia = True
params.minInertiaRatio = 0
detector_h = cv2.SimpleBlobDetector_create(params)

# Blob detector (Rescaled Resolution)
params_low = cv2.SimpleBlobDetector_Params()
params_low.filterByArea = True
params_low.minArea = 1
params_low.maxArea = int(params.maxArea*rescale_factor)
params_low.minDistBetweenBlobs = int(params.minDistBetweenBlobs/rescale_factor)
params_low.filterByCircularity = params.filterByCircularity
params_low.minCircularity = params.minCircularity
params_low.filterByConvexity = params.filterByConvexity
params_low.minConvexity = params.minConvexity
params_low.filterByInertia = params.filterByInertia
params_low.minInertiaRatio = params.minInertiaRatio
detector_l = cv2.SimpleBlobDetector_create(params_low)

# Color detection thersholds (YUV)
lower_range = np.array([  0,  0, 76])
upper_range = np.array([203,255,173])

# Camera matrix and distortion vector
calib_file = np.load("camera_intrinsics_2016x1520.npz")
#calib_file = np.load("camera_intrinsics_4032x3040.npz")
mtx=calib_file['mtx']
dist=calib_file['dist']

# Real world position of corners
################  	  Corner 0 	 #####     Corner 1   #####     Corner 2 #######    Corner 3 #####
'''
objp=np.array([[-115.5, -80.0, 0.0], [-79.5, -80.0, 0.0], [-79.5, -44.5, 0.0], [-115.5, -44.5,  0],\
			   [  79.5, -80.0, 0.0], [115.5, -80.0, 0.0], [115.5, -44.5, 0.0], [  79.5, -44.5,  0],\
			   [-115.5,  44.5, 0.0], [-79.5,  44.5, 0.0], [-79.5,  80.0, 0.0], [-115.5,  80.0,  0],\
			   [  79.5,  44.5, 0.0], [115.5,  44.5, 0.0], [115.5,  80.0, 0.0], [  79.5,  80.0,  0]],\
			   dtype = np.float32)
'''
objp=np.array([[   0.0,    0.0, 0.0], [ 173.0,    0.0, 0.0], [ 173.0,  172.5, 0.0], [   0.0,  172.5,  0],\
			   [1832.0,    0.0, 0.0], [2005.0,    0.0, 0.0], [2005.0,  172.5, 0.0], [1832.0,  172.5,  0],\
			   [   0.0, 1475.5, 0.0], [ 173.0, 1475.5, 0.0], [ 173.0, 1648.5, 0.0], [   0.0, 1648.5,  0],\
			   [1832.0, 1475.5, 0.0], [2011.5, 1475.5, 0.0], [2011.5, 1648.5, 0.0], [1832.0, 1648.5,  0]],\
			   dtype = np.float32)

PoIs=[(2028, 1630), (2868, 1636), (1183, 1623), (2034, 1158) ,(2022, 2097)]

PoIs_proj=np.array([[0.0, 0.0, 0.0], [-79.5, 0.0, 0.0], [0.0, -44.5, 0.0], [0.0, 44.5,  0],\
					[79.5, 0.0, 0.0]],dtype = np.float32)

def flattenList(list):
	return [item for sublist in list for item in sublist]

def detectArucos(frame):
	tic = time.perf_counter()
	
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
	parameters =  aruco.DetectorParameters_create()
	parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
	corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
	ret =  True if len(corners)!=0 else False

	#print(f"ArUco detection time: {time.perf_counter() - tic:0.4f} seconds")

	return ret, corners, ids

def undistortFrame(frame, mapx, mapy):
	tic = time.perf_counter()
	
	# Remaps the frame pixels to their new positions 
	frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	
	#print(f"Undistort time: {time.perf_counter() - tic:0.4f} seconds")
	return frame

def drawCorner(frame, corner, i):
	x=int(corner[0])
	y=int(corner[1])
	frame = cv2.line(frame, (x-10, y), (x+10, y),(0,0,255), 1)
	frame = cv2.line(frame, (x, y-10), (x, y+10),(0,0,255), 1)
	# Draw the text
	#cv2.putText(frame, str(i), (x,y), font, fontScale, color, thickness, cv2.LINE_AA)
	
def drawReprojection(frame, corner):
	x=int(corner[0])
	y=int(corner[1])
	frame = cv2.line(frame, (x-30, y), (x+30, y),(0,255,0), 1)
	frame = cv2.line(frame, (x, y-30), (x, y+30),(0,255,0), 1)

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
    
def drawWorldAxis(frame):
	axis=np.array([[0.0, 0.0, 0.0], [0.0, 1800.0, 0.0], [0.0, 0.0, 0.0], [2000.0, 0.0,  0] , [0.0, 0.0, 0.0], [0.0, 0.0, -1000.0]],dtype = np.float32)
	projs, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, None)
	# Y axis
	frame = cv2.arrowedLine(frame, (int(round(projs[0][0][0],0)), int(round(projs[0][0][1], 0))), (int(round(projs[1][0][0],0)), int(round(projs[1][0][1],0))),(0,255,0), 3, tipLength = 0.05)
	# X axis
	frame = cv2.arrowedLine(frame, (int(round(projs[2][0][0],0)), int(round(projs[2][0][1], 0))), (int(round(projs[3][0][0],0)), int(round(projs[3][0][1],0))),(30,30,255), 3, tipLength = 0.05)

def drawRealWorld(x, y, frame):
	frame = cv2.line(frame, (x-30, y), (x+30, y),(255,0,0), 2)
	frame = cv2.line(frame, (x, y-30), (x, y+30),(255,0,0), 2)

	image_point=(x, y)
	world_coords=getWorldCoordsAtZ(image_point, 0.0, mtx, rmat, tvec)
	txt = f"({float(world_coords[0]):0.2f}, {float(world_coords[1]):0.2f})mm"
	cv2.putText(frame, txt, (x,y-10), font, fontScale/2, color1, thickness, cv2.LINE_AA)

def drawRefLine(frame, imgPts):
	
	try:
		x1 = round(imgPts[0][0])
		y1 = round(imgPts[0][1])
		x2 = imgPts[1][0]
		y2 = imgPts[1][1]
		
		a = (1.0*(y2-y1))/(1.0*(x2-x1))
		b = -a*x1+y1
		y2_ = (int)(np.shape(frame)[1])
		x2_ = (int)((y2_-b)/a)
		frame = cv2.line(frame,(x1,y1),(x2_,y2_),(255,255,255),1)


		x2 = 208
		y2 = 1276
		
		a = (1.0*(y2-y1))/(1.0*(x2-x1))
		b = -a*x1+y1
		y1_ = (int)(0)
		y2_ = (int)(np.shape(frame)[1])
		x1_ = (int)((y1_-b)/a)
		x2_ = (int)((y2_-b)/a)
		frame = cv2.line(frame,(x1,y1),(x2,y2),(255,255,255),1)
		
		x1 = 208
		y1 = 1276
		x2 = 307
		y2 = 1274
		
		a = (1.0*(y2-y1))/(1.0*(x2-x1))
		b = -a*x1+y1
		y1_ = (int)(0)
		y2_ = (int)(np.shape(frame)[1])
		x1_ = (int)((y1_-b)/a)
		x2_ = (int)((y2_-b)/a)
		frame = cv2.line(frame,(x1_,y1_),(x2_,y2_),(255,255,255),1)
		
		x1 = 1324
		y1 = 379
		x2 = 1323
		y2 = 470
		
		a = (1.0*(y2-y1))/(1.0*(x2-x1))
		b = -a*x1+y1
		y1_ = (int)(0)
		y2_ = (int)(np.shape(frame)[1])
		x1_ = (int)((y1_-b)/a)
		x2_ = (int)((y2_-b)/a)
		frame = cv2.line(frame,(x1_,y1_),(x2_,y2_),(255,255,255),1)
		
	except:
		pass
		
def organizeObjpp(markers, ids):
	tic = time.perf_counter()
	
	# Flatten lists to make them more maneageable 
	markers = flattenList(markers)
	ids = flattenList(ids)

	# Here we create a list of found marker corners.
	imgPts=[]
	for m_id in sorted(ids):
		
		idx = ids.index(m_id)
		marker = markers[idx]
		
		corners=[]
		for i, corner in enumerate(marker):
			x=int(corner[0])
			y=int(corner[1])
			corners.append([x, y])
			drawCorner(frame, corner, i)
			
		imgPts.append(corners) # Ordered list of corners
			
	
	# List of pixel coordinate for each found marker corner 
	imgPts=np.array(flattenList(imgPts), dtype = np.float32)
	
	# Get corresponding objp of the detected imgPts
	objpp=[]
	for id in sorted(ids):
		ii=id*4
		for i in range(4):
			objpp.append(list(objp[ii]))
			ii+=1
	objpp=np.array(objpp)

	#print(f"Organize Objpp time: {time.perf_counter() - tic:0.4f} seconds")
	
	return objpp, imgPts
	

def detectBlob(frame):
	tic = time.perf_counter()
	
	keypoints = [] # List of detected keypoints in the frame
	keypoints_sizes = []
	keypoint = None # Target LED keypoint

	# Resize high resolution to low resolution
	frame_low = cv2.resize(frame, (int(RESOLUTION[0]/rescale_factor),int(RESOLUTION[1]/rescale_factor)),interpolation = cv2.INTER_NEAREST) 

	# only used in visualizer
	yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
	yuv_low = cv2.cvtColor(frame_low, cv2.COLOR_BGR2YUV)

	# Filter low resolution frame by color
	mask_low = cv2.inRange(yuv_low, lower_range, upper_range)

	# Blob detector (Low resolution)
	keypoints_low = detector_l.detect(mask_low)

	# Get rough keypoint positions from low resolution mask
	if keypoints_low:
		pts_rough = [keypoint.pt for keypoint in keypoints_low] # List of keypoint coordinates in low resolution
		pts_rough = [(int(x)*rescale_factor, int(y)*rescale_factor) for x,y in pts_rough] # Rescale coordinates

		for pt in pts_rough:
			pt_x=int(pt[0])
			pt_y=int(pt[1])
			# Crop frame around each estimated position
			yuv_crop = frame[(pt_y-crop_window):(pt_y+crop_window), (pt_x-crop_window):(pt_x+crop_window)]
			try:
				mask_high = cv2.inRange(yuv_crop, lower_range, upper_range)
			except:
				break

			# Detect blobs in each cropped region
			keypoints_tmp = detector_h.detect(mask_high)
			for keypoint_tmp in keypoints_tmp:
				# Adjust keypoint coordinates according to the crop window position
				x = keypoint_tmp.pt[0]+pt_x-crop_window
				y = keypoint_tmp.pt[1]+pt_y-crop_window
				keypoints.append(tuple((x,y)))
				keypoints_sizes.append(cv2.countNonZero(mask_high)) # Number of white pixels

	if keypoints:
		# Select the largest blob
		keypoint = keypoints[np.argmin(keypoints_sizes)] # Argmin because we have the number of empty pixels 
		
		# Correct for camera distortion  
		# keypoint = cv2.fisheye.undistortPoints(keypoint, cameraMatrix, cameraDistortion, None, cameraMatrix)
		#keypoint_realWorld = getWorldCoordsAtZ(keypoint, 0, mtx, rmat, tvec)

		x=int(keypoint[0])
		y=int(keypoint[1])
		#frame = cv2.line(frame, (x-30, y), (x+30, y),(0,0,255), 2)
		#frame = cv2.line(frame, (x, y-30), (x, y+30),(0,0,255), 2)
	
	#print(f"Blob detection time: {time.perf_counter() - tic:0.4f} seconds")
	return keypoint, frame, mask_low
	

# main
with picamera.PiCamera() as camera:
	
	camera.resolution = RESOLUTION
	camera.exposure_mode = 'sports'
	camera.iso = 1600
	#camera.color_effects = (128, 128)
	capture = PiRGBArray(camera, size=RESOLUTION)
	
	# Get undistortion maps (This allows for a much faster undistortion using cv2.remap)
	#newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
	#mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newCameraMatrix, (w, h), cv2.CV_32FC1)
	
	#mtx_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx, dist, RESOLUTION, np.eye(3), balance=1)
	mapx, mapy = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, RESOLUTION, cv2.CV_16SC2)
	
	N=100000
	n=1
	camera_pos_arr=[]
	while n<N:
		print("")
		
		tic=time.perf_counter()
		camera.capture(capture, 'rgb')
		
		frame = capture.array
		capture.truncate(0)
		
		# Undistort frame
		frame = undistortFrame(frame, mapx, mapy)
		#frame = cv2.fisheye.undistortImage()
		
		yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
		
		# Detect LED blob
		led_coord, frame, mask = detectBlob(frame)

		# Look for ArUco markers
		valid_markers, markers, ids = detectArucos(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
		if valid_markers:
			
			# Organize data obtained from detectedArucos
			objpp, imgPts = organizeObjpp(markers, ids)
			
			#drawRefLine(frame, imgPts)
			
			# Find the rotation and translation vectors.
			ret, rvecs, tvecs = cv2.solvePnP(objpp, imgPts, mtx, np.zeros((1,5)))
			
			# Camera pose calculation 
			tvec = np.array(tvecs)
			rmat,_= cv2.Rodrigues(rvecs)
			rmat = np.array(rmat)
			rmat = rmat.T
			R = Rotation.from_matrix(rmat)
			
			camera_pos = -rmat @ tvec
			camera_ori = R.as_euler('xyz', degrees=True)
			
			# Save position estimates over N iterations (only save if all markers are detected)
			if len(markers) == 4:
				n+=1
				camera_pos_arr.append(camera_pos)
			
			#print("Camera pos:\nx: %dmm\ny: %dmm\nz: %dmm" % (camera_pos[0], camera_pos[1], camera_pos[2]))
			#print("Camera ori:\nx: %.2fº\ny: %.2fº\nz: %.2fº" % (camera_ori[0], camera_ori[1], camera_ori[2]))

			# Reproject ArUco corners
			projs, jac = cv2.projectPoints(objpp, rvecs, tvecs, mtx, None)
			for proj in projs:
				drawReprojection(frame, proj[0]) 
			
			'''
			# Reproject testing points
			projs1, jac1 = cv2.projectPoints(PoIs_proj, rvecs, tvecs, mtx, None)
			for proj in projs1:
				drawReprojection(frame, proj[0])
			for poi in projs1:
				poi=poi[0]
				drawRealWorld(int(round(poi[0],0)), int(round(poi[1],0)), frame)
			'''
			
			# Draw XY axis 
			drawWorldAxis(frame)
			
			# Led Coord at z=0
			try:
				led_realWorld=getWorldCoordsAtZ(led_coord[0], 0, mtx, rmat, tvec)
				txt='('+str(round(led_realWorld[0][0],2))+'; '+str(round(led_realWorld[1][0],2))+')'
				cv2.putText(frame, txt, led_coord[0], font, 1, (0,0,255), 2, cv2.LINE_AA)
			except:
				pass
			
			# Draw information about camera pose
			txt = f"Camera position (xyz): {float(camera_pos[0]):0.2f}, {float(camera_pos[1]):0.2f} , {float(camera_pos[2]):0.2f} mm"
			#cv2.putText(frame, txt, (0,100), font, fontScale, (255,255,255), thickness, cv2.LINE_AA)
			txt = f"Euler angles (xyz): {float(camera_ori[0]):0.2f}, {float(camera_ori[1]):0.2f} , {float(camera_ori[2]):0.2f} deg"
			#cv2.putText(frame, txt, (0,200), font, fontScale, (255,255,255), thickness, cv2.LINE_AA)
			
		
		mask = cv2.inRange(yuv, lower_range, upper_range)
		
		
		toc = time.perf_counter()
		print(f"Calibration time: {toc - tic:0.4f} seconds")
		
		scale=0.7
		cv2.imshow('mask', cv2.resize(mask, None, fx=scale,fy=scale))
		cv2.imshow('Picamera',cv2.resize(frame, None, fx=scale,fy=scale))
		cv2.imwrite("frame.jpg", frame)
		cv2.imwrite("mask.jpg", mask)
		
		key=cv2.waitKey(33)
		if key == ord('q'):
			GPIO.output(2, GPIO.LOW)
			break
		if key == ord('s'): # Turn on the light
			cv2.imwrite("out.jpg", frame)	
		if key == ord('l'): # Turn on the light
			GPIO.output(2, GPIO.HIGH)
		if key == ord('o'): # Turn off the light
			GPIO.output(2, GPIO.LOW)

	fname = "%dx%d.csv" % (RESOLUTION[0], RESOLUTION[1])
	with open(fname, 'w') as f:
		write = csv.writer(f)
		for pos in camera_pos_arr:
			write.writerow(pos)
