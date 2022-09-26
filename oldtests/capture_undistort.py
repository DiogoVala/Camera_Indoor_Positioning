import numpy as np
import cv2
import re
from PIL import Image
from cv2 import aruco
import time

# GPIO stuff
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)
#GPIO.output(2, GPIO.HIGH)

# CV2 Text settings
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.7
color = (0, 0, 255)
thickness = 1

# Camera matrix and distortion vector
calib_file = np.load("calib.npz")
mtx=calib_file['mtx']
dist=calib_file['dist']

# Video capture settings
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 2016)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1520)

time.sleep(0.1)

def flattenList(list):
	return [item for sublist in list for item in sublist]
	
while True:
	
	ret,frame = cam.read()
	
	if frame is not None:
		
		#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		h, w = frame.shape[:2]
		# Obtain the new camera matrix and undistort the image
		newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
		undistortedFrame = cv2.undistort(frame, mtx, dist, None, newCameraMtx)
		
		x, y, w, h = roi
		undistortedFrame = undistortedFrame[y:y + h, x:x + w]
		undistortedFrame = cv2.resize(undistortedFrame, (2016,1520), interpolation = cv2.INTER_LANCZOS4)

		frame = undistortedFrame
		
		# Aruco detection
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
		parameters =  aruco.DetectorParameters_create()
		markers, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
		frame_markers = aruco.drawDetectedMarkers(frame.copy(), markers, ids)

		rvecs, tvecs = aruco.estimatePoseSingleMarkers(markers, 39, newCameraMtx, dist)
		print(rvecs)
		print(tvecs)
		'''
		tvec = np.array(tvecs)
		rmat,_= cv2.Rodrigues(rvecs)
		rmat = np.array(rmat)
		rmat = rmat.T
		camera_pose = -rmat @ tvecs

		print("Camera pose:\nx: %dmm\ny: %dmm\nz: %dmm" % (camera_pose[0], camera_pose[1], camera_pose[2]))
		'''	

		x_marker_center=[]
		y_marker_center=[]
		
		if(len(markers) !=0 and len(ids) != 0):
			markers = flattenList(markers)
			ids = flattenList(ids)
			
			for m_id in ids:
				
				idx = ids.index(m_id)
				
				marker = markers[idx]

				for i, corner in enumerate(marker):
					x=int(corner[0])
					y=int(corner[1])
					frame = cv2.line(frame, (x-10, y), (x+10, y),(0,0,255), 2)
					frame = cv2.line(frame, (x, y-10), (x, y+10),(0,0,255), 2)
					# Draw the text
					cv2.putText(frame, str(i), (x,y), font, fontScale, color, thickness, cv2.LINE_AA)
					

			if(len(x_marker_center)!=0):       
				img_center=(int(np.mean(x_marker_center)), int(np.mean(y_marker_center)))
				#frame = cv2.circle(frame, img_center, 1,(0,0,255), 3)
		
		cv2.imshow('Aruco detection with camera calibration',frame)
		#cv2.imshow('Aruco detection with camera calibration',frame_markers)
		
	
		#cv2.imshow('Calibration test', np.hstack((frame, undistortedFrame)))
		
	key=cv2.waitKey(33)
	if key == ord('q'):
		GPIO.output(2, GPIO.LOW)
		GPIO.cleanup()
		break
	if key == ord('l'):
		GPIO.output(2, GPIO.HIGH)
	if key == ord('o'):
		GPIO.output(2, GPIO.LOW)	

cam.release()
cv2.destroyAllWindows()
