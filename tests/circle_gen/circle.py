import numpy as np
import cv2
from PIL import Image

frame_size = (60,60)
circle_center = (30,30)
circle_radius = 20
frame_resize_scale = 15
frame = np.ones(frame_size)
img_num=0

def MousePoint(event, x, y, flags, params):
	global frame
	if event == cv2.EVENT_LBUTTONDOWN:
		x=int(x/frame_resize_scale)
		y=int(y/frame_resize_scale)
		print(x, y)
		if(frame[y, x]==1.0):
			frame[y, x]=0.0
		else:
			frame[y, x]=1.0
		cv2.imshow("frame", cv2.resize(frame, (0,0), fx=frame_resize_scale, fy=frame_resize_scale, interpolation = cv2.INTER_NEAREST))

while True:
	cv2.circle(frame, circle_center, circle_radius, (0,255,0), -1)
	cv2.imshow("frame", cv2.resize(frame, (0,0), fx=frame_resize_scale, fy=frame_resize_scale, interpolation = cv2.INTER_NEAREST))
	cv2.setMouseCallback("frame", MousePoint)

	key=cv2.waitKey(0)
	if key == ord('q'):
		exit()
	if key == ord('s'):
		cv2.imwrite("%02d.jpg"%(img_num), frame*255.0)
		img_num+=1
