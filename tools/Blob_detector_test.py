import cv2
import numpy as np
import datetime

RESOLUTION = (2016, 1520)
rescale_factor=8
crop_window = 200

frame = cv2.imread("image_with_blobs.jpg")

# Blob detector (High Resolution)
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 60
params.maxArea = 6000
params.minDistBetweenBlobs = 80
params.filterByCircularity = True
params.minCircularity = 0.3
params.filterByConvexity = True
params.minConvexity = 0
params.filterByInertia = True
params.minInertiaRatio = 0.1
detector_h = cv2.SimpleBlobDetector_create(params)

# Blob detector (Rescaled Resolution)
params_low = cv2.SimpleBlobDetector_Params()
params_low.filterByArea = True
params_low.minArea = int(params.minArea/rescale_factor)
params_low.maxArea = int(params.maxArea*rescale_factor)
params_low.minDistBetweenBlobs = int(params.minDistBetweenBlobs/rescale_factor)
params_low.filterByCircularity = params.filterByCircularity
params_low.minCircularity = params.minCircularity
params_low.filterByConvexity = params.filterByConvexity
params_low.minConvexity = params.minConvexity
params_low.filterByInertia = params.filterByInertia
params_low.minInertiaRatio = params.minInertiaRatio
detector_l = cv2.SimpleBlobDetector_create(params_low)

# Resize high resolution to low resolution
tic = datetime.datetime.now()

frame_low = cv2.resize(frame, (int(RESOLUTION[0]/rescale_factor),int(RESOLUTION[1]/rescale_factor)),interpolation = cv2.INTER_NEAREST) 
print("cv2.resize (s):",(datetime.datetime.now()-tic).microseconds/1000000)

mask = frame_low
cv2.imshow("Mask Low Resolution", mask)

# Blob detector
tic = datetime.datetime.now()
keypoints_low = detector_l.detect(mask)
print("Detect Low(s):",(datetime.datetime.now()-tic).microseconds/1000000)

leds_refined=[]

if keypoints_low:
	leds_rough = [keypoint.pt for keypoint in keypoints_low]
	leds_rough = [(int(x)*rescale_factor, int(y)*rescale_factor) for x,y in leds_rough]

for led in leds_rough:
	x=int(led[0])
	y=int(led[1])
	frame_crop = frame[(y-crop_window):(y+crop_window), (x-crop_window):(x+crop_window)]

	keypoints_high = detector_h.detect(frame_crop)
	
	for keypoint_high in keypoints_high:
		led_refined = keypoint_high.pt
		led_refined = (round(led_refined[0])+x-crop_window, round(led_refined[1])+y-crop_window)
		leds_refined.append(led_refined)


print(leds_refined)
if leds_refined:
	for led in leds_refined:
		x=int(led[0])
		y=int(led[1])
		frame = cv2.line(frame, (x-30, y), (x+30, y),(0,0,255), 5)
		frame = cv2.line(frame, (x, y-30), (x, y+30),(0,0,255), 5)

	
cv2.imshow("Keypoints", cv2.resize(frame,None,fx=0.3,fy=0.3))

cv2.waitKey(0)
