import cv2
import blob_detector as blob
import numpy as np
import timeit

# Camera Settings
w = 4032
h = 3040
fps = 1

frame_original = cv2.imread("test_image.jpg")


def frame_processor_basic(frame_original):
	frame = cv2.cvtColor(frame_original, cv2.COLOR_RGB2YUV)
	mask = 255-cv2.inRange(frame, blob.lower_range, blob.upper_range)
	keypoints = blob.detectBlob_HighRes(mask)


def frame_processor(frame_original):
	frame = cv2.cvtColor(frame_original, cv2.COLOR_RGB2YUV)
	#cv2.imshow("frame", cv2.resize(frame, None, fx=0.2, fy=0.2))
	 # Convert back to YUV
	# Converting twice is faster than manually building the YUV frame from the planar frame	
	#cv2.imshow("frame2", cv2.resize(frame, None, fx=0.2, fy=0.2))
	
	keypoints = [] # List of detected keypoints in the frame
	keypoints_sizes = []
	keypoint = None

	# Resize high resolution to low resolution
	frame_low = cv2.resize(frame, (w//blob.rescale_factor,h//blob.rescale_factor),interpolation = cv2.INTER_NEAREST)

	# Filter low resolution frame by YUV components
	mask_low = 255-cv2.inRange(frame_low, blob.lower_range, blob.upper_range)
	
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
				mask_high_crop = 255-cv2.inRange(yuv_crop, blob.lower_range, blob.upper_range)
				
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


## Test at max resolution

start = timeit.default_timer()
for i in range(10):
	frame_processor_basic(frame_original)
stop = timeit.default_timer()
print("No Pipeline, 4032x3040", (stop-start)/10)

start = timeit.default_timer()
for i in range(10):
	frame_processor(frame_original)
	
stop = timeit.default_timer()
print("Pipeline, 4032x3040", (stop-start)/10)

## Test at half resolution

frame_original = cv2.resize(cv2.imread("test_image.jpg"), None, fx=0.5, fy=0.5)

start = timeit.default_timer()
for i in range(10):
	frame_processor_basic(frame_original)
stop = timeit.default_timer()
print("No Pipeline, 2016x1520", (stop-start)/10)

start = timeit.default_timer()
for i in range(10):
	frame_processor(frame_original)
	
stop = timeit.default_timer()
print("Pipeline, 2016x1520", (stop-start)/10)