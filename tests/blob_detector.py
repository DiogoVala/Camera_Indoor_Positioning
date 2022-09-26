import cv2
import numpy as np

# Color detection thersholds (YUV)
lower_range = np.array([  0,  0, 76])
upper_range = np.array([203,255,173])

rescale_factor=8
crop_window = 30

# Blob detector (High Resolution)
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 1
params.maxArea = 6000
params.minDistBetweenBlobs = 300
params.filterByCircularity = True
params.minCircularity = 0.3
params.filterByConvexity = True
params.minConvexity = 0.2
params.filterByInertia = False
params.minInertiaRatio = 0.1
detector_h = cv2.SimpleBlobDetector_create(params)

# Blob detector (Rescaled Resolution)
params_low = cv2.SimpleBlobDetector_Params()
params_low.filterByArea = True
params_low.minArea = 1
params_low.maxArea = int(params.maxArea*rescale_factor)
params_low.minDistBetweenBlobs = 200
params_low.filterByCircularity = params.filterByCircularity
params_low.minCircularity = params.minCircularity
params_low.filterByConvexity = params.filterByConvexity
params_low.minConvexity = params.minConvexity
params_low.filterByInertia = params.filterByInertia
params_low.minInertiaRatio = params.minInertiaRatio
detector_l = cv2.SimpleBlobDetector_create(params_low)

def detectBlob_HighRes(image):
	# Look for blobs in each cropped region
	keypoints = detector_h.detect(image)
	return keypoints


def detectBlob_LowRes(image):
	# Look for blobs in each cropped region
	keypoints = detector_l.detect(image)
	return keypoints
