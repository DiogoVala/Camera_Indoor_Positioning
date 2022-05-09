import cv2
import numpy as np
import glob
CHECKERBOARD = (13,8)

subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = sorted(glob.glob('*.jpg'))
print(images)

for fname in images:
	print("Processing:", fname)
	img = cv2.imread(fname)
	if _img_shape == None:
		_img_shape = img.shape[:2]
	else:
		assert _img_shape == img.shape[:2], "All images must share the same size."
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	# Chess board corners
	ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
	# Image points (after refinin them)
	if ret == True:
		objpoints.append(objp)
		cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
		imgpoints.append(corners)

N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

ret, mtx, dist, rvecs, tvecs = \
	cv2.fisheye.calibrate(
		objpoints,
		imgpoints,
		gray.shape[::-1],
		K,
		D,
		rvecs,
		tvecs,
		calibration_flags,
		(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
)
np.savez('calib.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
	
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")
DIM=_img_shape[::-1]
balance=1
dim2=None
dim3=None


img = cv2.imread(images[0])
dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
print(dim1)
assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
if not dim2:
	dim2 = dim1
if not dim3:
	dim3 = dim1
scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
	# This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
#cv2.imshow("undist", cv2.resize(undistorted_img, None, fx=0.5, fy=0.5))
#key=cv2.waitKey(3555553)

# Print the camera calibration error
'''
error = 0
for i in range(len(objpoints)):
    imgPoints, _ = cv2.fisheye.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    print(cv2.norm(imgpoints[i], imgPoints, cv2.NORM_L2) / len(imgPoints))
    error += cv2.norm(imgpoints[i], imgPoints, cv2.NORM_L2) / len(imgPoints)

print("Total error: ", error / len(objpoints))

'''
