
import math
import cv2
import sys
import glob

# Add common modules path
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/common')
import blob_detector as blob
from sys_calibration_bare import *
from numpy.linalg import inv
from numpy import array, cross
from numpy.linalg import solve, norm

# Run system calibration before starting camera (Must be done before creating a PiCamera instance)
numDetectedMarkers, camera_pos, camera_ori, cameraMatrix, cameraDistortion, rmat, tvec = runCalibration()
if(numDetectedMarkers < 4):
	print("Exiting program.")
	quit()


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

fnames = sorted(glob.glob('*.jpg', recursive=False))

coord_offset = 0
for i in range(60):
	coord_offset+=30
	name = ("%d.txt" % (i))
	with open(name, 'w') as f:
		for fname in fnames:
			image = cv2.imread(fname)
			keypoints = blob.detectBlob_HighRes(image)
			for keypoint in keypoints:
				keypoint = cv2.fisheye.undistortPoints(keypoint, cameraMatrix, cameraDistortion, None, cameraMatrix)
				x=keypoint.pt[0]+coord_offset
				y=keypoint.pt[1]
				keypoint_rw = getWorldCoordsAtZ([x, y], 0, cameraMatrix, rmat, tvec)
				output = "%.3f,%.3f,%.3f,%.3f\n" % (x, y, keypoint_rw[0][0], keypoint_rw[1][0])
				f.write(output)
				print(output)
