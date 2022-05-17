
import math
import cv2
import sys
import glob
import scipy.io
from palettable.cartocolors.sequential import DarkMint_6
import matplotlib.pyplot as plt

# Add common modules path
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/common')
sys.path.insert(1, '/home/pi/Camera_Indoor_Positioning/system/192.168.82.10')
import blob_detector as blob
from sys_calibration_bare import *
from numpy.linalg import inv
from numpy import array, cross
from numpy.linalg import solve, norm

fname = "cam2_calib.npz"
calib_file = np.load(fname)
camera_pos=calib_file['camera_pos']
camera_ori=calib_file['camera_ori']
cameraMatrix=calib_file['cameraMatrix']
cameraDistortion=calib_file['cameraDistortion']
rmat=calib_file['rmat']
tvec=calib_file['tvec']

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


camera_height = 1520
camera_width  = 2016 

sigma_x = 0.06
sigma_y = 0.02

Ntests = 100
pixel_step=8

mean_x_err=np.zeros( (int(camera_height/pixel_step), int(camera_width/pixel_step)) )
std_x_err= np.zeros( (int(camera_height/pixel_step), int(camera_width/pixel_step)) )

for x in range(0, camera_width, pixel_step):
	for y in range(0, camera_height, pixel_step):
		print("Testing for (%d,%d)" % (x, y))
		proj_err_rw=[]
		blob_centers_err = np.random.normal(0.0, sigma_x, Ntests).reshape(Ntests,1)
		blob_centers_err = np.tile([x, y],(Ntests,1))+np.append(blob_centers_err, np.zeros((Ntests,1)), axis = 1)
		blob_centers = np.tile([x, y],(Ntests,1))

		for i in range(len(blob_centers)):
			proj_err_rw_it=abs(getWorldCoordsAtZ(blob_centers[i], 0.0, cameraMatrix, rmat, tvec)-getWorldCoordsAtZ(blob_centers_err[i], 0.0, cameraMatrix, rmat, tvec))
			proj_err_rw.append(proj_err_rw_it)
				
		x_s=int(x/pixel_step)
		y_s=int(y/pixel_step)
		mean_x_err[y_s, x_s]=np.mean(proj_err_rw)
		std_x_err[y_s, x_s]=np.std(proj_err_rw)

scipy.io.savemat('mean_x_err.mat', {'mean': mean_x_err})
scipy.io.savemat('std_x_err.mat', {'std': std_x_err})

xx, yy = np.mgrid[0:std_x_err.shape[0], 0:std_x_err.shape[1]]

# Creating plot
#ax.plot_surface(xx, yy, std_x_err, cmap=DarkMint_6.mpl_colormap)
c = plt.imshow(std_x_err, cmap=DarkMint_6.mpl_colormap)
plt.colorbar(c)
  
plt.title('Error of projection in real world coordinates (mm)', fontweight ="bold")
plt.show()

exec(open('mail.py').read())
