import cv2 as cv
import numpy as np
import sys
from enum import Enum

# XMING notes (when using WSL on windows 10)
# need to install xserver, export display, and run xlaunch 
#break image up into grid to allow for features to be well distributed
#across the image frame
# start XMING X server
# export DISPLAY=:0 variable
# does not use virtual env

# class FeatureExtractor(object):
# 	def __init__(self):
# 		pass
# 	def extract(self, img):
# 		return


class KPAlgorithm(Enum):
	SIFT 	= 	1
	SURF 	= 	2
	ORB 	=	3


#test functions before creating classes
#
#
# INPUTS: input cv2 mat, and algorithm enum (SIFT, ORB ...)
# Outputs: keypoints, descriptors
def extractFeatures(img, algorithm):
	kp = []
	des = []
	#SIFT, SURF, ORB
	if algorithm == KPAlgorithm.SIFT:
		#sift with default parameters
		sift = cv.SIFT_create()
		kp, des = sift.detectAndCompute(img, None)
	elif algorithm == KPAlgorithm.SURF:
		#surf with default parameters
		surf = cv.SURF_create()
		kp, des = surf.detectAndCompute(img, None)
	elif algorithm == KPAlgorithm.ORB:
		#orb with default parameters
		orb = cv.ORB_create()
		kp, des = orb.detectAndCompute(img, None)
	else:
		print("Incompatible algorithm type")
	return kp, des

def matchKeypoints(kpl, dl, kpr, dr):
	# FLANN parameters
	FLANN_INDEX_KDTREE = 1
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks=50)   # or pass empty dictionary
	flann = cv.FlannBasedMatcher(index_params,search_params)
	matches = flann.knnMatch(dl, dr, k=2)
	# Mask for good matches
	mask = [[0,0] for i in range(len(matches))]
	# ratio test as per Lowe's paper (from opencv tutorial)
	ptsL = []
	ptsR = []
	# ratio test as per Lowe's paper
	for i,(m,n) in enumerate(matches):
		if m.distance < 0.8*n.distance:
			ptsR.append(kpr[m.trainIdx].pt)
			ptsL.append(kpl[m.queryIdx].pt)
	params = dict(matchColor = (0,255,0),
					singlePointColor = (255,0,0),
					matchesMask = mask,
					flags = cv.DrawMatchesFlags_DEFAULT)
	ptsL = np.int32(ptsL)
	ptsR = np.int32(ptsR)
	fMat, maskF = cv.findFundamentalMat(ptsL,ptsR,cv.FM_LMEDS)
	# We select only inlier points
	ptsL = ptsL[maskF.ravel()==1]
	ptsR = ptsR[maskF.ravel()==1]
	return ptsL, ptsR, params, fMat



def undistortImagePair(imgLeft, imgRight, camMatL, camMatR, distCoeffL, distCoeffR):
	undLeft = cv.undistort(imgLeft, camMatL, distCoeffL)
	undRight = cv.undistort(imgRight, camMatR, distCoeffR)
	return undLeft, undRight

#default to un-calibrated camera
#so must solve fundamental matrix



#TODO: add rectification for calibrated camera pair
# requires a setreo camera (intel realsense camera (with imu?))
#rectify stereoPair uncalibrated*
def rectifyStereoPairUncal(imgL, imgR, pointsLeft, pointsRight, fMat, imgSize):
	#output homography transforms for image pair
	_, HL, HR = cv.stereoRectifyUncalibrated(pointsLeft, pointsRight, fMat, imgSize)
	warpL = cv.warpPerspective(imgL, HL, imgSize)
	warpR = cv.warpPerspective(imgR, HR, imgSize)
	return warpL, warpR

#Stereo depth
#test with https://vision.middlebury.edu/stereo/data/scenes2005/
# Feature extraction and matching
# for frame1, frame10:
	# extract()
	# match()
	# ...
	# getDisparity()
	# getPointCloud()
	# buildMesh()

# undistort()
#	kp1, des1 from imgLeft
# 	kp2, des2 from imgRight
# 	match keypoints
#	
# rectify your input images
# 




if __name__ == "__main__":
	
	#Verify using opencv 4.6.0
	print(cv.__version__)

	#Load a test image
	IMG_DIR 				= "./testImages/"
	IN_IMGL_FNAME 			= "inputLeft.jpg"
	IN_IMGR_FNAME 			= "inputRight.jpg"
	OUT_IMG_KP_FNAME 		= "kpMatchesOutput.jpg"
	IN_IMGL_PATH 			= f"{IMG_DIR}{IN_IMGL_FNAME}"
	IN_IMGR_PATH 			= f"{IMG_DIR}{IN_IMGR_FNAME}"
	OUT_IMG_KP_PATH 		= f"{IMG_DIR}{OUT_IMG_KP_FNAME}"

	#Camera Matrix parameters
	#need to calibrate cameras first

	fxl = 1.0
	fyl = 1.0
	cxl = 0.0
	cyl = 0.0

	fxr = 1.0
	fyr = 1.0
	cxr = 0.0
	cyr = 0.0


	camMatL = np.array([[fxl, 0.0, cxl],
					[0.0, fyl, cyl],
					[0.0,0.0,1.0]])

	camMatR = np.array([[fxr, 0.0, cxr],
					[0.0, fyr, cyr],
					[0.0,0.0,1.0]])

	#5 array of distortion coefficients (input parameters)
	distL = [0,0,0,0,0]
	distR = [0,0,0,0,0]

	#Read in image pair
	# TODO:
	# make into for loop for live video processing (maybe every 10 frames or so*)
	grayimgL = cv.imread(IN_IMGL_PATH, cv.IMREAD_GRAYSCALE)
	grayimgR = cv.imread(IN_IMGR_PATH, cv.IMREAD_GRAYSCALE)

	#TODO:
	#assert input images are the same size*

	#Undistort image pair
	undLeft, undRight = undistortImagePair(grayimgL, grayimgR, camMatL, camMatR, distL, distR)

	#Perform SIFT on stereo image pair
	kpl, desl = extractFeatures(undLeft, KPAlgorithm(1)) 
	kpr, desr = extractFeatures(undRight, KPAlgorithm(1))

	#Draw keypoints (just to see algorithm performance)
	kpImgL = cv.drawKeypoints(undLeft, kpl, None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	kpImgR = cv.drawKeypoints(undRight, kpr, None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		#output as an image
		#cv.imwrite(OUT_IMG_KP_PATH, kpImgL)
		#Draw matched keypoints using FLANN based matcher and write to a file

	#Match keypoints 
	ptsL, ptsR, params, fMat = matchKeypoints(kpl, desl, kpr, desr)

	#rectify stereo pair
	rectifiedLeft, rectifiedRight = rectifyStereoPairUncal(undLeft, undRight, ptsL, ptsR, fMat, grayimgL.shape)
	#output rectified stereo pair of images

	print("completed feature extraction")


