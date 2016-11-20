import numpy as np
import cv2
import glob
from matplotlib import pyplot as plt

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpointsL = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
objpointsR = []
imgpointsR = []

images = glob.glob('left*.jpg') 

for fname in images:
    print(fname)
    img = cv2.imread(fname)
    grayL = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, cornersL = cv2.findChessboardCorners(grayL, (9,6),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpointsL.append(objp)

        cv2.cornerSubPix(grayL,cornersL,(11,11),(-1,-1),criteria)
        imgpointsL.append(cornersL)


images = glob.glob('right*.jpg')

for fname in images:
    img = cv2.imread(fname)
    grayR = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, cornersR = cv2.findChessboardCorners(grayR, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpointsR.append(objp)

        cv2.cornerSubPix(grayR,cornersR,(11,11),(-1,-1),criteria)
        imgpointsR.append(cornersR)



retval,cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpointsL, imgpointsL, imgpointsR, (320,240))

# Assuming you have left01.jpg and right01.jpg that you want to rectify
lFrame = cv2.imread('LA.jpg')
rFrame = cv2.imread('RA.jpg')
w, h = lFrame.shape[:2] # both frames should be of same shape
frames = [lFrame, rFrame]

# Params from camera calibration
camMats = [cameraMatrix1, cameraMatrix2]
distCoeffs = [distCoeffs1, distCoeffs2]

camSources = [0,1]
for src in camSources:
    distCoeffs[src][0][4] = 0.0 # use only the first 2 values in distCoeffs

# The rectification process
newCams = [0,0]
roi = [0,0]
for src in camSources:
    newCams[src], roi[src] = cv2.getOptimalNewCameraMatrix(cameraMatrix = camMats[src], 
                                                           distCoeffs = distCoeffs[src], 
                                                           imageSize = (w,h), 
                                                           alpha = 0)



rectFrames = [0,0]
for src in camSources:
        rectFrames[src] = cv2.undistort(frames[src], 
                                        camMats[src], 
                                        distCoeffs[src])

# See the results
view = np.hstack([frames[0], frames[1]])    
rectView = np.hstack([rectFrames[0], rectFrames[1]])

cv2.imshow('view', view)
cv2.imshow('rectView', rectView)

imgL = cv2.imread('LA.jpg',0)
imgR = cv2.imread('RA.jpg',0)

print distCoeffs[0]
dstLeft=cv2.undistort(imgL,camMats[0],distCoeffs[0])     
dstRight=cv2.undistort(imgR,camMats[1],distCoeffs[1])

dstLeft2=cv2.undistort(imgL,camMats[1],distCoeffs[1])
dstRight2=cv2.undistort(imgR,camMats[0],distCoeffs[0])


cv2.imshow('newLeft',dstLeft)
cv2.imshow('newRight',dstRight)

cv2.imshow('newLeft',dstLeft2)
cv2.imshow('newRight',dstRight2)

stereo = cv2.StereoSGBM(1, 112 ,3)
disparity = stereo.compute(dstLeft2,dstRight2)
plt.imshow(disparity)
plt.show()
disparity2 = stereo.compute(dstLeft,dstRight)
plt.imshow(disparity2)
plt.show()



# Wait indefinitely for any keypress
cv2.waitKey(0)