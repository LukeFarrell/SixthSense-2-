import numpy as np
import cv2
from matplotlib import pyplot as plt
import Image
import time


capLeft = cv2.VideoCapture(1)
capRight = cv2.VideoCapture(0)

for x in range(10):
	# Capture frame-by-frame
	retL, frameL = capLeft.read()
	retR, frameR = capRight.read()

	# edgesL = cv2.Canny(frameL,100,200)

	# edgesR = cv2.Canny(frameR,100,200)
	# do what you want with frame
	#  and then save to file
	# cv2.imwrite('C:/Users/Luke Farrell/Desktop/HACKDUKE/LA.jpg', frameL)
	# cv2.imwrite('C:/Users/Luke Farrell/Desktop/HACKDUKE/RA.jpg', frameR)

	# imgL = cv2.imread('LA.jpg',0)
	# imgR = cv2.imread('RA.jpg',0)
	cv2.imshow('left', frameL)
	cv2.imshow('right', frameR)

	stereo = cv2.StereoSGBM(1, 112 ,25)
	disparity = stereo.compute(frameL,frameR)
	plt.imshow(disparity)
	plt.show()
	time.sleep(1)
	plt.close()

# When everything done, release the capture
capLeft.release()
capRight.release()
cv2.destroyAllWindows()