import cv2
import numpy as np
from matplotlib import pyplot as plt
 
imgL = cv2.imread('LA.jpg',0)
edgesL = cv2.Canny(imgL,100,200)

imgR = cv2.imread('RA.jpg',0)
edgesR = cv2.Canny(imgR,100,200)

cv2.imwrite('C:/Users/Luke Farrell/Desktop/HACKDUKE/LAE.jpg', edgesL)
cv2.imwrite('C:/Users/Luke Farrell/Desktop/HACKDUKE/RAE.jpg', edgesR)
