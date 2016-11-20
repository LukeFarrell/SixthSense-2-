import numpy as np
import cv2
import serial
import time
import struct

arduino = serial.Serial('COM3', 9600,timeout=1)
time.sleep(2) #give the connection a second to settle
arduino.flushInput()
arduino.write("1".encode())
print arduino.read()
# arduino.flush()
# print arduino.readline()
# print "DONE"
# time.sleep(4) # with the port open, the response will be buffered 
#               # so wait a bit longer for response here

# # Serial read section

# msg = arduino.read(arduino.inWaiting()) # read everything in the input buffer
# print ("Message from arduino: ")
# print (msg)



# count = 0
# while True:
#     if(count%5==0):
#         letter  = 'L'
#     else:
#         letter  = 'R'
# 	if(arduino.inWaiting()>0):
# 		arduino.write(letter)
# 		myData = arduino.readline()
# 		print myData
#     count+=1
# print "READY"



window_size = 3
min_disp = 16
num_disp = 112-min_disp
stereo = cv2.StereoSGBM(
    minDisparity = min_disp,
    numDisparities = num_disp,
    SADWindowSize = window_size,
    uniquenessRatio = 10,
    speckleWindowSize = 100,
    speckleRange = 32,
    disp12MaxDiff = 1,
    P1 = 8*3*window_size**2,
    P2 = 32*3*window_size**2,
    fullDP = False
)
 
# morphology settings
kernel = np.ones((12,12),np.uint8)
 
counter = 450

capLeft = cv2.VideoCapture(1)
capRight = cv2.VideoCapture(0)
l=0;
r=0;
while True:
 
    # increment counter
    counter += 1
    if counter % 3 != 0: continue
 
    # only process every third image (so as to speed up video)

    retL, frameL = capLeft.read()
    retR, frameR = capRight.read()

    cv2.imwrite('C:/Users/Luke Farrell/Desktop/HACKDUKE/LA.jpg', frameL)
    cv2.imwrite('C:/Users/Luke Farrell/Desktop/HACKDUKE/RA.jpg', frameR)

 
    # load stereo image
    filename = str(counter).zfill(4)
 
    image_left = cv2.imread('LA.jpg'.format(filename))
    image_right = cv2.imread('RA.jpg'.format(filename))
 	

    # compute disparity
    disparity = stereo.compute(image_left, image_right).astype(np.float32) / 16.0
    disparity = (disparity-min_disp)/num_disp
    edgesR = cv2.Canny(image_right,100,200)
    edgesL = cv2.Canny(image_left,100,200)
    y1 = 0
    y2 = len(edgesR)
    x1= 0
    x2 = len(edgesR[0]/6)

    # th,im_th =  cv2.threshold(edgesR+edgesL,200,255,cv2.THRESH_BINARY_INV);
    # im_floodfill =  im_th.copy()
    # h,w =  im_th.shape[:2]
    # mask=np.zeros((h+2,w+2),np.uint8)
    # cv2.floodFill(im_floodfill,mask,(0,0),255);
    # im_floodfill_inv=cv2.bitwise_not(im_floodfill)
    # im_out=im_th | im_floodfill_inv
    # cv2.imshow("thresh",im_th)
    # cv2.imshow("flood",im_floodfill)
    # cv2.imshow("invFlood",im_floodfill_inv)
    # cv2.imshow("fore",im_out)
    thresholdR = cv2.threshold(edgesR, 0, 1.0, cv2.THRESH_BINARY)[1]
    morphologyR = cv2.morphologyEx(thresholdR, cv2.MORPH_OPEN, kernel)
    thresholdL = cv2.threshold(edgesR, 0, 1.0, cv2.THRESH_BINARY)[1]
    morphologyL = cv2.morphologyEx(thresholdL, cv2.MORPH_OPEN, kernel)
    cv2.imshow("ML",morphologyL)
    cv2.imshow("MR",morphologyR)
    blurR = cv2.blur(edgesR,(7,7))
    blurL = cv2.blur(edgesL,(7,7))

    bitAndR=cv2.bitwise_and(disparity,disparity,mask=blurR)
    bitAndL=cv2.bitwise_and(disparity,disparity,mask=blurL)
    final=bitAndR+bitAndL
    # dst = cv2.inpaint(final,final,3,cv2.INPAINT_TELEA)
    # cv2.imshow("inp",dst)
    final1 = final[y1:480, 0:300]
    # final2 = final[y1:480, 100:200]
    # final3 = final[y1:480, 200:300]
    # final4 = final[y1:480, 300:400]
    # final5 = final[y1:480, 400:500]
    final6 = final[y1:480, 300:600]

    cv2.imshow("final1",final1)
    cv2.imshow("final6",final6)
    avgColorRow1=np.average(final1,axis=0)
    avgColor1=np.average(avgColorRow1,axis=0)
    avgColorRow2=np.average(final6,axis=0)
    avgColor2=np.average(avgColorRow2,axis=0)
    print l,r
    if(avgColor1>l):
        l=avgColor1
    if(avgColor2>r):
        r=avgColor2


    if avgColor1 > .5:
        pass
    

    # if(arduino.inWaiting()>0):
    #     if avgColor1 > .01 and avgColor1 < .03:
    #     	arduino.write('A')
    #     	# myData = arduino.readline()
    #     	print "A"
    #     elif avgColor1 > .03 and avgColor1 < .06:
    #         arduino.write('B')
    #         # myData = arduino.readline()
    #         print "B"
    #     elif avgColor1 > .06 and avgColor1 < .09:
    #         arduino.write('C')
    #         # myData = arduino.readline()
    #         print "C"
    #     else:
    #         arduino.write('L')
    #         myData = arduino.readline()
    #         print "L",myData
    #     print "Done"
    #     time.sleep(1)


    # dst = cv2.addWeighted(disparity,0.7,edgesR,0.3,0)
    # for pix in range(len(edgesR)):
    # 	print edgesR[pix]
    # 	if edgesR[pix] == 0:
    # 		disparity[pix][0] = 0
    # cv2.imshow("edge",edgesR)
    # cv2.imshow("blur",blur)
    cv2.imshow("left",image_left)
    cv2.imshow("right",image_right)
    cv2.imshow('bitAND',bitAndR+bitAndL)
    # cv2.imshow('disparity', disparity)
    # cv2.imshow('edges1', edges1)
    # cv2.imshow('edges2', edges2)
    # cv2.imshow('edges3', edges3) 
    # cv2.imshow('edges4', edges4)
    # cv2.imshow('edges5', edges5)
    # cv2.imshow('edges6', edges6)
    # cv2.imshow('combo',bitAnd)
    cv2.waitKey(1)

	
	
	
	
	
	
