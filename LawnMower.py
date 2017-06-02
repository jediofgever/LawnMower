#!/usr/bin/env python
# -*- coding: utf-8 -*-
from picamera.array import PiRGBArray
from picamera import PiCamera
from random import randint
from time import sleep
import RPi.GPIO as GPIO
import numpy as np
import cv2
import time
import math
import serial
from math import atan2, degrees, pi
global imghsv


time.sleep(0.1)



def blink(pin):
	GPIO.output(pin, GPIO.LOW)
        time.sleep(0.125)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.125)
	GPIO.output(pin, GPIO.LOW)
        time.sleep(0.125)
        GPIO.output(pin, GPIO.HIGH)
        return
GPIO.setmode(GPIO.BOARD) 
GPIO.setup(13, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)

	
	 
		        
	
#ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5)
#time.sleep(1.5)
#ser.flush()


capture =cv2.VideoCapture(0) 
capture1 =cv2.VideoCapture(1) 

while True:
    
    blink(11)

    f, frame = capture.read()
    frame = cv2.resize(frame,(320,240))  
    frame[118:122,158:162] = (0,255,0) 
    image = frame
    img  = image
   




    f, frame1 = capture1.read()
    frame1 = cv2.resize(frame1,(480,360))
    frame1 = frame1[100:280,0:480]  
    crop_img = frame1  			                           # Crop from x, y, w, h -> 100, 200, 300, 400
    img_below = crop_img                                         			 # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
    crop_img[100:180,239:241] = (0,255,0)

    imghsv=np.zeros(image.shape,dtype=np.uint8)
    imghsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    
    imghsv1=np.zeros(crop_img.shape,dtype=np.uint8)
    imghsv1=cv2.cvtColor(crop_img,cv2.COLOR_BGR2HSV)
                
        
    lower_green = np.array([29,86,6])
    upper_green = np.array([64,255,255])
  
    imggreen=cv2.inRange(imghsv,lower_green, upper_green)             # get green values from upper cam
    imggreen_capture1=cv2.inRange(imghsv1,lower_green, upper_green)   # get green values for below cam
    
    
     
    color_image1 = cv2.bitwise_and(crop_img,crop_img, mask=imggreen_capture1)

   
    imgray = imggreen
    imggreen = cv2.medianBlur(imggreen,3)
    imggreen_capture1 = cv2.medianBlur(imggreen_capture1,3)
 
    
 
    f, thres = cv2.threshold(imggreen,127,255,0)
    f, thres_below = cv2.threshold(imggreen_capture1,127,255,0)
    print(thres_below)
    _, contours, _ = cv2.findContours(thres,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    _, contours_below, _ = cv2.findContours(thres_below,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 

    areaArray = []
    areaArray_below = []

    count = 1

    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        areaArray.append(area)
    sorteddata = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)
    largestcontour = sorteddata[0][1]
 
    for c in contours:
								# compute the center of the contour
	M = cv2.moments(c)
	cX = int(M["m10"] / (M["m00"]+1))
	cY = int(M["m01"] / (M["m00"]+1))
 
								# draw the contour and center of the shape on the image
	x,y,w,h = cv2.boundingRect(largestcontour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),11)
        cv2.drawContours(image, largestcontour, -1, (0, 255, 255), 11)
 
 

    
    image1 =  cv2.medianBlur(image,3)


  
    for i, c in enumerate(contours_below):
        area_below = cv2.contourArea(c)
        areaArray_below.append(area_below)

    sorteddata_below = sorted(zip(areaArray_below, contours_below), key=lambda x: x[0], reverse=True)
    largestcontour_below = sorteddata_below[0][1]
 
    for c in contours_below:
                                                                  # compute the center of the contour
	M = cv2.moments(c)
	cX = int(M["m10"] / (M["m00"]+1))
	cY = int(M["m01"] / (M["m00"]+1))                                                         # draw the contour and center of the shape on the image
	x,y,w,h = cv2.boundingRect(largestcontour_below)
        cv2.rectangle(color_image1,(x,y),(x+w,y+h),(0,0,255),11)
        cv2.drawContours(color_image1, largestcontour_below, -1, (0, 255, 255), 3)
 
    crop_img1 = cv2.medianBlur(crop_img,3)

    StepSize = 2
    EdgeArray = []
    EdgeArray_below = []
    GpsArray = []
    DistanceArray = []
    LineLenght = []


    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)                 #convert img to grayscale and store result in imgGray
    imgGray = cv2.bilateralFilter(imgGray,9,30,30)                 #blur the image slightly to remove noise             
    imgEdge = cv2.Canny(imgGray, 50, 100)                          #edge detection
 
    imgGray_below = cv2.cvtColor(img_below,cv2.COLOR_BGR2GRAY)      #convert img to grayscale and store result in imgGray
    imgGray_below = cv2.bilateralFilter(imgGray_below,9,30,30)     #blur the image slightly to remove noise             
    imgEdge_below = cv2.Canny(imgGray_below, 50, 100)              #edge detection
     
    imagewidth = imgEdge.shape[1] - 1
    imageheight = imgEdge.shape[0] - 1
    
    imagewidth_below = imgEdge_below.shape[1] - 1
    imageheight_below = imgEdge_below.shape[0] - 1
   
    
    for j in range (0,imagewidth,StepSize):    			   #for the width of image array
        for i in range(imageheight-5,0,-1):    			   #step through every pixel in height of array from bottom to top
                                               			   #Ignore first couple of pixels as may trigger due to undistort
            if imggreen.item(i,j) == 0:      			   #check to see if the pixel is white which indicates an edge has been  
                EdgeArray.append((j,i))     			   #if it is, add x,y coordinates to ObstacleArray
                break                       			   #if white pixel is found, skip rest of pixels in column
        else:                               			   #no white pixel found
            EdgeArray.append((j,0))         			   #if nothing found, assume no obstacle. Set pixel position way          
                                             			   #no obstacle detected
    
    for j in range (0,imggreen_capture1.shape[1],StepSize):        #for the width of image array
        for i in range(imggreen_capture1.shape[0]-5,0,-1):         #step through every pixel in height of array from bottom to top
                                              			   #Ignore first couple of pixels as may trigger due to undistort
            if imggreen_capture1.item(i,j) == 0: 	           #check to see if the pixel is white which indicates an edge has been  
                EdgeArray_below.append((j,i))     		   #if it is, add x,y coordinates to ObstacleArray
                break                           	           #if white pixel is found, skip rest of pixels in column
        else:                                			   #no white pixel found
            EdgeArray_below.append((j,0))         		   #if nothing found, . Set pixel position way off the screen to indicate
    


   

    #receieved = ser.readline()
    receieved = 2817982719710012
    readed_value = str(receieved)
    #print(receieved)
    time.sleep(0.1)


    y1=int(readed_value[2:4])							   # from arduino						
    x1=int(readed_value[6:8])                                                         # from arduino 
    angle = int(readed_value[-4:])                                                     # from arduino
    length = 10
    theta = angle * 3.14 / 180.0
    x2 = x1 + length * math.cos(theta)
    y2 = y1 + length * math.sin(theta)                                         			   #GPS MODULE
    Array1 = []
    Array1.append((x1,y1))
    Array2 = []
    Array2.append((x2,y2))
    ArrayAngle = []
    ArrayAngle.append(angle)



    
    gps_image = np.zeros((100,100,3), np.uint8)
    gps_image[:0:100] = (0,0,0)						# GPS Scanner İmage
    gps_image = cv2.cvtColor(gps_image, cv2.COLOR_BGR2GRAY)
    gps_image = cv2.Canny(gps_image,10,250) 
    



    for i in range (0,len(Array1),1):
        cv2.circle(gps_image, (int(Array1[i][0]),int(Array1[i][1])), 2, (255, 255, 255), 3)        #put received values into Scanner İmage 
        cv2.line(gps_image, (int(Array1[i][0]),int(Array1[i][1])), (int(Array2[i][0]),int(Array2[i][1])),(255,255,255),1) 
        
    
 

        

    for j in range (0,gps_image.shape[1],1):                                             # store mowed points edges into an array  		   
        for i in range(0,gps_image.shape[0],1):   		    
                                              			    
             while(gps_image.item(i,j)==255): 	           
                GpsArray.append((j,i))     		    
                break                           			  
    


    #cv2.imshow("gps",gps_image)# Bu şey önemli
    





    for x in range (0,len(GpsArray),1):
        distance = DistanceArray.append((math.sqrt(abs(x1-GpsArray[x][0])**2+(y1-GpsArray[x][1])**2))) #if current distance is close to                                                                               stored values then turn heading angle about 90 degrees  

    if((min(DistanceArray)) <= 10):
       HeadingAngleGps = ArrayAngle[len(ArrayAngle)-1]+90  





    for x in range (len(EdgeArray)-1):     				 #draw lines between points in ObstacleArray 
        cv2.line(image, EdgeArray[x], EdgeArray[x+1],(255,255,0),1) 
    for x in range (len(EdgeArray)):       				 #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(image, ((x)*StepSize,imageheight), EdgeArray[x],(0,255,0),4)
         

    for x in range (len(EdgeArray)-1):     				 #draw lines between points in ObstacleArray 
        cv2.line(image1, EdgeArray[x], EdgeArray[x+1],(0,255,0),1) 
    for x in range (len(EdgeArray)):        				 #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(image1, ((x)*StepSize,imageheight), EdgeArray[x],(0,255,0),1)

    for x in range (len(EdgeArray_below)-1):      			 #draw lines between points in ObstacleArray 
        cv2.line(crop_img, EdgeArray_below[x], EdgeArray_below[x+1],(255,255,0),1) 
    for x in range (len(EdgeArray_below)):        			 #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(crop_img, ((x)*StepSize,imageheight_below), EdgeArray_below[x],(0,255,0),4)
    for x in range (len(EdgeArray_below)-1):      			 #draw lines between points in ObstacleArray 
        cv2.line(crop_img1, EdgeArray_below[x], EdgeArray_below[x+1],(0,255,0),1) 
    for x in range (len(EdgeArray_below)):       			 #draw lines from bottom of the screen to points in ObstacleArray
        cv2.line(crop_img1, ((x)*StepSize,imageheight_below), EdgeArray_below[x],(0,255,0),1)
   


 
        
    lower_green = np.array([0,255,0])
    upper_green = np.array([0,255,0])
   
    imghsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)   
    imghsv_below=cv2.cvtColor(crop_img,cv2.COLOR_BGR2HSV)             

    contourImage= cv2.inRange(image,lower_green, upper_green)
    contourImage_below= cv2.inRange(crop_img,lower_green, upper_green) 

 
   
    contourImage = cv2.medianBlur(contourImage,9)
    contourImage_below = cv2.medianBlur(contourImage_below,9)
    
    ret, thres2 = cv2.threshold(contourImage,0,255,0)
    _,contours1, _ = cv2.findContours(thres2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)   

    ret, thres2_below = cv2.threshold(contourImage_below,0,255,0)
    _, contours1_below, _ = cv2.findContours(thres2_below,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
    





    for c in contours1:
											# compute the center of the contour
	M = cv2.moments(c)
	cx = int(M["m10"] / (M["m00"]+1))
	cy = int(M["m01"] / (M["m00"]+1))
        cv2.circle(image1, (cx, cy), 3, (0, 0, 255), 5)
        cv2.line(image1, (len(EdgeArray),imageheight),(cx,cy),(0,0,255),8)
	cv2.circle(contourImage, (cx, cy), 3, (0, 255, 0), 5)
	cv2.putText(contourImage, "center", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 255, 0), 9)
        rads = atan2((imageheight-cy),(cx-len(EdgeArray)))
        rads %= 2*pi
        degs = 90-degrees(rads)
        

    for c in contours1_below:
 
	M = cv2.moments(c)
	cx = int(M["m10"] / (M["m00"]+1))
	cy = int(M["m01"] / (M["m00"]+1))
        cv2.circle(crop_img1, (cx, cy), 3, (0, 0, 255), 5)
        cv2.line(crop_img1, (len(EdgeArray_below),imageheight_below),(cx,cy),(0,0,255),8)
	cv2.circle(contourImage_below, (cx, cy), 3, (0, 255, 0), 5)
	cv2.putText(contourImage_below, "center", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 255, 0), 9)
        rads_below=atan2((cx-len(EdgeArray_below)),(cy-imageheight_below)) 
        rads_below %= 2*pi
        degs_below = degrees(rads_below)-270
      
    omega = 0.7     
    zeta = 0.1
    aplha = 0.2 
    heading_angle = omega*degs + aplha*degs_below +  zeta*HeadingAngleGps
    print(heading_angle)



    #ser.write(int(heading_angle))
    
    cv2.imshow("image1", image1)
 
 
  
    cv2.imshow("crop_img1", crop_img1)
 
    


    

    blink(13)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

out.release()
capture.release()
cv2.destroyAllWindows()


