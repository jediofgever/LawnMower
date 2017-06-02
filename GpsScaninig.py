#!/usr/bin/env python
# -*- coding: utf-8 -*-
 

import numpy as np
import cv2
import time
import math
 
from math import atan2, degrees, pi
global imghsv

GpsArray = []
GpsArray1 = []
DistanceArray = []
LineLenght = []
StepSize = 10
 
receieved = 2817982719710012
readed_value = str(receieved)
#print(receieved)
time.sleep(0.1)


y1=int(20)							       						
x1=int(20)                                                         # from arduino 
angle =  int(0)                                                     # from arduino
length = 10
theta = angle * 3.14 / 180.0
x2 = x1 + length * math.cos(theta)
y2 = y1 + length * math.sin(theta)

lower_green = np.array([29,86,6])
upper_green = np.array([64,255,255])


                                         			   #GPS MODULE
Array1 = [(22,20),(29,23),(38,20),(39,21),(39,20),(44,22),(49,22),(59,19),(61,23),(70,20),(80,22),(90,22),(101,19),(106,23)]
Array1.append((x1,y1))
Array2 = [(24,20),(28,20),(30,20),(33,20),(35,20),(38,20),(44,20),(48,20),(50,20),(68,20),(76,22),(88,22),(96,19),(102,23)]
Array2.append((x2,y2))
ArrayAngle = []
ArrayAngle.append(angle)


gps_image = np.zeros((300,300,3), np.uint8)
gps_image[0:300,0:300] = (0,255,0)

for i in range (0,len(Array1),1):
    cv2.circle(gps_image, (int(Array1[i][0]),int(Array1[i][1])), 2,(255, 255, 255), 3)       
    cv2.line(gps_image, (int(Array1[i][0]),int(Array1[i][1])), (int(Array2[i][0]),int(Array2[i][1])),(255,255,255),2) 


gps_imageGray = cv2.cvtColor(gps_image, cv2.COLOR_BGR2GRAY)


imghsv=cv2.cvtColor(gps_image,cv2.COLOR_BGR2HSV)					 
imggreen=cv2.inRange(imghsv,lower_green, upper_green) 

cv2.imshow("www",imggreen)

for j in range (0,imggreen.shape[1],StepSize):                                             	   
    for i in range(imggreen.shape[0]-5,0,-1):   		    
                                      			    
         if (imggreen.item(i,j)==0): 	           
            GpsArray.append((j,i))     		    
            break                         			  
    else:                               			  
            GpsArray.append((j,0))  

for j in range (0,imggreen.shape[1],StepSize):                                             	   
    for i in range(0,imggreen.shape[0],1):   		    
                                      			    
         if (imggreen.item(i,j)==0): 	           
            GpsArray1.append((j,i))     		    
            break                         			  
    else:                               			  
            GpsArray1.append((j,0)) 


for x in range (len(GpsArray)-1):         
    cv2.line(gps_image, GpsArray[x], GpsArray[x+1],(0,0,255),2)    
for x in range (len(GpsArray)):         
    cv2.line(gps_image, ((x)*StepSize,300), GpsArray[x],(0,0,255),1)   


for x in range (len(GpsArray1)-1):         
    cv2.line(gps_image, GpsArray1[x], GpsArray1[x+1],(0,0,255),2)    
for x in range (len(GpsArray1)):         
    cv2.line(gps_image, ((x)*StepSize,0), GpsArray1[x],(0,0,255),1)     			                
              

cv2.imshow("gps",gps_image) 
cv2.waitKey(0)
cv2.destroyAllWindows()

