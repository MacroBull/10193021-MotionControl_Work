#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Sat May 31 20:36:03 2014
Project	:OSU Cookie!! - Make servo works like Cookiez!
	The work for 10193021 - 孟濬 - 运动控制技术.
Version	:0.1.4
@author	:Macrobull
	Teammate: Unix Yu, Catherine Wang
"""

import numpy as np
import cv2
import cv2.cv as cv
import time
import serial
from math import *

from macrobull.misc import linear

capture = cv.CaptureFromCAM(2)

########### Target screen size (16/9) ####################
SCR_LEN = 320
SCR_WID = 180
#SCR_WID = 240
#SCR_LEN = 256
#SCR_WID = 192

########### Video framerate control & Output rate ################

#PERIOD = 143 # fps = 7
#PERIOD = 83 # fps = 15
#PERIOD = 62 # fps = 16
PERIOD = 55 # fps = 18
#PERIOD = 50 # fps = 20
ACTION_PERIOD = 1

########### Expected range of radius of circles ####################

MINR = int(0.05 * SCR_WID)
MAXR = int(0.08  * SCR_WID)
#MINDIST = int(0.10 * SCR_WID)
MINDIST = MAXR

############ Circle score parameters for removing duplicate and denoising ###########################

cirScoreInit = 200
cirScoreInc = 3
cirScoreDec = 9
cirScoreMax = 250
cirScoreThr = 150
cirScoreDone = 100

sameCirTh = MINDIST ** 2  / 3

################## Operation parameters ###################
#CLOSE_KERNEL_SHAPE = (3,3)
#CLOSE_KERNEL_SHAPE = (9,9)
#OPEN_KERNEL_SHAPE = (9,9)
OPEN_KERNEL_SHAPE = (13,9)

rectContCredit = 70

############## Debug ##################

DEBUG_TIME = False
#DEBUG_TIME = True

###################### Initialize Correction Library ########
try:
	coreAvailable = 1
	from macrobull.projCorrection.core import *
	print hello() # ProjCorrection Library Test
except BaseException:
	print "Using OpenCV internal correction method"
	coreAvailable = 0

############### Initialize serial interface #################

#ser = serial.Serial("COM6", 115200, timeout = 1) #For Windows
try:
	from macrobull.misc import serialChecker
	dev = serialChecker(True, 'USB','AMA','ACM') #For *nix
	ser = serial.Serial(dev, 9600, timeout = 1)  #For Linux kernel ACM reset
	ser.close()
	ser = serial.Serial(dev, 115200, timeout = 1)
except BaseException:
	ser = None  #Fallback to offline mode


################### Initialize ######################

xOutput = yOutput = 0
circles = []

xmap = [0] * SCR_LEN * SCR_WID
ymap = [0] * SCR_LEN * SCR_WID
counter = 1
img2 = np.zeros((SCR_WID, SCR_LEN,3), dtype = np.uint8)

def timeElps(s): # Time benchmark for debug
	if DEBUG_TIME: print s+ '\t', time.time() - tStart

def findTL(pList): # Rotate quadrangle(pList) until P0 = TopLeft
	while not((pList[0][0]+pList[0][1] <= pList[1][0]+pList[1][1]) and
		(pList[0][0]+pList[0][1] <= pList[2][0]+pList[2][1]) and
		(pList[0][0]+pList[0][1] <= pList[3][0]+pList[3][1])):
			tmp = pList[3]
			pList[1:] = pList[:-1]
			pList[0] = tmp
	return pList


def calPos(x,y): # Convert centimeter coord to servo position value, models provided by Catherine. W.
	powx=x**2
	powy=y**2
	sqrtxy=sqrt(powx+powy)
	theta = 180*atan(x/y)/3.14159
	theta2 = 180*acos(((powx+powy)+12.12)/20.8/sqrtxy)/3.14159
	theta3 = 180*acos((204.2-powx-powy)/203.84)/3.14159
	#K0_pos = (int)((13500-100*theta)/9)
	K0_pos = (int)((1600 * 9-100*theta)/9)
	K1_pos = (int)(384+100*theta2/9)
	K2_pos = (int)(2500-100*theta3/9)
	return K0_pos,K1_pos,K2_pos

def updateCircle(nc): # Update candidate circle queue
	newCir = True
	for i, c in enumerate(circles):
		if (c[0] - nc[0])**2 + (c[1] - nc[1])**2 < sameCirTh:
			if (c[3]>0) and (c[3]<cirScoreMax):  # Regard as the same circle, update to average value
				circles[i][0] = (c[0] + int(nc[0])) >> 1
				circles[i][1] = (c[1] + int(nc[1])) >> 1
				circles[i][2] = (c[2] + int(nc[2])) >> 1
				circles[i][3] += cirScoreInc
			newCir = False
			break
	if newCir: # A new circle found
		circles.append([int(nc[0]), int(nc[1]), int(nc[2]), cirScoreInit])


def display_detected(): # Show all detected circles for debug
	#@img3 = np.copy(img2)
	if None != detected_circles:
		for c in detected_circles[0,:]:
			cv2.circle(img2, (c[0], c[1]), c[2],(255, 0,0),2)
			cv2.circle(img2,(c[0], c[1]), 2,(255,0,255),3)
		#cv2.imshow('GG circles',img3)

def display_queue(): # Show all circles in queue for debug
	i = 0
	l = len(circles)
	for c in circles:
		i += 1
		cv2.circle(img2,(c[0], c[1]), 2, (cirScoreMax - c[3], i * 256 / l, 255), 3)


def correctMethod0():
		global img2
		src = [p0, p1, p2, p3]
		dst = [(0,0),(0,SCR_WID),(SCR_LEN,SCR_WID),(SCR_LEN,0)]
		src = np.array(src, dtype = "float32")
		dst = np.array(dst, dtype = "float32")
		M = cv2.getPerspectiveTransform(src, dst)
		timeElps("Built:")

		img2 = cv2.warpPerspective(img, M, (SCR_LEN, SCR_WID))
		timeElps("Remapped:")

if coreAvailable:
	def correctMethod1():
			global img2
			build2(xmap, ymap, SCR_LEN, SCR_WID, p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], 0) # Implemented in macrobull/projCorrection/core.c for performance, model provided by Catherine. W.
			timeElps("Built:")
			img2 = img[ymap, xmap].reshape((SCR_WID, SCR_LEN, 3)) # Remapping image
			#cv2.remap(img, img2, xmap, ymap, 0)
			timeElps("Remapped:")
else:
	correctMethod1 = correctMethod0


while 1: # Main loop

	tStart = time.time()
	#for i in range(1): frame = cv.QueryFrame(capture)
	frame = cv.QueryFrame(capture)
	img = np.asarray(frame[:,:]) # Caputre image

	hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) # Find colors in HSV space for best performance

	################ Find color presets ##################

	#inr = cv2.inRange(hsv, (90,90,00), (140,255,255)) #blue
	inr = cv2.inRange(hsv, (110,90,00), (140,255,255)) #blue
	#inr = cv2.inRange(img, (220,20,20), (255, 255,200)) # blue
	#inr = cv2.inRange(img, (0,0,0), (60, 40, 40)) #black
	#inr = cv2.inRange(img, (200,200,200), (255, 255, 255)) #white
	#inr = cv2.inRange(img, (160, 160, 160), (255, 255, 255)) #white


	#inr = cv2.morphologyEx(inr, cv2.MORPH_CLOSE, np.ones(CLOSE_KERNEL_SHAPE, dtype = np.uint8)) # Close operation for denoise
	inr = cv2.morphologyEx(inr, cv2.MORPH_OPEN, np.ones(OPEN_KERNEL_SHAPE, dtype = np.uint8)) # Close operation for denoise
	#inr = cv2.GaussianBlur(inr, (11, 11), 0) # Gaussian blur for denoise


	###################### Find color output #####################
	#cv2.imshow("HSV", hsv)
	#cv2.imshow("inRange", inr)


	cont, hier = cv2.findContours(inr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Find the contour of the quadrangle of screen

	timeElps("Find Screen:")
	h = 0
	cmax = 1
	for c in cont:
		cl = len(c)
		if cl> cmax:
			cmax = cl
			cc = c

	if cmax> rectContCredit: # The really large polygon is what I want
		l = 0.
		r = 1.
		while (h != 4) and (r - l > 0.01): # Binary search for h = 4, simplify to a quadrangle
			m = (l + r) / 2
			approx = cv2.approxPolyDP(cc, m*cv2.arcLength(cc, True), True)
			h = len(approx)
			if h > 4 :
				l = m
			else:
				r = m
	else:
		rectContCredit = (cmax + rectContCredit) >>1

	if h == 4: # Success to find a quadrangle
		p0, p1, p2, p3 = approx[0,0], approx[1,0], approx[2,0], approx[3,0]
		p0, p1, p2, p3 = findTL([p0, p1, p2, p3]) # Rotate to the front view to the screen

		timeElps("Get bound:")

		############### Debug for screen geometry ########################
		#for p in (p0, p1, p2, p3): print "({},{})\t".format(p[0], p[1]),
		#print ""

		####################### Correction to rectangle ###################
		#correctMethod1()
		correctMethod0()

		cv2.drawContours(img, [approx], 0, (0,255,0), 3) # Draw the quadrangle for debug
		#cv2.imshow('img2',img2)
		img2 = cv2.GaussianBlur(img2, (3, 3), 0) #Blur the corrected result for denoising and debanding
		grey = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)# Graying and binarization for better performance in Cannay operation

		'''
		detected_circles = cv2.HoughCircles(grey,cv.CV_HOUGH_GRADIENT,1,MINDIST,
			param1=180,param2=18,minRadius= MINR,maxRadius= MAXR)

		'''
		thr, grey = cv2.threshold(grey, linear(np.average(grey), np.max(grey), 0.4), 255, cv2.THRESH_BINARY)
		detected_circles = cv2.HoughCircles(grey,cv.CV_HOUGH_GRADIENT,1,MINDIST,
			param1=180,param2=10,minRadius= MINR,maxRadius= MAXR) # Detect circles
		'''
		'''


		#cv2.imshow('grey',grey)
		#display_detected()

		timeElps("Found circles:")

		################ Update candidate queue #########################
		if None != detected_circles:
			#print detected_circles
			for c in detected_circles[0,:]:
				updateCircle(c)

		for i in range(len(circles) -1, -1, -1):
			circles[i][3] -= cirScoreDec
			if circles[i][3] < 0:
				circles.pop(i)

		#print circles
		display_queue()
		print "Candidate length:", len(circles)

		if 0 == counter: ########### Frame to ouput ##########
			for i in range(len(circles)): # Find first circle in queue to output
				if  circles[i][3]> cirScoreThr:
					xOutput = circles[i][0]
					yOutput = circles[i][1]
					cv2.circle(img2,(xOutput, yOutput), circles[i][2],(0, 0, 0), 4)
					cv2.circle(img2,(xOutput, yOutput), 2,(0, 0, 255), 3)
					#circles.pop(i)
					circles[i][3] = cirScoreDone
					print "output = ({}, {})[{}]".format(xOutput, yOutput, circles[i][2])
					break


			if ser: # Send instructions
				################ Convert pixels to centimeters #############
				#print "original = ", (xOutput, yOutput)
				x = xOutput/19.753086419753 * 1.84 - 16
				y = 24  - 2.5 - yOutput/19.753086419753 * 2.

				#x = xOutput/19.753086419753 * 1.84 + 16
				#y = 8 + yOutput/19.753086419753 * 2.

				#x = -x
				#y = 32 - y

				if x**2+ y**2>19**2: y = (19**2-x**2)**0.5

				print "in cm = ", (x,y)
				############# Convert centimeters to instruction ############
				K0_pos,K1_pos,K2_pos = calPos(x,y)
				ins = '#0 P{} #1 P{} #2 P{} T200\r\n'.format(K0_pos,K1_pos,K2_pos)
				print ins
				ser.write( ins)

			cv2.imshow('Detected circles',img2)

	cv2.imshow("Original",img)

	timeElps("Display done:")
	print "Loop:", counter
	#gc.collect()
	delay = PERIOD - int((time.time()-tStart)*1000) # Keep framerate constant simply
	print "Delay:", delay
	if delay <= 0 : delay = 1
	if 27 == cv2.waitKey(delay): break
	counter =0 if counter >= ACTION_PERIOD else counter + 1
	if ser: # In case serial buffered too much
		ser.flushOutput()
		ser.flush()

################## Quit the program ###########################
cv2.destroyAllWindows()
if ser: ser.close()
print "closed"