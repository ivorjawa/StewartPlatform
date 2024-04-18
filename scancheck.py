#!/usr/bin/env python

import numpy as np
import cv2
import uvc 

def find_mode(cap, width, height, fps):
    """
    for mode in cap.available_modes:
        print(
            f"MODE: {mode.width} x {mode.height} @ {mode.fps} ({mode.format_name}) {mode.__class__}"
        )
    """
    for mode in cap.available_modes:
        if (mode.width == width) and (mode.height == height) and (mode.fps == fps):
            return mode
            
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
width = 640
height = 480
device = uvc.device_list()[0]
cap = uvc.Capture(device["uid"]) 
cap.frame_mode = find_mode(cap, width, height, 30) 
print(f"frame mode: {cap.frame_mode}")
while True:
    img = cap.get_frame()
    #print (img)
    img = img.bgr
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #gray = img.gray
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)

    # If found, add object points, image points (after refining them)
    waittime = 1
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (8,6), corners2, ret)
        waittime = 500
        
    cv2.line(img, (int(0),int(height/2)), (int(width), int(height/2)), (0, 0, 255), 1)
    cv2.line(img, (int(width/2),int(0)), (int(width/2), int(height)), (0, 0, 255), 1)
    
    cv2.imshow('img', img)
    if (cv2.waitKey(waittime) == 27):
        break