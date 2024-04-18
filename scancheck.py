#!/usr/bin/env python

import pickle
import numpy as np
import cv2
import uvc 
from rich import print as rprint

np.set_printoptions(suppress=True, formatter={'float_kind':'{:5.3f}'.format})  

# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html



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
            
class Calibrator(object):
    def __init__(self, sqx=8, sqy=6, width=640, height=480):            
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.sqx = sqx
        self.sqy = sqy
        self.objp = np.zeros((sqx*sqy,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:sqx,0:sqy].T.reshape(-1,2)
 
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
 
        self.width = width
        self.height = height
        self.device = uvc.device_list()[0]
        self.cap = uvc.Capture(self.device["uid"]) 
        self.cap.frame_mode = find_mode(self.cap, width, height, 30)
        self.sufficient_pts = False 
        print(f"frame mode: {self.cap.frame_mode}")
    def calibrate(self):
        while True:
            img = self.cap.get_frame()
            #print (img)
            img = img.bgr
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #gray = img.gray
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.sqx,self.sqy), None)

            # If found, add object points, image points (after refining them)
            waittime = 1
            if ret == True:
                self.objpoints.append(self.objp)


                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
                self.imgpoints.append(corners2)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, (self.sqx,self.sqy), corners2, ret)
                #waittime = 500
        
            cv2.line(img, (int(0),int(self.height/2)), (int(self.width), int(self.height/2)), (0, 0, 255), 1)
            cv2.line(img, (int(self.width/2),int(0)), (int(self.width/2), int(self.height)), (0, 0, 255), 1)
    
            cv2.imshow('img', img)
            if (cv2.waitKey(waittime) == 27):
                break
            numpts = len(self.objpoints)
            rprint(f"now I have {numpts} points", end='\r')
            if numpts > 50:
                print("")
                self.sufficient_pts = True
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
                if ret:
                    self.mtx = mtx
                    self.dist = dist
                    self.rvecs = rvecs
                    self.tvecs = tvecs
                    rprint(f"Successful calibration")
                    rprint(f"Camera matrix: {mtx}")
                    rprint(f"Distortion coefficients: {dist}")
                    caminfo = {"cam_mtx": mtx, "distortion": dist}
                    pfilename = "caminfo.pickle"
                    with open(pfilename, "wb") as f:
                        pickle.dump(caminfo, f)
                    print(f"saved as {pfilename}")
                    #print(f"Rotation vectors: {rvecs}")
                    #print(f"Translation vectors: {tvecs}")
                    
                return
    def demonstrate(self):
        while True:
            img = self.cap.get_frame().bgr
            h, w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
            
            # undistort
            mapx, mapy = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (w,h), 5)
            dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
 
            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            cv2.imshow('dst', dst)
            if (cv2.waitKey(1) == 27):
                break
            
            
        
                
if __name__ == "__main__":
    c = Calibrator()
    c.calibrate()
    c.demonstrate()