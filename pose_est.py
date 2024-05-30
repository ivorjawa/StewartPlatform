#!/usr/bin/env python

import sys
import os
import time
import pickle
import math as m

import numpy as np
import cv2
from scipy.spatial.transform import Rotation   

import linear as lin

from rich import print as rp
from rich.pretty import pprint as rpp # yeah you know me

np.set_printoptions(suppress=True, 
                    formatter={'float_kind':'{:5.2f}'.format}) 

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):
    
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			cv2.circle(image, topLeft, 4, (255, 0, 255), -1)
            
			
			cv2.putText(image, str(markerID),(topLeft[0]+30, topLeft[1] - 30), cv2.FONT_HERSHEY_SIMPLEX,
				1.5, (255, 0, 255), 2)
			#print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return image


class PoseError(Exception): pass

class PoseInfo(object):
    def __init__(self, tvec, rvec, heading, roll, pitch, acorners, aids, rejected):
        self.trans_vec = tvec
        self.rot_vec = rvec
        self.heading = heading
        self.roll = roll
        self.pitch = pitch
        self.aruco_corners = acorners
        self.aruco_ids = aids
        self.rejected_points = rejected
        

"""
https://docs.opencv.org/4.x/db/da9/tutorial_aruco_board_detection.html

// Get object and image points for the solvePnP function
 cv::Mat objPoints, imgPoints;
 board.matchImagePoints(corners, ids, objPoints, imgPoints);

// Find pose
 cv::solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec);
        
           #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues
            #https://www.reddit.com/r/opencv/comments/kczhoc/question_solvepnp_rvecs_what_do_they_mean/
            #https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
            #https://stackoverflow.com/questions/54616049/converting-a-rotation-matrix-to-euler-angles-and-back-special-case
"""
        
def pose_estimation(frame, ArucoBoard, aruco_dict_type, matrix_coefficients, distortion_coefficients, height, width):
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    parameters =  cv2.aruco.DetectorParameters()
    cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    
    try:
        objPoints, imgPoints = ArucoBoard.matchImagePoints(corners, ids)
        for i in range(objPoints.shape[0]):
            #print(f"{i} {objPoints[i]}, {imgPoints[i]}")
            pass
        #print(f"objPoints: {objPoints.shape}, imgPoints: {imgPoints.shape}")
        ret,rvecs, tvecs = cv2.solvePnP(objPoints, imgPoints, matrix_coefficients, distortion_coefficients)
        if ret:
            rod, jack = cv2.Rodrigues(rvecs)

           
            rodmat =  Rotation.from_matrix(rod)
            heading, roll, pitch  = rodmat.as_euler("zyx",degrees=True)
            
            pitch = 180 - (pitch % 360)
            heading = 90 - (heading % 360)
            #roll = 360-(roll%360)
            
            print(f"angles: (heading, pitch, roll): ({heading:5.2f}, {pitch:5.2f}, {roll:5.2f})")
            return PoseInfo(tvecs, rvecs, heading, roll, pitch, corners, ids, rejected_img_points)
        else:
            raise PoseError("solvePnP failed")  
    except cv2.error as e:
        raise PoseError(f"Match failed: {e}")
    
def drawhud(frame, pose_info, matrix_coefficients, distortion_coefficients, height, width):
    if not (pose_info.trans_vec is None):
        testpts = np.float32([
            [183, 0, 0], 
            [0,183,0], 
            [0,0,0], 
            [183,183,0], 
            [91.5, 91.5, 0],
            [0, 91.5, 0],
            [183, 91.5, 0],
            [91.5, 0, 0],
            [91.5, 183, 0],
        ]).reshape(-1,3)
        imgpts, jac = cv2.projectPoints(
            testpts, 
            pose_info.rot_vec, 
            pose_info.trans_vec, matrix_coefficients, distortion_coefficients)
            
        try:
            for i, pt in enumerate(imgpts):
                cv2.circle(frame, np.intp(pt[0]), 3, (255, 255, 255), 2)
            cv2.line(frame, np.intp(imgpts[5][0]), np.intp(imgpts[6][0]), (0, 255, 0), 1)
            cv2.line(frame, np.intp(imgpts[7][0]), np.intp(imgpts[8][0]), (0, 255, 0), 1)
            
            steps = np.arange(0, 36, 1)*10
            r = 166/2
            x = r*np.cos(np.radians(steps))+91.5
            y = r*np.sin(np.radians(steps))+91.5
            z = 0 * steps
            testpts = np.stack([x, y, z], axis=1)
            imgpts, jac = cv2.projectPoints(
                testpts, 
                pose_info.rot_vec, 
                pose_info.trans_vec, matrix_coefficients, distortion_coefficients)
            imgpts2 = [ip[0] for ip in imgpts]
            cv2.polylines(frame, np.intp([imgpts2]), True, (0, 255, 255), 2)
            mask = np.zeros((height, width), np.uint8)
            #cv2.circle(mask,(xav,yav),rav+30,1,-1)
            cv2.fillPoly(mask, np.intp([imgpts2]), (1))

        except cv2.error as e:
            print(f"oops: {e}")
            
    # Y is red, X is green, Z is blue
    cv2.drawFrameAxes(
        frame, 
        matrix_coefficients, 
        distortion_coefficients, 
        pose_info.rot_vec, 
        pose_info.trans_vec, 
        15, 2)
              
    if len(pose_info.aruco_corners) > 0:
        aruco_display(
            pose_info.aruco_corners, 
            pose_info.aruco_ids, 
            pose_info.rejected_points, 
            frame)
    return mask

cb_valid = [False, False, False]
circle_buf = [[], [], []]
cb_times = [time.time(), time.time(), time.time()]

def detect_ball(frame, rblur):
    global circle_buf, cb_valid, cb_times
    
    # look for ball
    circles = cv2.HoughCircles(rblur,cv2.HOUGH_GRADIENT,1,20,param1=130,param2=30,minRadius=55,maxRadius=90)
    #circles = None
    if np.any(circles):
        circles = np.uint16(np.around(circles))
        if len(circles) == 1: 
            circle_buf.append(circles[0][0])
            cb_valid.append(True)
        else:
            circle_buf.append([])
            cb_valid.append(False)
        cb_times.append(time.time())
        circle_buf.pop(0)
        cb_valid.pop(0)
        cb_times.pop(0)
        for i in circles[0,:]:
             # draw the outer circle
             x = i[0]
             y = i[1]
             r = i[2]
             #print(f"found circle with radius: {r}")
             # draw circle
             cv2.circle(frame,(x, y),r,(0,255,0),2)
             # draw the center of the circle
             cv2.circle(frame,(x,y),2,(255,255,255),3)
    if sum(cb_valid) == 3:
        print("possible solution found!")
        try:
            c1, c2, c3 = [np.array(x, dtype="float64") for x in circle_buf]
            dt1 = cb_times[1]- cb_times[0]
            dt2 = cb_times[2]- cb_times[1] 
            ds1 = (c2 - c1)[:2]
            ds2 = (c3 - c2)[:2]
            v1 = ds1/dt1
            v2 = ds2/dt2
            dv = v2-v1
            a = dv/dt2
            print(f"c1: {np.intp(c1)}, c2: {np.intp(c2)}, c3: {np.intp(c3)}")
            print(f"v1: {v1}, v2: {v2}, dv: {dv}")
            print(f"dt1: {dt1:3.3f}, dt2: {dt2:3.3f}, ds1: {ds1}, ds2: {ds2}")
            v1s = m.sqrt(abs(lin.dot(*v1)))
            #print("x")
            v2s = m.sqrt(abs(lin.dot(*v2)))
            #print(f"a: {a}")
            a_s = m.sqrt(abs(lin.dot(*a)))
            
            print(f"v1: {v1s:8.2f}, v2: {v2s:8.2f}, a: {a_s:8.2f}", end='\n')
            cv2.line(frame, np.intp((c1[0], c1[1])), np.intp((c2[0], c2[1])), (255, 255, 0), 2)
            cv2.line(frame, np.intp((c2[0], c2[1])), np.intp((c3[0], c3[1])), (255, 255, 0), 2)
        except Exception as e:
            pass
            print("Did I divide by zero?:  ", e)
    return frame, rblur

def go():
    aruco_type = "DICT_4X4_50"
    arucoParams =  cv2.aruco.DetectorParameters()
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    with open('caminfo.pickle', 'rb') as f:
        caminfo = pickle.load(f)
        cam_mtx = caminfo['cam_mtx']
        distortion = caminfo['distortion']
        
        
    with open('corner_info.pickle', 'rb') as f:
        corner_info = pickle.load(f)
        aruco_ids = corner_info['ids']
        aruco_corners = corner_info['corners']
    #print(f"aruco_ids: {aruco_ids}")
    #print(f"aruco_corners: {aruco_corners}")
    ArucoBoard=cv2.aruco.Board(aruco_corners.astype(np.float32), arucoDict, aruco_ids)

    
    cap = cv2.VideoCapture(0)

    width = 640
    height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    os.system("./uvc-util -I 0 -s auto-focus=0")
    os.system("./uvc-util -I 0 -s focus-abs=0")
    os.system("./uvc-util -I 0 -s auto-exposure-mode=1")
    os.system("./uvc-util -I 0 -s exposure-time-abs=150")
    
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
    #cap.set(cv2.CAP_PROP_FOCUS, 0) 

    while cap.isOpened():
    
        ret, img = cap.read()
        
        # save for later ball-seeking
        rin = img[:, :, 2].copy()
        rblur = cv2.medianBlur(rin,5)
    
        #print(f"canvas shape: {canvas.shape}")
        try:
            pose_info = pose_estimation(img, ArucoBoard, ARUCO_DICT[aruco_type], cam_mtx, distortion, height, width)
            mask = drawhud(img, pose_info, cam_mtx, distortion, height, width)
            rin = rin * mask
            rblur = cv2.medianBlur(rin,5)
            output, rblur = detect_ball(img, rblur)

            cv2.line(output, (int(0),int(height/2)), (int(width), int(height/2)), (0, 0, 255), 1)
            cv2.line(output, (int(width/2),int(0)), (int(width/2), int(height)), (0, 0, 255), 1)
    
            red = cv2.cvtColor(rblur,cv2.COLOR_GRAY2BGR)
    
            canvas = np.zeros((height, width*2, 3), np.uint8)
            canvas[0:480, 0:640] = output
            canvas[0:480, 640:1280] = red
            cv2.imshow('Estimated Pose', canvas)
        except Exception as e:
            print(e)
        if cv2.waitKey(1) == 27:
            break
        #key = cv2.waitKey(1) & 0xFF
        #if key == ord('q'):
        #    break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    go()
