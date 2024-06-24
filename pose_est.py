#!/usr/bin/env python

import sys
import os
import time, datetime
import pickle
import math as m

import numpy as np
import cv2
from scipy.spatial.transform import Rotation   

import linear as lin

from rich import print as rp
from rich.pretty import pprint as rpp # yeah you know me

np.set_printoptions(suppress=True, 
                    formatter={'float_kind':'{:6.2f}'.format}) 

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
            
			
			cv2.putText(image, str(markerID),(topLeft[0]+10, topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				.75, (255, 0, 255), 2)
			#print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return image


class PoseError(Exception): pass

fs = lambda a: ','.join([f"{x:6.3f}" for x in np.array(a).ravel()])

class PoseInfo(object):
    def __init__(self, tvec, rvec, heading, roll, pitch, acorners, aids, rejected, centerpos):
        self.position = centerpos
        self.trans_vec = tvec
        self.rot_vec = rvec
        self.heading = heading
        self.roll = roll
        self.pitch = pitch
        self.aruco_corners = acorners
        self.aruco_ids = aids
        self.rejected_points = rejected
    def header(self):
        return("px,py,tvx,tvy,tvz,rvx,rvy,rvz,heading,roll,pitch")
    def __str__(self):
        return f"{fs(self.position)},{fs(self.trans_vec)},{fs(self.rot_vec)},{self.heading:6.3f},{self.roll:6.3f},{self.pitch:6.3f}"

#bi = BallInfo([c1, c2, c3], [dt1, dt2], [ds1, ds1], [v1, v2], a)       
class BallInfo(object):
    def __init__(self, centers, timedeltas, spacedeltas, velocities, acceleration):
        self.time = time.time()
        self.centers = centers
        self.dts = timedeltas
        self.dss = spacedeltas
        self.vs = velocities
        self.accel = acceleration
    def header(self):
        return("time,c1x,c1y,c1z,c2x,c2y,c2z,c3x,c3y,c3z,dt1,dt2,ds1x,ds1y,ds2x,ds2y,v1x,v1y,v2x,v2y,ax,ay")
    def __str__(self):
        return f"{self.time},{fs(self.centers)},{fs(self.dts)},{fs(self.dss)},{fs(self.vs)},{fs(self.accel)}"

    
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

class Recognizer(object):
    def __init__(self, height, width):
        self.draw_valines = False
        self.height = height
        self.width = width
        self.aruco_type = "DICT_4X4_50"
        self.arucoParams =  cv2.aruco.DetectorParameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        with open('caminfo.pickle', 'rb') as f:
            caminfo = pickle.load(f)
            self.cam_mtx = caminfo['cam_mtx']
            self.distortion = caminfo['distortion']        
        with open('corner_info.pickle', 'rb') as f:
            corner_info = pickle.load(f)
            self.aruco_ids = corner_info['ids']
            self.aruco_corners = corner_info['corners']
        #print(f"aruco_ids: {aruco_ids}")
        #print(f"aruco_corners: {aruco_corners}")
        self.ArucoBoard=cv2.aruco.Board(
            self.aruco_corners.astype(np.float32), 
            self.arucoDict, 
            self.aruco_ids)
        self.parameters =  cv2.aruco.DetectorParameters()
        cv2.aruco_dict = self.arucoDict 
        
        self.cb_valid = [False, False, False]
        self.circle_buf = [[], [], []]
        self.cb_times = [time.time(), time.time(), time.time()]
        
        self.ball_info = None # data log of ball
        self.pose_info = None # data log of platform
        self.have_pose = False
        self.have_ball = False
        self.have_estimate = False
        
        self.log_data = []
        self.logging = False
        self.csv_stemname = "Pose_est"
        
    def start_logging(self):
        if not self.logging:
            rp("[yellow on red]Begin Logging")
            self.log_data = []
            self.logging = True
            
    def stop_logging(self):
        if self.logging:
            rp(f"[yellow on red]Stop Logging have {len(self.log_data)} lines")
            if len(self.log_data) > 0:
                fn = f"{self.csv_stemname}-{datetime.datetime.now().isoformat()}.csv"
                d0 = self.log_data[0] 
                header = ','.join([d.header() for d in d0]) 
                #header = f"{self.log_data[0][0].header()},{self.log_data[0][1].header()}"
                with open(fn, "w") as cvsfile:
                    print(header, file=cvsfile)
                    #for bi, pi in self.log_data:
                    for fields in self.log_data:
                        line = ','.join([str(field) for field in fields])
                        #print(f"{str(bi)},{str(pi)}", file=cvsfile)
                        print(line, file=cvsfile)
                rp(f"[yellow on red]Wrote log to {fn}")
        self.logging = False
        
            
            
    def log(self):
        #print(ball_info)
        #print(pose_info)
        if self.logging:
            self.log_data.append([self.ball_info, self.pose_info])
    
    
    def pose_estimation(self, frame):
    
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = cv2.aruco.ArucoDetector(cv2.aruco_dict, self.parameters)
        #corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
        #gray, cv2.aruco_dict,parameters=self.parameters)
        
        corners, ids, rejected_img_points = detector.detectMarkers(gray)
        corners, ids, rejected_img_points, _ = detector.refineDetectedMarkers(
            gray, 
            self.ArucoBoard, 
            corners, 
            ids, rejected_img_points, self.cam_mtx, self.distortion)
        try:
            objPoints, imgPoints = self.ArucoBoard.matchImagePoints(corners, ids)
            for i in range(objPoints.shape[0]):
                #print(f"{i} {objPoints[i]}, {imgPoints[i]}")
                pass
            #print(f"objPoints: {objPoints.shape}, imgPoints: {imgPoints.shape}")
            ret,rvecs,tvecs = cv2.solvePnP(objPoints, imgPoints, self.cam_mtx, self.distortion)
            if ret:
                rvecs,tvecs = cv2.solvePnPRefineLM(
                    objPoints, imgPoints, self.cam_mtx, self.distortion, rvecs, tvecs)
                rod, jack = cv2.Rodrigues(rvecs)
                rodmat =  Rotation.from_matrix(rod)
                heading, roll, pitch  = rodmat.as_euler("zyx",degrees=True)
            
                pitch = 180 - (pitch % 360)
                heading = 90 - (heading % 360)
                #roll = 360-(roll%360)
            
                testpts = np.float32([[91.5, 91.5, 0],])
                imgpts, jac = cv2.projectPoints(testpts,rvecs, tvecs, self.cam_mtx, self.distortion)
                center = imgpts[0][0]
                #print(f"center: {center}, angles: (heading, pitch, roll): ({heading:5.2f}, {pitch:5.2f}, {roll:5.2f})")
                self.pose_info =  PoseInfo(tvecs, rvecs, heading, roll, pitch, corners, ids, rejected_img_points, center)
                self.dxp = -int(self.width/2 - self.pose_info.position[0])              
                self.dyp = int(self.height/2 - self.pose_info.position[1])
                self.find_playfield_mask(frame)
                return self.pose_info
            else:
                raise PoseError("solvePnP failed")  
        except cv2.error as e:
            raise PoseError(f"Match failed: {e}")
    
    def hudtext(self, frame):
        # (origin) (width height)
        cv2.rectangle(frame, (0, 100), (130, 325), (0, 0, 0), -1)
        fontspec = (cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
        
        cv2.putText(frame,f"Roll:",(0, 120),*fontspec)
        cv2.putText(frame,f"Pitch:",(0, 140),*fontspec)
        cv2.putText(frame,f"Heading:",(0, 160),*fontspec)
        cv2.putText(frame,f"CX:",(0, 180),*fontspec)
        cv2.putText(frame,f"CY:",(0, 200),*fontspec)
        cv2.putText(frame,f"CZ:",(0, 220),*fontspec)                          
        cv2.putText(frame,f"dxp:",(0, 248),*fontspec)                          
        cv2.putText(frame,f"dyp:",(0, 268),*fontspec)                          
        
        x0 = 75
        cv2.putText(frame,f"{self.pose_info.roll:5.1f}",(x0, 120),*fontspec)
        cv2.putText(frame,f"{self.pose_info.pitch:5.1f}",(x0, 140),*fontspec)
        cv2.putText(frame,f"{self.pose_info.heading:5.1f}",(x0, 160),*fontspec)
        #print(f"self.trans_vec: {self.pose_info.trans_vec.T[0]}")
        tv = self.pose_info.trans_vec.T[0]
        cv2.putText(frame,f"{tv[0]:5.1f}",(x0, 180),*fontspec)
        cv2.putText(frame,f"{tv[1]:5.1f}",(x0, 200),*fontspec)
        cv2.putText(frame,f"{tv[2]:5.1f}",(x0, 220),*fontspec)             
        cv2.putText(frame,f"{self.dxp:4}",(x0, 248),*fontspec)
        cv2.putText(frame,f"{self.dyp:4}",(x0, 268),*fontspec)

        if self.have_ball == True:
            #ballstring1 = "BALL FOUND"
            #ballstring2 = f"{self.ball_pos}"
            ballstring1 = "BALL FOUND"
            ballstring2 = f"BDP: {(self.ball_dxp, self.ball_dyp)}"
        else:
            ballstring1 = "NO BALL"
            ballstring2 = "(-,-)"
            
        cv2.putText(frame,ballstring1,(0, 295),*fontspec)
        cv2.putText(frame,ballstring2,(0, 315),*fontspec)
        
    def find_playfield_mask(self, frame):
        steps = np.arange(0, 36, 1)*10
        r = 166/2
        x = r*np.cos(np.radians(steps))+91.5
        y = r*np.sin(np.radians(steps))+91.5
        z = 0 * steps
        testpts = np.stack([x, y, z], axis=1)
        imgpts, jac = cv2.projectPoints(
            testpts, 
            self.pose_info.rot_vec, 
            self.pose_info.trans_vec, self.cam_mtx, self.distortion)
        
        imgpts2 = [ip[0] for ip in imgpts]
        
        self.playfield_circle = np.intp([imgpts2])
        self.playfield_mask = np.zeros((self.height, self.width), np.uint8)
        
        cv2.polylines(frame, self.playfield_circle, True, (0, 255, 255), 2)
        cv2.fillPoly(self.playfield_mask, self.playfield_circle, (1))  
        
             
    def drawhud(self, frame, pose_info):
        # FIXME: might have scrolling graph of PID and Kalman stats at bottom
        # Terminator up this shit
        # Also make version in Terminator Red
        # Draw grid on checkerboard
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
                pose_info.trans_vec, self.cam_mtx, self.distortion)
            
            try:
                for i, pt in enumerate(imgpts):
                    cv2.circle(frame, np.intp(pt[0]), 3, (255, 255, 255), 2)
                cv2.line(frame, np.intp(imgpts[5][0]), np.intp(imgpts[6][0]), (0, 255, 0), 1)
                cv2.line(frame, np.intp(imgpts[7][0]), np.intp(imgpts[8][0]), (0, 255, 0), 1)
                cv2.polylines(frame, self.playfield_circle, True, (0, 255, 255), 2)
                self.hudtext(frame)

            except cv2.error as e:
                print(f"oops: {e}")
            
        # Y is red, X is green, Z is blue
        cv2.drawFrameAxes(
            frame, 
            self.cam_mtx, 
            self.distortion, 
            pose_info.rot_vec, 
            pose_info.trans_vec, 
            15, 2)
              
        if len(pose_info.aruco_corners) > 0:
            aruco_display(
                pose_info.aruco_corners, 
                pose_info.aruco_ids, 
                pose_info.rejected_points, 
                frame)
        #return mask

    def detect_ball(self, frame, rblur):
        #global circle_buf, cb_valid, cb_times
    
        # look for ball
        circles = cv2.HoughCircles(rblur,cv2.HOUGH_GRADIENT,1,20,param1=130,param2=30,minRadius=55,maxRadius=90)
        #circles = None
        if not np.any(circles):
            self.cb_times.append(time.time())
            self.circle_buf.append([])
            self.cb_valid.append(False)
            
            self.cb_valid.pop(0)
            self.cb_times.pop(0)
            self.circle_buf.pop(0)
            return frame, rblur
        #if np.any(circles):
        else:
            circles = np.uint16(np.around(circles))
            if len(circles) == 1: 
                self.circle_buf.append(circles[0][0])
                self.cb_valid.append(True)
                self.have_ball = True
            else:
                self.circle_buf.append([])
                self.cb_valid.append(False)
            self.cb_times.append(time.time())
            self.circle_buf.pop(0)
            self.cb_valid.pop(0)
            self.cb_times.pop(0)
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
                 self.ball_pos = (x, y)
                 self.ball_dxp = -int(self.width/2 - self.ball_pos[0])              
                 self.ball_dyp = int(self.height/2 - self.ball_pos[1])
        if sum(self.cb_valid) == 3:
            #print("possible solution found!")
            try:
                c1, c2, c3 = [np.array(x, dtype="float64") for x in self.circle_buf]
                dt1 = self.cb_times[1]- self.cb_times[0]
                dt2 = self.cb_times[2]- self.cb_times[1] 
                ds1 = (c2 - c1)[:2]
                ds2 = (c3 - c2)[:2]
                v1 = ds1/dt1
                v2 = ds2/dt2
                dv = v2-v1
                a = dv/dt2
                self.ball_info = BallInfo([c1, c2, c3], [dt1, dt2], [ds1, ds1], [v1, v2], a)
                #print(f"c1: {np.intp(c1)}, c2: {np.intp(c2)}, c3: {np.intp(c3)}")
                #print(f"v1: {v1}, v2: {v2}, dv: {dv}")
                #print(f"dt1: {dt1:3.3f}, dt2: {dt2:3.3f}, ds1: {ds1}, ds2: {ds2}")
                v1s = m.sqrt(abs(lin.dot(*v1)))
                v2s = m.sqrt(abs(lin.dot(*v2)))
                a_s = m.sqrt(abs(lin.dot(*a)))
            
                #print(f"v1: {v1s:8.2f}, v2: {v2s:8.2f}, a: {a_s:8.2f}", end='\n')
                self.have_estimate = True
                cv2.line(frame, np.intp((c1[0], c1[1])), np.intp((c2[0], c2[1])), (255, 255, 0), 2)
                cv2.line(frame, np.intp((c2[0], c2[1])), np.intp((c3[0], c3[1])), (255, 255, 0), 2)
                
                # FIXME predict next step using kalman filter here
                # state space physics model of ball and input
                # feed-forward
                #print(f"c3: {c3}  v2: {v2}")
                vline = c3[:2] + v2
                aline = c3[:2] + a
                if self.draw_valines:
                    cv2.line(frame, np.intp((c3[0], c3[1])), np.intp((vline[0], vline[1])), (0, 255, 0), 2)
                    cv2.line(frame, np.intp((c3[0], c3[1])), np.intp((aline[0], aline[1])), (0, 0, 255), 2)

            except Exception as e:
                pass
                print("Did I divide by zero?:  ", e)
        return frame, rblur
    def recognize(self, frame):
        self.have_pose = False
        self.have_ball = False
        self.have_estimate = False
        # save for later ball-seeking
        rin = frame[:, :, 2].copy()
        try:
            pose_info = self.pose_estimation(frame)
            self.have_pose = True
            rin = rin * self.playfield_mask
            rblur = cv2.medianBlur(rin,5)
            
            self.output, rblur = self.detect_ball(frame, rblur)
            self.red = cv2.cvtColor(rblur,cv2.COLOR_GRAY2BGR)
            cv2.line(frame, np.intp((0,self.height/2)), np.intp((self.width, self.height/2)), (0, 0, 255), 1)
            cv2.line(frame, np.intp((self.width/2,0)), np.intp((self.width/2,self.height)), (0, 0, 255), 1)
            self.drawhud(frame, pose_info)

                
            return pose_info
        except Exception as e:
            print(f"recognizer error: {e}")
            #raise

def go():    
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
    

    rec = Recognizer(height, width)
    
    # FIXME create state machine to pan roll and pitch and measure slew rate
    # strategy for balancing:  first center machine
    # get true gravity vector from brick
    # once platform centered, center ball on platform
    # should expose 6-DOF api on stuart
    while cap.isOpened():
        ret, img = cap.read()
        if ret:
            rec.recognize(img)
            # self.have_estimate
            if rec.have_estimate:
                print("have estimate")
                rec.log()
                print(f"logged estimate have {len(rec.log_data)} lines")
            else:
                print("no estimate")
                

            canvas = np.zeros((height, width*2, 3), np.uint8)
            canvas[0:480, 0:640] = rec.output
            canvas[0:480, 640:1280] = rec.red
            cv2.imshow('Estimated Pose', canvas)

        key = cv2.waitKey(1)
        if key == 27:
            break
        elif key == ord('l'):
            rec.start_logging()
        elif key == ord('s'):
            rec.stop_logging()
        
        #key = cv2.waitKey(1) & 0xFF
        #if key == ord('q'):
        #    break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    go()
