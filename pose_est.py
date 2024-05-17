#!/usr/bin/env python

import sys
import os
import time
import pickle
import math as m

import numpy as np
import cv2

import linear as lin

np.set_printoptions(suppress=True, 
                    formatter={'float_kind':'{:3.3f}'.format}) 

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


def drawg(img, corners, imgpts):
 corner = tuple(corners[0].ravel())
 img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
 img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
 img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
 return img
 
vdegrees = lambda v: np.array([np.degrees(x) for x in v])

cb_valid = [False, False, False]
circle_buf = [[], [], []]
cb_times = [time.time(), time.time(), time.time()]

def pose_estimation(frame, ArucoBoard, aruco_dict_type, matrix_coefficients, distortion_coefficients, width, height):
    global circle_buf, cb_valid, cb_times

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    #parameters = cv2.aruco.DetectorParameters_create()
    parameters =  cv2.aruco.DetectorParameters()
    cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)#,
        #cameraMatrix=matrix_coefficients,
        #distCoeff=distortion_coefficients)

    """

        // Get object and image points for the solvePnP function
         cv::Mat objPoints, imgPoints;
         board.matchImagePoints(corners, ids, objPoints, imgPoints);
        
        // Find pose
         cv::solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec);
        """
    try:
        objPoints, imgPoints = ArucoBoard.matchImagePoints(corners, ids)
        for i in range(objPoints.shape[0]):
            print(f"{i} {objPoints[i]}, {imgPoints[i]}")
        print(f"objPoints: {objPoints.shape}, imgPoints: {imgPoints.shape}")
        ret,rvecs, tvecs = cv2.solvePnP(objPoints, imgPoints, matrix_coefficients, distortion_coefficients) 
        if ret:
            r,p,y = [m.degrees(x) for x in rvecs]
            x,y,z = tvecs # this is the origin
            x = x[0]
            y = y[0]
            z = z[0]
            #print(f"x: {x}")
            print(f"solvePnP orientation: ({r:4.3f}, {p:4.3f}, {y:4.3f}) offset: ({x:4.3f}, {y:4.3f}, {z:4.3f})") #  tvecs: {tvecs[:3]}
            
            testpts = np.float32([[x,y,z], [0,0,z], [400,400,z]]).reshape(-1,3)
            imgpts, jac = cv2.projectPoints(testpts, rvecs, tvecs, matrix_coefficients, distortion_coefficients)
            for i, pt in enumerate(imgpts):
                print(f"testpt: {testpts[i]} imgpt: {imgpts[i]}")
                print(f"pt: {pt[0]} {frame.shape}")
                x, y = pt[0]
                # this is drawing on pre-processed frame
                cv2.circle(frame, np.intp((x, y)), 10, (255, 255, 255), -1)
                
                
        else:
            print("solvePnP failed")  
    except cv2.error as e:
        print(f"Match failed: {e}")
         
    tags = {
        'red27': 27,
        'blue03': 3,
        'green15': 15,
        'bubble': 42,
    }
    rtags = {
        27: 'red:27', 
        3: 'blue:3',
        15: 'green:15',
        42: 'bubble:42',
    }  
    
    # save for later ball-seeking
    rin = frame[:, :, 2].copy()
    rblur = cv2.medianBlur(rin,5)
      
    if len(corners) > 0:
        mask_circs = []
        mask_rads = [] # 145 - 235
        aruco_display(corners, ids, rejected_img_points, frame)
        #print(f"detected ids: {ids}")
        for i in range(0, len(ids)):
            tid = ids[i][0]

            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            
            #print(f"tid: {tid} corners[{i}]: {corners[i]}")
            #criteria = (cv2.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            #corners2 = cv2.cornerSubPix(frame,corners,(11,11),(-1,-1),criteria)
            #ret,rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            #if not ret:
            #    print("solvePnP failed"
            #    continue
            #cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Y is red, X is green, Z is blue
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.06, 1)
 
            axis = np.float32([[.03,0,0], [0,.03,0], [0,0,-.03]]).reshape(-1,3)
            imgpts, jac = cv2.projectPoints(axis, rvec, tvec, matrix_coefficients, distortion_coefficients)
            #print(f"imgpts.shape: {imgpts.shape}, imgpts: {imgpts}  ")
            
            #print(f"axis: {axis}, axis.shape: {axis.shape}")
            #for j in range(len(axis)):
            #    print(f"point: {axis[j]} projected: {imgpts[j]}")
            #cv2.line(frame, np.intp(imgpts[0][0]), np.intp(imgpts[1][0]), (0, 255, 255), 2)
            #cv2.line(frame, np.intp(imgpts[1][0]), np.intp(imgpts[2][0]), (255, 0, 255), 2)
            #cv2.line(frame, np.intp(imgpts[2][0]), np.intp(imgpts[0][0]), (255, 255, 0), 2)
            
            crlen = 0.162
            if tid == 42: # bubble, ie false
                #centeray = np.float32([[0, 0, 0], [0, crlen, 0]]).reshape(-1,3)
                continue
            else:
                centeray = np.float32([[0, 0, 0], [-crlen, 0, 0]]).reshape(-1,3)
            imgpts, jac = cv2.projectPoints(centeray, rvec, tvec, matrix_coefficients, distortion_coefficients)
            cv2.line(frame, np.intp(imgpts[0][0]), np.intp(imgpts[1][0]), (0, 255, 255), 2)
            crp0 = lin.vector(imgpts[0][0])
            crp1 = lin.vector(imgpts[1][0])
            
            crplen = int(lin.vmag(crp1-crp0))
            #print(f"centeray projected length is {crplen}")
            
            if (crplen > 145) and (crplen < 235):
                mask_circs.append(np.intp(imgpts[1][0]))
                mask_rads.append(crplen)
            cv2.circle(frame,np.intp(imgpts[1][0]),crplen+30,(0,255,255),2)
            for j in range(len(centeray)):
                pass
                #print(f"cpoint: {centeray[j]} projected: {imgpts[j]}")
            #print("jac:  ", jac.shape)
            try:
                #print(f"id: {rtags[tid]}  x: {frame[0]:3.3f}, y: {frame[1]:3.3f}, z: {frame[2]:3.3f}")
                rvd = vdegrees(rvec[0][0])
                #tvd = vdegrees(tvec[0][0])
                tvd = tvec[0][0] # not radians
                
                #print(f"id: {rtags[tid]}  rvec: {rvd}, tvec: {tvd}")
            except KeyError as e:
                print(f"unknown tag {tid}")
        if len(mask_circs ) > 0:
            #print (f"mask centers: {mask_circs}")
            #print (f"mask radii: {mask_rads}")
            cx = []
            cy = []
            for c in mask_circs:
                cx.append(c[0])
                cy.append(c[1])
            #cx, cy = mas_circs
            xsum = np.sum(cx)
            ysum = np.sum(cy)
            rsum = np.sum(mask_rads)
            scale = 1.0*len(mask_circs)
            xav = np.intp(xsum/scale)
            yav = np.intp(ysum/scale)
            rav = np.intp(rsum/scale)
            #(cx, cy) = np.intp((np.sum(mask_circs)/(1.0*len(mask_circs))))
            #cr = np.intp(np.sum(mask_rads)/(1.0*len(mask_rads)))
            #print(f"cx, cy: ({xav},{yav}), Radius: {rav}")
            mask = np.zeros((height, width), np.uint8)
            cv2.circle(mask,(xav,yav),rav+30,1,-1)
            rin = rin * mask
            rblur = cv2.medianBlur(rin,5)
            
            
            
            
                
        # look for ball

        #red = cv2.cvtColor(rblur,cv2.COLOR_GRAY2BGR)
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

    #arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    #arucoParams = cv2.aruco.DetectorParameters_create()
    #intrinsic_camera = np.array(((933.15867, 0, 657.59),(0,933.1586, 400.36993),(0,0,1)))
    #distortion = np.array((-0.43948,0.18514,0,0))

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
    print(f"aruco_ids: {aruco_ids}")
    print(f"aruco_corners: {aruco_corners}")
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
    
        canvas = np.zeros((height, width*2, 3), np.uint8)
    
        output, rblur = pose_estimation(img, ArucoBoard, ARUCO_DICT[aruco_type], cam_mtx, distortion, width, height)

        cv2.line(output, (int(0),int(height/2)), (int(width), int(height/2)), (0, 0, 255), 1)
        cv2.line(output, (int(width/2),int(0)), (int(width/2), int(height)), (0, 0, 255), 1)
    
        red = cv2.cvtColor(rblur,cv2.COLOR_GRAY2BGR)
    
        canvas[0:480, 0:640] = output
        canvas[0:480, 640:1280] = red
        cv2.imshow('Estimated Pose', canvas)

        if cv2.waitKey(1) == 27:
            break
        #key = cv2.waitKey(1) & 0xFF
        #if key == ord('q'):
        #    break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    go()
