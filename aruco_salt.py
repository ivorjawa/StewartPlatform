#!/usr/bin/env python

import pickle
import cv2
#from object_detector import *
import numpy as np

# Load Aruco detector
#parameters = cv2.aruco.DetectorParameters_create()
#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters =  cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
with open('caminfo.pickle', 'rb') as f:
    caminfo = pickle.load(f)
    mtx = caminfo['cam_mtx']
    dist = caminfo['distortion']



# Load Cap
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    _, img = cap.read()

    # Get Aruco marker
    corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    if corners:
        print(ids)
        # Draw polygon around the marker
        int_corners = np.intp(corners)
        cv2.polylines(img, int_corners, True, (0, 255, 0), 1)

        # Aruco Perimeter
        aruco_perimeter = cv2.arcLength(corners[0], True)

        # Pixel to cm ratio
        pixel_cm_ratio = aruco_perimeter / 3





    cv2.imshow("Image", img)
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()