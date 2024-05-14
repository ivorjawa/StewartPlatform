#!/usr/bin/env python

import sys
import time
import math as m

from rich import print as rp

import numpy as np
import cv2
import imutils

import linear as lin

tags = {
    'red27': 27,
    'blue03': 3,
    'green15': 15,
    'bubble': 42,
}

c = lambda b, g, r: tuple([int(b), int(g), int(r)])
red = c(0, 0, 255)
green = c(0, 255, 0)
yellow = c(0, 255, 255)
white = c(255, 255, 255)
purple = c(255, 0, 255)
blue = c(255, 0, 0)
orange = c(0, 165, 255)

def rot2deg(deg, pt):
    return lin.rotate(lin.zaxis, m.radians(deg), np.array(pt))[:2]
    
#rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) -> img
#cv2.circle(frame,(x,y),2,(255,255,255),3)
# circle(img, center, radius, color[, thickness[, lineType[, shift]]]) -> img

	

def axismin(axis, points):
    pa = [p[axis] for p in points]
    return np.min(pa)
    
def axismax(axis, points):
    pa = [p[axis] for p in points]
    return np.max(pa)
    
def galvatron():
    width = 800
    height = 800
    scale = 700/183
    xo = 50
    yo = 50
    xc = int(width/2)
    yc = int(height/2)
    origin = (xo, yo)
    center = (xc, yc)
    ora = np.array(origin)
    cea = np.array(center)
    
    print(f"scale: {scale} red: {red}")
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("output", width, height) 
    canvas = np.zeros((height, width, 3), np.uint8)
    
    red27 = cv2.imread("aruco/markers/aruco_red27_27.png")
    green15 = cv2.imread("aruco/markers/aruco_green15_15.png")
    blue03 = cv2.imread("aruco/markers/aruco_blue03_3.png")
    arucos = [red27, green15, blue03]
    aheight, awidth = red27.shape[:2]
    print(f"red27.shape: {red27.shape}")

    while 1:
        # crosshairs
        cv2.line(canvas, np.intp((0, height/2)), np.intp((width, height/2)), red, 1)
        cv2.line(canvas, np.intp((width/2, 0)), np.intp((width/2, height)), red, 1)
        
        #plate
        pl = 183*scale
        cv2.rectangle(canvas, origin, np.intp((xo+pl, yo+pl)), yellow)
        
        # circles
        ocr = (166/2)*scale # outer
        cv2.circle(canvas, center, int(ocr), yellow)
        
        icr = (138/2)*scale # inner
        cv2.circle(canvas, center, int(icr), yellow)
        
        ccr = (159.3/2)*scale # little circles center
        cv2.circle(canvas, center, int(ccr), red)
        
        hcr = (4.77/2)*scale # lego pin socket
        hv = lin.vector(ccr, 0, 0)
        
        for i in range(4): # secures big paper square to platform
            hvr = rot2deg(i*90, hv)+cea
            cv2.circle(canvas, np.intp(hvr), int(hcr), green)
            
        for i in range(3): # secures ring to gear
            hvr = rot2deg(i*120+30, hv)+cea
            cv2.circle(canvas, np.intp(hvr), int(hcr), red)
            
        tscr = (ocr - icr)/2 + icr # target square circle radius
        cv2.circle(canvas, center, int(tscr), white)
        
        
        tsr = (13/2)*scale # boarder exactly 13mm on a side for printed ArUco squares
        tsir = (10/2)*scale # with the squares themselves exactly 10cm
        
        tsv = lin.vector(0, tscr, 0) # starts offset 90deg
        print(f"target square radius  {tsr}")
        
        tvx = lin.vector(tsr, 0, 0)
        tvy  = lin.vector(0, tsr, 0)
        # ArUco is upside down when red27 is up
        ur = (tsv+tvx-tvy)
        lr = (tsv+tvx+tvy)
        ul = (tsv-tvx-tvy)
        ll = (tsv-tvx+tvy)
        
        for i in range(3):
            tang = i*120
            print(f"target angle: {tang}")
            tscrv = lin.vector(0, tscr, 0)
            tscenter = rot2deg(tang, tscrv)+cea
            print(f"tscenter:{lin.fv(tscenter)}")
            ulp = rot2deg(tang, ul)+cea
            llp = rot2deg(tang, ll)+cea
            urp = rot2deg(tang, ur)+cea
            lrp = rot2deg(tang, lr)+cea
            tpoints = [ulp, llp, lrp, urp]
            txmin = axismin(0, tpoints)
            txmax = axismax(0, tpoints)
            tymin = axismin(1, tpoints)
            tymax = axismax(1, tpoints)
            twidth = txmax - txmin
            theight = tymax - tymin
            print(f"txmin: {txmin}, txmax: {txmax}, tymin: {tymin}, tymax: {tymax}, twidth: {twidth}, theight: {theight}")

            
            acenter = (awidth/2, aheight/2)
            print(f"acenter: {acenter}")
            aruco = arucos[i]
            aruco_s = cv2.resize(aruco, np.intp((tsir*2, tsir*2)))
            print(f"aruco_s.shape: {aruco_s.shape}")
            
            #aM = cv2.getRotationMatrix2D(acenter, tang, 1.0)
            #rotated = cv2.warpAffine(aruco_s, aM, np.intp((tsr*2, tsr*2)))
            #rotated = cv2.warpAffine(aruco, aM, np.intp((twidth, theight)), 1/scale)
            rotated = imutils.rotate_bound(aruco_s, tang+90)
            print(f"rotated.shape: {rotated.shape}")
            #targc = (lrp-ulp)/2
            asv = lin.vector(*aruco_s.shape[:2])/2
            targo = tscenter - asv
            targo = np.intp(targo)
            x_offset = targo[0]
            y_offset = targo[1]
            #x_offset = int(txmin)
            #y_offset = int(tymin)
            print(f"targo: {targo}, x_off: {x_offset}, y_off: {y_offset}")
            canvas[y_offset:y_offset+rotated.shape[0], x_offset:x_offset+rotated.shape[1]] = rotated
            
            cv2.line(canvas, np.intp(ulp), np.intp(llp), white, 1)
            cv2.line(canvas, np.intp(llp), np.intp(lrp), white, 1)
            cv2.line(canvas, np.intp(lrp), np.intp(urp), white, 1)
            cv2.line(canvas, np.intp(urp), np.intp(ulp), white, 1)
            cv2.circle(canvas, np.intp(ulp), 5, green)
        
        # checker board 
        cbw = (105/2)*scale
        cbh = (75/2)*scale
        cs = (15)*scale
        cbxo = xc-cbw
        cbyo = yc-cbh
        cbxm = xc+cbw
        cbym = yc+cbh
        #cv2.rectangle(canvas, np.intp((cbxo, cbyo)), np.intp((cbxm, cbym)), green)
        
        for i in range(8):
            cv2.line(canvas, np.intp((cbxo+(i*cs), cbyo)), np.intp((cbxo+i*cs, cbym)), green)
        for i in range(6):
            cv2.line(canvas, np.intp((cbxo, cbyo+(i*cs))), np.intp((cbxm, cbyo+i*cs)), green)
            


        
        
        
        
        cv2.imshow('output', canvas)
        if cv2.waitKey(10000) == 27:
            break
        
if __name__ == "__main__":
    print()
    galvatron() 