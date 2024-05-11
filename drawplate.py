#!/usr/bin/env python

import sys
import time
import math as m

from rich import print as rp

import numpy as np
import cv2

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
        
        
        tsr = (13/2)*scale # exactly 13mm on a side for printed ArUco squares
        tsv = lin.vector(0, tscr, 0) # starts offset 90deg
        
        tvx = lin.vector(tsr, 0, 0)
        tvy  = lin.vector(0, tsr, 0)
        # ArUco is upside down when red27 is up
        ur = (tsv+tvx-tvy)
        lr = (tsv+tvx+tvy)
        ul = (tsv-tvx-tvy)
        ll = (tsv-tvx+tvy)
        
        for i in range(3):
            ulp = rot2deg(i*120, ul)+cea
            llp = rot2deg(i*120, ll)+cea
            urp = rot2deg(i*120, ur)+cea
            lrp = rot2deg(i*120, lr)+cea
            cv2.line(canvas, np.intp(ulp), np.intp(llp), white, 1)
            cv2.line(canvas, np.intp(llp), np.intp(lrp), white, 1)
            cv2.line(canvas, np.intp(lrp), np.intp(urp), white, 1)
            cv2.line(canvas, np.intp(urp), np.intp(ulp), white, 1)
            cv2.circle(canvas, np.intp(ulp), 5, green)
        
        
        
        cv2.imshow('output', canvas)
        if cv2.waitKey(1000) == 27:
            break
        
if __name__ == "__main__":
    galvatron() 