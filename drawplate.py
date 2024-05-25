#!/usr/bin/env python

import sys
import time
import math as m
import pickle

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
black = c(0, 0, 0)

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

class SquareBoard(object):
    def __init__(self, sidesize=800, activesize=700, activemm=183):
        self.activemm = activemm
        self.width = sidesize
        self.height = sidesize
        self.scale = activesize/activemm
        self.offsetlen = (sidesize-activesize)/2
        xo = self.offsetlen
        yo = self.offsetlen
        xc = int(self.width/2)
        yc = int(self.height/2)
        self.origin = (xo, yo)
        self.center = (xc, yc)
        self.origin_n = np.array(self.origin)
        self.center_n = np.array(self.center)
        #print(f"scale: {scale} red: {red}")
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("output", self.width, self.height) 
        self.canvas = np.zeros((self.height, self.width, 3), np.uint8)
        
    def crosshairs(self):
        cv2.line(self.canvas, np.intp((0, self.height/2)), np.intp((self.width, self.height/2)), red, 1)
        cv2.line(self.canvas, np.intp((self.width/2, 0)), np.intp((self.width/2, self.height)), red, 1)
        
    def tp(self, p):
        "translate point from mm to screen pixels, accounting for orientation"
        flipped = lin.vector(p[0], self.activemm-p[1]) 
        #print(f"p: {p}, flipped: {flipped}")
        return np.intp(flipped*self.scale + self.origin_n)
        
    def line(self, p1, p2, color, width=1):
        cv2.line(self.canvas, self.tp(p1), self.tp(p2), color, width)
        
    def rectangle(self, p1, p2, color, width=1):
        cv2.rectangle(self.canvas, self.tp(p1), self.tp(p2), color, width)
        
    def circle(self, center, radius, color, width=1):
        #print(f"circle c:{center}")
        cv2.circle(self.canvas, self.tp(center), int(radius*self.scale), color, width)
    
    def polylines(self, points, isClosed, color, thickness=1):
        tpoints = np.array([self.tp(p) for p in points])
        cv2.polylines(self.canvas, [tpoints], isClosed, color, thickness) 
        
    def fillPoly(self, points, color):
        #print(f"fillPoly points: {points}")
        tpoints = np.array([self.tp(p) for p in points])
        #print(f"fillPoly tpoints: {tpoints}")
        
        cv2.fillPoly(self.canvas, [tpoints], color)
        
    def show(self):
        cv2.imshow('output', self.canvas)
        
    def rotstamp(self, image, angle, width, height, point):
        px = point[0]
        py = point[1]
        print(f"image shape: {image.shape}, angle: {angle}, width: {width}, height: {height} point: ({px:4.1f}, {py:4.1f})")
        #tpoint = point*self.scale + self.center_n
        #flipped = lin.vector(p[0], self.activemm-p[1])
        tpoint = self.tp(point)
        
        #aruco = arucos[i]
        #aruco_s = cv2.resize(aruco, np.intp((tsir*2, tsir*2)))
        #rotated = imutils.rotate_bound(aruco_s, tang+90)        
        
        
        image_s = cv2.resize(image, np.intp((width*self.scale, height*self.scale)))
        rotated = imutils.rotate_bound(image_s, angle)
        #rotated = cv2.bitwise_not(rotated)

        asv = lin.vector(*rotated.shape[:2])/2 # offset by half
        targo = tpoint - asv
        targo = np.intp(targo)
        print(f"transcaled point: ({tpoint[0]:4.1f}, {tpoint[1]:4.1f}) rotated shape: {rotated.shape}, asv: {asv} targo: {targo}")
        x_offset = targo[0]
        y_offset = targo[1]
        self.canvas[y_offset:y_offset+rotated.shape[0], x_offset:x_offset+rotated.shape[1]] |= rotated
        
        
        
def rot_square(radius, angle):
    tvx = lin.vector(radius, 0, 0)
    tvy  = lin.vector(0, radius, 0) 
    
    ul = (-tvx+tvy)
    ur = (tvx+tvy)
    lr = (tvx-tvy)
    ll = (-tvx-tvy)
    
    ulp = rot2deg(angle, ul)
    urp = rot2deg(angle, ur)
    lrp = rot2deg(angle, lr) 
    llp = rot2deg(angle, ll)
    
    return np.array((ulp[:2], urp[:2], lrp[:2], llp[:2]))

def galivate():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    marker_size = 200
    #marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    def gm(marker_id):
        marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        return cv2.cvtColor(marker_image,cv2.COLOR_GRAY2RGB)
    grid_mm = 183
    grid = SquareBoard(800, 700, grid_mm)
    
    #red27 = cv2.imread("aruco/markers/aruco_red27_27.png")
    #green15 = cv2.imread("aruco/markers/aruco_green15_15.png")
    #blue03 = cv2.imread("aruco/markers/aruco_blue03_3.png")
    #arucos = [blue03, green15, red27]
    aruco_ids = [3, 15, 27]
    arucos = [gm(mid) for mid in aruco_ids]
    aheight, awidth = arucos[0].shape[:2]
    print(f"arucos[0].shape: {arucos[0].shape}")
    #print(f"red27.shape: {red27.shape}")
    
    grid.crosshairs()
    grid.circle((0,0), 5, white)
    center = np.array((grid_mm/2, grid_mm/2))
    grid.rectangle((0,0), (grid_mm, grid_mm), yellow)
    ocr = 166/2
    grid.circle(center, ocr, yellow) # outer circle
    icr = 138/2
    grid.circle(center, icr, yellow) # inner circle
    
    ccr = 159.3/2
    grid.circle(center, ccr, red)
    
    hole_rad = 4.77/2
    hole_v = lin.vector(ccr, 0, 0)
    for i in range(4): # secures big paper square to platform
        hvr = rot2deg(i*90, hole_v)
        grid.circle(hvr+center, hole_rad, green)       
    for i in range(3): # secures ring to gear
        hvr = rot2deg(i*120+30, hole_v)
        grid.circle(hvr+center, hole_rad, red)
    
    tscr = (ocr - icr)/2 + icr # target square circle radius
    grid.circle(center, tscr, white)
    
    tsr = 13/2
    tsir = 10/2
    
    for i in range(len(arucos)):
        tang = i*120+30
        tsc = rot2deg(tang, lin.vector(tscr, 0, 0))
        tpoints = rot_square(tsr, tang)+center+tsc
        grid.fillPoly(tpoints, white) 
        #grid.fillPoly(tpoints, black)        
               
        tpoints = rot_square(tsir, tang)+center+tsc
        grid.fillPoly(tpoints, black)
        
        #tpoints = rot_square(tsr, tang)+center+tsc
        #grid.polylines(tpoints, True, white,5)
        
        #image, angle, width, height, point
        print(f"aruco id {aruco_ids[i]}, tang: {tang}")
        # rotate stamp in opposite direction
        grid.rotstamp(arucos[i], -tang, tsir*2, tsir*2, tsc+center)
            
    grid.show()

    while(1):
        if cv2.waitKey(10000) == 27:
            break
        
                   
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
    aruco_ids = [27, 15, 3]
    aheight, awidth = red27.shape[:2]
    print(f"red27.shape: {red27.shape}")

    while 1:
        # crosshairs
        cv2.line(canvas, np.intp((0, height/2)), np.intp((width, height/2)), red, 1)
        cv2.line(canvas, np.intp((width/2, 0)), np.intp((width/2, height)), red, 1)
        
        #plate
        pl = 183*scale
        extents = np.intp((xo+pl, yo+pl))
        cv2.rectangle(canvas, origin, extents, yellow)
        ucstents = (extents - ora) / scale
        print(f"Extents: {extents} ucstents: {ucstents}")
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
        tvix = lin.vector(tsir, 0, 0)
        tviy  = lin.vector(0, tsir, 0)
        # ArUco is upside down when red27 is up
        ur = (tsv+tvx-tvy)
        lr = (tsv+tvx+tvy)
        ul = (tsv-tvx-tvy)
        ll = (tsv-tvx+tvy)
        uri = (tsv+tvix-tviy)
        lri = (tsv+tvix+tviy)
        uli = (tsv-tvix-tviy)
        lli = (tsv-tvix+tviy)
        out_corners = []        
        for i in range(3):
            tang = i*120
            #print(f"target angle: {tang}")
            tscrv = lin.vector(0, tscr, 0)
            tscenter = rot2deg(tang, tscrv)+cea
            #print(f"tscenter:{lin.fv(tscenter)}")
            ulp = rot2deg(tang, ul)+cea
            llp = rot2deg(tang, ll)+cea
            urp = rot2deg(tang, ur)+cea
            lrp = rot2deg(tang, lr)+cea
            ulip = rot2deg(tang, uli)+cea
            llip = rot2deg(tang, lli)+cea
            urip = rot2deg(tang, uri)+cea
            lrip = rot2deg(tang, lri)+cea
            tpoints = [ulp, llp, lrp, urp]
            txmin = axismin(0, tpoints)
            txmax = axismax(0, tpoints)
            tymin = axismin(1, tpoints)
            tymax = axismax(1, tpoints)
            twidth = txmax - txmin
            theight = tymax - tymin
            #print(f"txmin: {txmin}, txmax: {txmax}, tymin: {tymin}, tymax: {tymax}, twidth: {twidth}, theight: {theight}")

            
            #acenter = (awidth/2, aheight/2)
            #print(f"acenter: {acenter}")
            aruco = arucos[i]
            #aruco_1bp = aruco[1]
            #print(aruco_1bp)
            aruco_s = cv2.resize(aruco, np.intp((tsir*2, tsir*2)))
            #print(f"aruco_s.shape: {aruco_s.shape}")
            
            #aM = cv2.getRotationMatrix2D(acenter, tang, 1.0)
            #rotated = cv2.warpAffine(aruco_s, aM, np.intp((tsr*2, tsr*2)))
            #rotated = cv2.warpAffine(aruco, aM, np.intp((twidth, theight)), 1/scale)
            rotated = imutils.rotate_bound(aruco_s, tang+90)
            #print(f"rotated.shape: {rotated.shape}")
            #targc = (lrp-ulp)/2
            asv = lin.vector(*rotated.shape[:2])/2
            targo = tscenter - asv
            targo = np.intp(targo)
            x_offset = targo[0]
            y_offset = targo[1]
            #x_offset = int(txmin)
            #y_offset = int(tymin)
            #print(f"tcenter: {lin.fv(tscenter)}, targo: {targo}, x_off: {x_offset}, y_off: {y_offset}")
            fpp = [np.intp(tpoints)]
            print(f"filly: {fpp}")
            cv2.fillPoly(canvas, fpp, white)
            canvas[y_offset:y_offset+rotated.shape[0], x_offset:x_offset+rotated.shape[1]] = rotated
            
            cv2.line(canvas, np.intp(ulp), np.intp(llp), white, 1)
            cv2.line(canvas, np.intp(llp), np.intp(lrp), white, 1)
            cv2.line(canvas, np.intp(lrp), np.intp(urp), white, 1)
            cv2.line(canvas, np.intp(urp), np.intp(ulp), white, 1)
            cv2.circle(canvas, np.intp(urip), 5, green) # actually "upper left"
            
            ora3 = lin.vector(*ora, 0)
            corners = [urip, ulip, llip, lrip]
            corners = np.array([lin.vector(*x, 0) for x in corners])
            uncorners = (corners-ora3)/scale #unscaled corners
            #uncorners = np.array([(x-ora)/scale for x in corners])
            #print(f"Aruco id: {aruco_ids[i]} corners: {corners}\nuncorners: {uncorners}")
            print(f"Aruco id: {aruco_ids[i]} uncorners:\n{uncorners}")
            out_corners.append(uncorners)
        corner_info = {"ids": np.array(aruco_ids), "corners": np.array(out_corners)}
        pfilename = "corner_info.pickle"
        with open(pfilename, "wb") as f:
            pickle.dump(corner_info, f)
        
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
    #galvatron()     
    galivate() 