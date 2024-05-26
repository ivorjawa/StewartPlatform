#!/usr/bin/env python

import sys
import time
import math as m
import pickle
import base64

from rich import print as rp

import numpy as np
import cv2
import imutils

import svgwrite
from svglib.svglib import svg2rlg
from reportlab.graphics import renderPDF

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
cyan = c(255, 255, 0)
magenta = c(255, 0, 255)

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
        
        self.pheight = 11*25.4 #paper height, mm
        self.pwidth = 8.5*25.4 #paper width, mm
        print(f"paper width: {self.pwidth:5.3f}mm, height: {self.pheight:5.3f}mm")
        self.pcx = self.pwidth/2
        self.pcy = self.pheight/2
        self.origin_p = lin.vector(self.pcx-(self.activemm/2), self.pcy-(self.activemm/2))
        print(f"paper center: {self.pcx:5.3f}, {self.pcy:5.3f} origin_p: {self.origin_p}")
        self.dwgfilename = "plate.svg"
        self.dwg = svgwrite.Drawing(
            self.dwgfilename, 
            size=(f"{self.pwidth:5.3f}mm", f"{self.pheight:5.3f}mm"),
            #size=(f"{self.pwidth:5.3f}", f"{self.pheight:5.3f}"),
            viewBox=(f"0 0 {self.pwidth:5.3f} {self.pheight:5.3f}"),
            #viewBox=(f"0 0 {int(8.5*300)} {11*300}"),
            debug=True
        )
        #self.lines = self.dwg.add(self.dwg.g(id='lines', stroke='black'))

        
        #self.lines = self.dwg.add(self.dwg.g(id='lines'))
        self.print_layer = self.dwg.add(self.dwg.g(id='print_layer'))
        self.cut_layer = self.dwg.add(self.dwg.g(id='cut_layer'))
        self.debug_layer = self.dwg.add(self.dwg.g(id='debug_layer'))
        #self.lines.add(self.dwg.rect(insert=(0, 0), size=('100%', '100%'), rx=None, ry=None, fill='rgb(50,50,50)'))
        #paragraph = self.dwg.add(self.dwg.g(font_size=14))
        #paragraph.add(self.dwg.text("This is a Test", x=[10], y=[40, 45, 50, 55, 60]))
        #print(f"scale: {scale} red: {red}")
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("output", self.width, self.height) 
        self.canvas = np.zeros((self.height, self.width, 3), np.uint8)
        cv2.rectangle(self.canvas, (0,0), (self.width, self.height), white, -1)
    def crosshairs(self):
        cv2.line(self.canvas, np.intp((0, self.height/2)), np.intp((self.width, self.height/2)), red, 1)
        cv2.line(self.canvas, np.intp((self.width/2, 0)), np.intp((self.width/2, self.height)), red, 1)
        
    def tp(self, p):
        "translate point from mm to screen pixels, accounting for orientation"
        flipped = lin.vector(p[0], self.activemm-p[1]) 
        #print(f"p: {p}, flipped: {flipped}")
        return np.intp(flipped*self.scale + self.origin_n)
    def tpsvg(self, p):
        # FIXME output is mirrored!
        #flipped = lin.vector(p[0], self.pheight-p[1]) 
        #flipped = lin.vector(p[0], self.activemm-p[1]) 
        #retval = flipped + self.origin_p
        retval = lin.vector(p[0], p[1]) + self.origin_p
        return retval
        #lin.vector(self.pcx-(183/2), self.pcy-(183/2))
        
    def line(self, p1, p2, color, width=1, layer=None):
        if layer is None:
            layer=self.print_layer
        cv2.line(self.canvas, self.tp(p1), self.tp(p2), color, width)
        print(p1, p2)
        layer.add(self.dwg.line(start=f"{p1}mm", end=f"{p2}mm"))
        print("also", p1, p2)
        
    def rectangle(self, p1, p2, color, width=1, layer=None):
        if layer is None:
            layer=self.print_layer
        cv2.rectangle(self.canvas, self.tp(p1), self.tp(p2), color, width)
        p1v = lin.vector(*p1)
        p2v = lin.vector(*p2)
        insert = p1v 
        size = p2v - p1v
        print(f"insert: {insert} size: {size}")
        # px means mm?  what the hell?
        r = self.dwg.rect(insert=self.tpsvg(insert), size=(size[0]*svgwrite.px, size[1]*svgwrite.px))
        if width > 0:
            print("ring rect")
            r.fill('white', opacity=0.0)
            r.stroke(self.bgr2hexrgb(color), width=f"{width*.1}px")
        else:
            print("filled rect")
            r.fill(self.bgr2hexrgb(color))
            r.stroke(self.bgr2hexrgb(color), width="0.2px")
        layer.add(r)
        
    def bgr2hexrgb(self, bgr):
        b, g, r = bgr
        hexstring = f"#{r:02x}{g:02x}{b:02x}"
        #print(f"b: {b} g: {g} r: {r} hexstring: {hexstring}")
        return hexstring
    
        
    def circle(self, center, radius, color, width=1, layer=None):
        if layer is None:
            layer=self.print_layer
        print(f"circle c:{center}, self.center: {self.center}")
        cv2.circle(self.canvas, self.tp(center), int(radius*self.scale), color, width)
        
        centered = self.tpsvg(center) 
        print(f"centered: {centered}")
        circle = self.dwg.circle(centered, radius)
        if width > 0:
            print("ring circle")
            circle.fill('white', opacity=0.0)
            circle.stroke(self.bgr2hexrgb(color), width=f"{width*0.3}px")
        else:
            print("filled circle")
            circle.fill(self.bgr2hexrgb(color))
            circle.stroke(self.bgr2hexrgb(color), width="0.3px")
        layer.add(circle)
        
    
    def polylines(self, points, isClosed, color, thickness=1):
        tpoints = np.array([self.tp(p) for p in points])
        cv2.polylines(self.canvas, [tpoints], isClosed, color, thickness) 
        
    def fillPoly(self, points, color, layer=None):
        if layer is None:
            layer=self.print_layer
        #print(f"fillPoly points: {points}")
        tpoints = np.array([self.tp(p) for p in points])
        #print(f"fillPoly tpoints: {tpoints}")
        
        cv2.fillPoly(self.canvas, [tpoints], color)
        
        ppoints = np.array([self.tpsvg(p) for p in points])
        poly = self.dwg.polygon(ppoints)
        poly.fill(self.bgr2hexrgb(color))
        layer.add(poly)
        
    def fillarc(self, centerpt, inner_r, outer_r, start_angle, end_angle, color, segs=30, layer=None):
        if layer is None:
            layer=self.print_layer
        start_angle = start_angle % 360
        end_angle = end_angle % 360
        min_angle = m.radians(min([start_angle, end_angle]))
        max_angle = m.radians(max([start_angle, end_angle]))
        delta = max_angle - min_angle
        da = delta/segs
        #print(f"center: {centerpt}, inner_r: {inner_r} outer_r: {outer_r} start_angle: {start_angle} end_angle: {end_angle}")
        #print(f"min_angle: {min_angle}, max_angle: {max_angle}, delta: {delta}, da: {da}")
        points = []
        for i in range(segs+1):
            angle = min_angle + da*i
            x = inner_r * m.cos(angle)
            y = inner_r * m.sin(angle)
            #print(f"i: {i} angle: {m.degrees(angle):4.3f} ({x:4.3f}, {y:4.3f})")
            points.append(lin.vector(x,y)+centerpt)
        for i in range(segs+1):
            #i = (segs)-i
            angle = max_angle - da*i
            x = outer_r * m.cos(angle)
            y = outer_r * m.sin(angle)
            #print(f"i: {i} angle: {m.degrees(angle):4.3f} ({x:4.3f}, {y:4.3f})")
            points.append(lin.vector(x,y)+centerpt)
        #print(f"points: {points}")    
        self.fillPoly(points, color, layer)
        
            
    def show(self):
        self.dwg.save(pretty=True)
        drawing = svg2rlg(self.dwgfilename)
        renderPDF.drawToFile(drawing, self.dwgfilename.replace("svg", "pdf"))
        #with open('plate.svg', 'w', encoding='utf-8') as f:
        #    self.dwg.write(f, pretty=True)
        cv2.imshow('output', self.canvas)
        
    def rotstamp(self, image, angle, width, height, point, layer=None):
        if layer is None:
            layer=self.print_layer
        #img_encode = cv.imencode('.png', img)[1]
        # Converting the image into numpy array 
        #data_encode = np.array(img_encode) 
        # Converting the array to bytes. 
        #byte_encode = data_encode.tobytes()
        img_encode = cv2.imencode('.png', image)[1]
        byte_encode = img_encode.tobytes()
        bbytes = base64.b64encode(byte_encode).decode()
        xlink = f"data:image/png;base64,{bbytes}"
        ppoint = self.tpsvg(point)
        cx = ppoint[0]-5
        cy = ppoint[1]-5
        ximage = self.dwg.image(
            href=(xlink), 
            x=f"{cx}px", 
            y=f"{cy}px",
            width="10px", 
            height="10px")
        #ximage.scale(sx=10, sy=10)
        #ximage.translate(*point)
        ximage.rotate(-angle, center=ppoint)
        layer.add(ximage)
        
        px = point[0]
        py = point[1]
        #print(f"image shape: {image.shape}, angle: {angle}, width: {width}, height: {height} point: ({px:4.1f}, {py:4.1f})")
        tpoint = self.tp(point)
        
        image_s = cv2.resize(image, np.intp((width*self.scale, height*self.scale)))
        rotated = imutils.rotate_bound(image_s, angle)
        #rotated = cv2.bitwise_not(rotated)

        asv = lin.vector(*rotated.shape[:2])/2 # offset by half
        targo = tpoint - asv
        targo = np.intp(targo)
        #print(f"transcaled point: ({tpoint[0]:4.1f}, {tpoint[1]:4.1f}) rotated shape: {rotated.shape}, asv: {asv} targo: {targo}")
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

def new_6_marker_board():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    def gm(marker_id):
        marker_size = 200
        marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        return cv2.cvtColor(marker_image,cv2.COLOR_GRAY2RGB)
    grid_mm = 183
    grid = SquareBoard(800, 700, grid_mm)
    
    ang_dict = {
        15: 1,
        #30: 3,
        45: 4,
        135: 13,
        #150: 15,
        165: 16,
        255: 25,
        #270: 27,
        285: 38,
    }
    aruco_angs = ang_dict.keys() # insertion order is guaranteed in python 3.7+
    arucos = [gm(ang_dict[ang]) for ang in aruco_angs]
    aheight, awidth = arucos[0].shape[:2]
    print(f"arucos[0].shape: {arucos[0].shape}")
    
    grid.circle((0,0), 5, white, layer=grid.debug_layer) # origin
    center = np.array((grid_mm/2, grid_mm/2))
    grid.rectangle((0,0), (grid_mm, grid_mm), black, -1, layer=grid.print_layer)
    grid.rectangle((0,0), (grid_mm, grid_mm), yellow, layer=grid.cut_layer)
    
    ocr = 166/2 # outer circle
    grid.circle(center, ocr, cyan, -1, layer=grid.print_layer) # outer circle
    
    icr = 138/2 # inner circle
    for i in range(3):
        #grid.fillarc(center, icr, ocr, (i+1)*120+60, (i+1)*120+119.99, cyan)
        #grid.fillarc(center, icr, ocr, (i+1)*120+90, (i+1)*120+119.999, cyan)
        grid.fillarc(center, icr, ocr, (i)*120+7.5, (i)*120+52.5, white, layer=grid.print_layer)
        grid.fillarc(center, icr, ocr, (i)*120+22.5, (i)*120+37.5, cyan, layer=grid.print_layer)
        
    grid.circle(center, ocr, yellow, layer=grid.cut_layer) # outer circle
    grid.circle(center, icr, black, layer=grid.cut_layer) # inner circle
        
    
    ccr = 159.3/2 # center of lego pin holes
    grid.circle(center, ccr, red, layer=grid.debug_layer)
    
    hole_rad = 4.77/2
    hole_v = lin.vector(ccr, 0, 0)
    for i in range(4): # secures big paper square to platform
        hvr = rot2deg(i*90, hole_v)
        grid.circle(hvr+center, hole_rad, black, -1, layer=grid.print_layer)       
        grid.circle(hvr+center, hole_rad, green, layer=grid.cut_layer)       
    for i in range(3): # secures ring to gear
        hvr = rot2deg(i*120+30, hole_v)
        grid.circle(hvr+center, hole_rad, black, -1, layer=grid.print_layer)
        grid.circle(hvr+center, hole_rad, red, layer=grid.cut_layer)
    
    tscr = (ocr - icr)/2 + icr # target square circle radius
    grid.circle(center, tscr, white, layer=grid.debug_layer)
    
    tsr = 13/2
    tsir = 10/2
    
    corners = []
    for (i, tang) in enumerate(aruco_angs):
        aid = ang_dict[tang]
        tsc = rot2deg(tang, lin.vector(tscr, 0, 0))
        tpoints = rot_square(tsr, tang)+center+tsc
        grid.fillPoly(tpoints, white, layer=grid.print_layer) 
        #grid.fillPoly(tpoints, black)        
        
        inner_square = rot_square(tsir, tang)  
        corners.append(inner_square)   
        tpoints = inner_square+center+tsc
        grid.fillPoly(tpoints, black, layer=grid.print_layer)
        
        #tpoints = rot_square(tsr, tang)+center+tsc
        #grid.polylines(tpoints, True, white,5)
        
        #print(f"aruco id {aid}, tang: {tang}")
        # rotate stamp in opposite direction
        grid.rotstamp(arucos[i], -tang, tsir*2, tsir*2, tsc+center, layer=grid.print_layer)
        grid.circle(tpoints[0], 1.5, red, 2, layer=grid.debug_layer) # upper left
        
    
    # checker board 
    xc = grid_mm/2
    yc = grid_mm/2
    cbw = (105/2)
    cbh = (75/2)
    cs = (15)
    cbxo = xc-cbw
    cbyo = yc-cbh
    cbxm = xc+cbw
    cbym = yc+cbh
    #cv2.rectangle(canvas, np.intp((cbxo, cbyo)), np.intp((cbxm, cbym)), green)
    
    #for i in range(8):
    #    grid.line(np.intp((cbxo+(i*cs), cbyo)), np.intp((cbxo+i*cs, cbym)), black)
    #for i in range(6):
    #    grid.line(np.intp((cbxo, cbyo+(i*cs))), np.intp((cbxm, cbyo+i*cs)), black)

    for x in range(0, 8, 2):
        for y in range(0, 6, 2):
            x1 = cbxo+x*cs
            y1 = cbyo+y*cs
            x2 = cbxo+(x+1)*cs
            y2 = cbyo+(y+1)*cs
            #print(f"x: {x}, y: {y}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
            grid.rectangle((x1, y1), (x2, y2), black, -1, layer=grid.print_layer)    
        for x in range(1, 7, 2):
            for y in range(1, 5, 2):
                x1 = cbxo+x*cs
                y1 = cbyo+y*cs
                x2 = cbxo+(x+1)*cs
                y2 = cbyo+(y+1)*cs
                #print(f"x: {x}, y: {y}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
                grid.rectangle((x1, y1), (x2, y2), black, -1, layer=grid.print_layer)
    """
            uncorners = (corners-ora3)/scale #unscaled corners
            #uncorners = np.array([(x-ora)/scale for x in corners])
            #print(f"Aruco id: {aruco_ids[i]} corners: {corners}\nuncorners: {uncorners}")
            print(f"Aruco id: {aruco_ids[i]} uncorners:\n{uncorners}")
            out_corners.append(uncorners)
        corner_info = {"ids": np.array(aruco_ids), "corners": np.array(out_corners)}
        pfilename = "corner_info.pickle"
        with open(pfilename, "wb") as f:
            pickle.dump(corner_info, f)
    """  
    grid.crosshairs()
              
    grid.show()

    while(1):
        if cv2.waitKey(10000) == 27:
            break
        
                   
def old_3_marker_board():
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
    #old_3_marker_board()     
    new_6_marker_board() 