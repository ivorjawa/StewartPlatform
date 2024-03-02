#!/usr/bin/env python

import sys
import math as m
import time
import numpy as np
import cv2
import hid

#https://blog.thea.codes/talking-to-gamepads-without-pygame/
# decoding ps4
# https://web.archive.org/web/20210301230721/https://www.psdevwiki.com/ps4/DS4-USB

for device in hid.enumerate():
    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

gamepad = hid.device()

#gamepad.open(0x054c, 0x09cc) # crappy PS4 controller
#from gamepads import decode_ps4 as decode_report

gamepad.open(0x1209, 0x4f54) # Taranis
from gamepads import decode_taranis as decode_report

gamepad.set_nonblocking(True)

import igraph

# convert from 3-space to 2-space
twod = lambda p: np.array([p[0], p[1]])

from linear import xaxis, yaxis, zaxis, rotate
    
def testrot():
    point1 = np.array([0, 1, 0])
    point2 = np.array([1, 0, 0])
    theta = np.radians(45)
    #point1_r = np.dot(rotation_matrix(zaxis, theta), point1)
    point1_r = rotate(zaxis, theta, point1)
    print(f"point1: {point1}, point_r: {point1_r}")
    #points = np.array([point1, point2])
    #points_r = np.dot(rotation_matrix(zaxis, theta), points)
    #print(f"point2: {point2}, points_r: {points_r}")
    
class Rotor(object):
    # coordinate system is +z-up +x-forward
    def __init__(self, rotor_rad, c_min, c_max):
        self.c_min = c_min
        self.c_max = c_max
        self.c_range = c_max - c_min
        self.rotor_rad = rotor_rad
        self.coll = np.array([0, 0, 0])
        self.cyl_front = np.array([-rotor_rad, 0, 0])
        self.cyl_port = rotate(zaxis, m.radians(120), self.cyl_front)
        self.cyl_star = rotate(zaxis, m.radians(-120), self.cyl_front)
        self.cyls = [self.cyl_front, self.cyl_port, self.cyl_star]
        tmat = np.array([0, 0, .5*self.c_range + self.c_min])
        self.old_coll = self.coll + tmat
        self.old_cf = self.cyl_front + tmat
        self.old_cs = self.cyl_star + tmat
        self.old_cp = self.cyl_port + tmat
        print(f"front: {self.cyl_front}, port: {self.cyl_port}, startboard: {self.cyl_star}")
    def validate(self, cf, cp, cs, coll, pitch, roll, collpct):
        # validate
        cfcp = igraph.vlen(cp-cf)
        cfcs = igraph.vlen(cs-cf)
        cpcs = igraph.vlen(cp-cs)
    
        front = igraph.vlen(cf - self.cyl_front)
        port = igraph.vlen(cp - self.cyl_port)
        star = igraph.vlen(cs - self.cyl_star)
        col_l = igraph.vlen(coll - self.coll)
    
        cfc = igraph.vlen(cf - coll)
        cpc = igraph.vlen(cp - coll)
        csc = igraph.vlen(cs - coll)
    
        print(f"pitch: {pitch}, roll: {roll}, collpct: {collpct}")
        print("dist betweeen points:")
        print(f"cfcp: {cfcp}, cfcs: {cfcs}, cpcs: {cpcs}")
        print("blade lengths:")
        print(f"cfc: {cfc}, cpc: {cpc}, csc: {csc}")
        print(f"front: {front}, port: {port}, starboard: {star}, collective: {col_l}")
    def newsolve(self, pitch, roll, collpct):
        pitch_v = rotate(yaxis, m.radians(pitch), np.array([1, 0, 0]))
        roll_v = rotate(xaxis, m.radians(roll), np.array([0, 1, 0]))
        cyclic_norm = np.cross(pitch_v, roll_v)
        cyclic_norm_n = cyclic_norm/np.linalg.norm(cyclic_norm)
        #print(f"pitch_v: {pitch_v}, roll_v: {roll_v}, cyclic_norm_n: {cyclic_norm_n}")
        coll_v = [0, 0, self.c_min + collpct*self.c_range] # top of mast at collective setting
        arms = []
        for i, cyl in enumerate(self.cyls):
            #print(f"i: {i}, cyl: {cyl}")
            cyl_norm = np.cross(cyl, coll_v)
            cyl_norm_n = cyl_norm / np.linalg.norm(cyl_norm)
            isect = np.cross(cyclic_norm, cyl_norm) # should be plane intersection
            isect_n = isect / np.linalg.norm(isect)
            #print(f"cylinder: {i}, cyl_norm_n: {cyl_norm_n}, intersection_n: {isect_n}")
            arm_v = (self.rotor_rad * isect_n) + coll_v
            arms.append(arm_v)
            cyl_len = igraph.vlen(arm_v - cyl)
            if cyl_len < self.c_min:
                raise ValueError(f"too short! Cyl: {cyl_len:{4}.{4}} min: {self.c_min:{4}}")
            elif cyl_len > self.c_max:
                raise ValueError(f"too long! Cyl: {cyl_len:{4}.{4}} max: {self.c_max:{4}}")
        (cf, cp, cs) = arms
        coll = coll_v
        
        #self.validate(cf, cp, cs, coll, pitch, roll, collpct)
        
        
        return (cf, cp, cs, coll)
            
        
    def draw2(self, grid, p1, p2, color, width):
        grid.line(twod(p1), twod(p2), color, width)
        
    def draw(self, grid, pitch, roll, collpct, scene_zrot):
        zrr = m.radians(scene_zrot)
        screen_r = m.radians(-90)
        try:
            (cf, cp, cs, coll) = self.newsolve(pitch, roll, collpct)
            self.old_cf = cf
            self.old_cp = cp
            self.old_cs = cs
            self.old_coll = coll
        except ValueError as e:
            label = f"Range Error P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} {e}"
            print(label)
            igraph.draw_button(grid.canvas, label, 100, 70, .5)
            cf = self.old_cf
            cp = self.old_cp
            cs = self.old_cs
            coll = self.old_coll
            
        cfc = igraph.vlen(cf - self.cyl_front)
        cpc = igraph.vlen(cp - self.cyl_port)
        csc = igraph.vlen(cs - self.cyl_star)
        label = f"P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} CF: {cfc:{4}.{4}} CP: {cpc:{4}.{4}} CS: {csc:{4}.{4}}"
        cf_r = rotate(zaxis, zrr, cf)
        cs_r = rotate(zaxis, zrr, cs)
        cp_r = rotate(zaxis, zrr, cp)
        
        cf_r = rotate(xaxis, screen_r, cf_r)
        cs_r = rotate(xaxis, screen_r, cs_r)
        cp_r = rotate(xaxis, screen_r, cp_r)
        
        # don't need to rotate center point/coll about itself
        cyl_front_r = rotate(zaxis, zrr, self.cyl_front)
        cyl_port_r = rotate(zaxis, zrr, self.cyl_port)
        cyl_star_r = rotate(zaxis, zrr, self.cyl_star)
        
        cyl_front_r = rotate(xaxis, screen_r, cyl_front_r)
        cyl_port_r = rotate(xaxis, screen_r, cyl_port_r)
        cyl_star_r = rotate(xaxis, screen_r, cyl_star_r)
        
        scoll_s = rotate(xaxis, screen_r, self.coll)
        coll_s = rotate(xaxis, screen_r, coll)
        
        self.draw2(grid, scoll_s, coll_s, igraph.white, 1)
        
        self.draw2(grid, scoll_s, cyl_front_r, igraph.red, 2)
        self.draw2(grid, scoll_s, cyl_port_r, igraph.green, 2)
        self.draw2(grid, scoll_s, cyl_star_r, igraph.blue, 2)
        
        self.draw2(grid, coll_s, cf_r, igraph.red, 2)
        self.draw2(grid, coll_s, cp_r, igraph.green, 2)
        self.draw2(grid, coll_s, cs_r, igraph.blue, 2)
        
        self.draw2(grid, cyl_front_r, cf_r, igraph.red, 1)
        self.draw2(grid, cyl_port_r, cp_r, igraph.green, 1)
        self.draw2(grid, cyl_star_r, cs_r, igraph.blue, 1)
        
        igraph.draw_button(grid.canvas, label, 100, 40, .5)
        
        
def testrot2():
    print("\ntestrot2()")
    print("rotor with blade r = 70, coll_min = 100, coll_max = 130")
    rot = Rotor(70, 100, 130)
    rot.solve(-5, 5, .5)

def testrot4():
    print("\ntestrot4()")
    rot = Rotor(70, 100, 130)
    rot.newsolve(-5, -5, .5)
    
def testrot3():
    divs = 50
    degrees = 15
    divs_zero = np.linspace(0, 0, divs)
    divs_degs = np.linspace(-degrees, degrees, divs)
    
    pitches = np.append(divs_zero, divs_zero)
    pitches = np.append(pitches, divs_degs)
    pitches = np.append(pitches, divs_degs)
    
    rolls = np.append(divs_zero, divs_degs)
    rolls = np.append(rolls, divs_zero)
    rolls = np.append(rolls, divs_degs)
    
    rot = Rotor(70, 100, 130)
    numstates = len(rolls)
    statec = 0
    print(f"len(rolls): {len(rolls)}, len(pitches): {len(pitches)}")
    while 1:
        for scenerot in range(0, 360):
            grid = igraph.SimGrid(800, 800, 1.5)
            pitch = pitches[statec]
            roll = rolls[statec]
            coll = .5 # todo, vary collective
            statec = (statec + 1) % numstates
            rot.draw(grid, pitch, roll, coll, scenerot)
            
            grid.display()
            inkey  = cv2.waitKey( 30)
            if inkey == 27:
                sys.exit(0)

def testrot5():
    # arm radius, min cylinder, max cylinder
    rot = Rotor(60, 160, 200) # real robot
    #rot = Rotor(70, 100, 130) #pneumatic dummy
    while 1:
        scenerot = (time.time()*30)% 360
        #for scenerot in range(0, 360):
        grid = igraph.SimGrid(800, 800, 1.5)
        report = gamepad.read(64)
        if report:
            # Taranis major axes 3 5 7 9 lx = 9, ly = 7, rx = 3, ry = 5 
            rd = decode_report(report)
            coll_in = rd['coll']/2048.0 
            roll_in = rd['roll']/2048.0          
            pitch_in = rd['pitch']/2048.0
            yaw_in = rd['yaw']/2048.0
            coll = coll_in
            roll = (roll_in - 0.5) * 40
            pitch = (-pitch_in + .5) * 40
            #print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}")
            rot.draw(grid, pitch, roll, coll, scenerot)
        
            grid.display()
            inkey = cv2.waitKey(1)
            #inkey  = cv2.waitKey( 30)
            if inkey == 27:
                sys.exit(0)
            
# https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1
    
def intersect(p1, r1, p2, r2):
    R = igraph.vlen(p2-p1) # also "d"
    l = (r1**2 - r2**2 + R**2) / (2*R)
    h = m.sqrt(r1**2 - l**2)
    
    xa1 = (l/R)*(p2[0]-p1[0])
    xa2 = (h/R)*(p2[1]-p1[1])
    x1 = xa1 + xa2 + p1[0]
    x2 = xa1 - xa2 + p1[0]
    
    ya1 = (l/R)*(p2[1]-p1[1])
    ya2 = (h/R)*(p2[0]-p1[0])
    y1 = ya1 + ya2 + p1[1]
    y2 = ya1 - ya2 + p1[1]
    print(f"x1: {x1}, x2: {x2}, y1: {y1} y2: {y2}")
    
    # I think this will work
    pgx = max(x1, x2)
    pgy = max(y1, y2)
    pg = np.array([pgx, pgy])
    return pg
    #print(f"pg: {pg}")

def test_rmat():
   cyl_front = np.array([-100, 0, 0])
   cyl_port = rotate(zaxis, m.radians(120), cyl_front)  
   print(f"front: {cyl_front}, port: {cyl_port}")  
    
def run():
    #testrot()
    #testrot2()
    #testrot3()
    #testrot4()
    testrot5()
    #test_rmat()
    sys.exit()

            
if __name__ == "__main__":
    run()