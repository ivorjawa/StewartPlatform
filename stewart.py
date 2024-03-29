#!/usr/bin/env python

import sys
import math as m
import time
import numpy as np
import cv2

import controllers as ctrl
import cvgraph

from linear import xaxis, yaxis, zaxis, rotate, vector
import linear as lin

# convert from 3-space to 2-space
twod = lambda p: np.array([p[0], p[1]])
   
decode_report = ctrl.decode_taranis
gamepad = ctrl.open_taranis()

def get_circle(radius, n = 128):
    circle = []
    colors = []
    for i in range(n):
        theta = (i/float(n)) * 2*m.pi
        circle.append(
            [radius*m.cos(theta),
            radius*m.sin(theta),
            0])
        color = cvgraph.hsv82bgr((i/float(n))*255, 255, 255)
        colors.append(color)
    return circle, colors

class Stewart(object): # millimeters
    def __init__(self, inner_r, outer_r, footprint, min_cyl, max_cyl):
        self.Cmin = min_cyl
        self.Cmax = max_cyl
        self.Crange = max_cyl - min_cyl
        self.inner_r = inner_r
        self.outer_r = outer_r
        self.p0 = lin.vector(0, 0, 0)
        big_line = lin.vector(outer_r, 0, 0)  
        small_line = lin.vector(inner_r, 0, 0)
        fp_line = lin.vector(footprint/2.0, 0, 0)
        

        
        # three spokes of inner wheel are sA, sB, sC ccw
        # three spokes of outer wheel are s1-s6
        # three feet of outer wheel are fA, fB, fC
        self.sA = rotate(zaxis, m.radians(30), small_line)
        self.sB = rotate(zaxis, m.radians(150), small_line)
        self.sC = rotate(zaxis, m.radians(-90), small_line)
        
        self.s1 = rotate(zaxis, m.radians(-60), fp_line) + 2*self.sA
        self.s2 = rotate(zaxis, m.radians(120), fp_line) + 2*self.sA
        self.s3 = rotate(zaxis, m.radians(60), fp_line) + 2*self.sB
        self.s4 = rotate(zaxis, m.radians(240), fp_line) + 2*self.sB
        self.s5 = rotate(zaxis, m.radians(180), fp_line) + 2*self.sC
        self.s6 = rotate(zaxis, m.radians(0), fp_line) + 2*self.sC
        # circle(self, center, radius, color, lw=1):   
    
    def draw(self, grid):
        white = cvgraph.white
        red = cvgraph.red
        green = cvgraph.green
        
        self.drawline(grid, self.p0, self.sA, white, 2)   
        self.drawline(grid, self.p0, self.sB, white, 2)       
        self.drawline(grid, self.p0, self.sC, white, 2) 
        
        self.drawline(grid, self.s1, self.s2, green, 2)      
        self.drawline(grid, self.s3, self.s4, green, 2) 
        self.drawline(grid, self.s5, self.s6, green, 2)  
        
        self.drawline(grid, self.s1, self.sA, red, 2) 
        self.drawline(grid, self.s2, self.sA, red, 2) 
        self.drawline(grid, self.s3, self.sB, red, 2) 
        self.drawline(grid, self.s4, self.sB, red, 2)
        self.drawline(grid, self.s5, self.sC, red, 2) 
        self.drawline(grid, self.s6, self.sC, red, 2)  
        
        
    def draw(self, grid, roll, pitch, yaw, coll):
        # TODO make mode where plate moves in sphere and mode where plate is kept level
        white = cvgraph.white
        red = cvgraph.red
        green = cvgraph.green

        Vp = lin.vector(m.cos(m.radians(pitch)), 0, m.sin(m.radians(pitch)))
        Vr = lin.vector(0, m.cos(m.radians(roll)), m.sin(m.radians(roll)))
        
        # Normal of rotor disk
        Vdisk = lin.cross(Vp, Vr) 
        Vdisk_n = lin.normalize(Vdisk)
        
        coll_p = coll*self.Crange+self.Cmin
        coll_v = coll_p * Vdisk_n
        
        sa = rotate(Vdisk_n, m.radians(yaw), self.sA)+coll_v
        sb = rotate(Vdisk_n, m.radians(yaw), self.sB)+coll_v
        sc = rotate(Vdisk_n, m.radians(yaw), self.sC)+coll_v
        
        
        self.drawline(grid, coll_v, sa, white, 2)   
        self.drawline(grid, coll_v, sb, white, 2)       
        self.drawline(grid, coll_v, sc, white, 2) 
        
        self.drawline(grid, self.s1, self.s2, green, 2)      
        self.drawline(grid, self.s3, self.s4, green, 2) 
        self.drawline(grid, self.s5, self.s6, green, 2)  
        
        self.drawline(grid, self.s1, sa, red, 2) 
        self.drawline(grid, self.s2, sa, red, 2) 
        self.drawline(grid, self.s3, sb, red, 2) 
        self.drawline(grid, self.s4, sb, red, 2)
        self.drawline(grid, self.s5, sc, red, 2) 
        self.drawline(grid, self.s6, sc, red, 2)   
            
    def drawline(self, grid, p1, p2, color, width):
        grid.line(twod(p1), twod(p2), color, width)
    
    
    def drawframe(self, grid, pitch, roll, collpct, scene_zrot):
        zrr = m.radians(scene_zrot)
        screen_r = m.radians(-90)
        try:
            (cf, cp, cs, coll) = self.solve(pitch, roll, collpct)
        except ValueError as e:
            label = f"Range Error P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} {e}"
            print(label)
            cvgraph.draw_button(grid.canvas, label, 100, 70, .5)
            cf = self.s_Cf
            cp = self.s_Cp
            cs = self.s_Cs
            coll = self.s_Vmast
        

        label = f"P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} CF: {self.cfc:{4}.{4}} CP: {self.cpc:{4}.{4}} CS: {self.csc:{4}.{4}}"

        # rotate arms for animation
        cf_r = rotate(zaxis, zrr, cf)
        cs_r = rotate(zaxis, zrr, cs)
        cp_r = rotate(zaxis, zrr, cp)
        
        # rotate arms to screen
        cf_r = rotate(xaxis, screen_r, cf_r)
        cs_r = rotate(xaxis, screen_r, cs_r)
        cp_r = rotate(xaxis, screen_r, cp_r)
        
        
        #rotate feet for animation
        cyl_front_r = rotate(zaxis, zrr, self.Ff)
        cyl_port_r = rotate(zaxis, zrr, self.Fp)
        cyl_star_r = rotate(zaxis, zrr, self.Fs)
        
        #rotate feet for screen
        cyl_front_r = rotate(xaxis, screen_r, cyl_front_r)
        cyl_port_r = rotate(xaxis, screen_r, cyl_port_r)
        cyl_star_r = rotate(xaxis, screen_r, cyl_star_r)
        
        # don't need to rotate center point/coll about itself for animation

        # rotate collective for screen
        scoll_s = rotate(xaxis, screen_r, self.P0)
        coll_s = rotate(xaxis, screen_r, coll)
        
        # draw collective
        self.drawline(grid, scoll_s, coll_s, cvgraph.white, 1)
        
        # draw feet
        self.drawline(grid, scoll_s, cyl_front_r, cvgraph.red, 2)
        self.drawline(grid, scoll_s, cyl_port_r, cvgraph.green, 2)
        self.drawline(grid, scoll_s, cyl_star_r, cvgraph.blue, 2)
        
        # draw arms
        self.drawline(grid, coll_s, cf_r, cvgraph.red, 2)
        self.drawline(grid, coll_s, cp_r, cvgraph.green, 2)
        self.drawline(grid, coll_s, cs_r, cvgraph.blue, 2)
        
        # draw cylinders
        self.drawline(grid, cyl_front_r, cf_r, cvgraph.red, 1)
        self.drawline(grid, cyl_port_r, cp_r, cvgraph.green, 1)
        self.drawline(grid, cyl_star_r, cs_r, cvgraph.blue, 1)
        
        cvgraph.draw_button(grid.canvas, label, 100, 40, .5)    
def test_controller():
    # test with input from game controller
    
    circle, colors = get_circle(200)
    Stew = Stewart(100, 200, 200, 200, 300) #inner, outer radius, footprint, min, max cylinder extension
    
    # arm radius, min cylinder, max cylinder
    #rot = Rotor(60, 160, 200) # real robot
    #rot = Rotor(70, 100, 130) #pneumatic dummy
    
    t0 = time.time()
    output = []
    count = 0
    counting = True
    
    while 1:
        scenerot = (time.time()*30)% 360
        #for scenerot in range(0, 360):
        grid = cvgraph.SimGrid(800, 800, 1.5)
        report = gamepad.read(64)
        
        #for i in range(len(circle)-1):
        #    Stew.drawline(grid, circle[i], circle[i+1], colors[i], 3)
        #Stew.drawline(grid, circle[-1], circle[0], colors[-1], 3)
        #Stew.draw(grid)
        #report = None
        if report:
            # Taranis major axes 3 5 7 9 lx = 9, ly = 7, rx = 3, ry = 5 
            rd = decode_report(report)
            scale = 256.0
            coll_in = rd['coll']/scale
            roll_in = rd['roll']/scale          
            pitch_in = rd['pitch']/scale
            yaw_in = rd['yaw']/scale
            glyph = rd['glyph']
            
            #print(f"c: {coll_in:{3.3}}, r: {roll_in:{3.3}}, p: {pitch_in:{3.3}}")
            
            coll = coll_in
            roll = (-roll_in + 0.5) * 40
            pitch = (-pitch_in + .5) * 40
            yaw = (-yaw_in + 0.5) * 90 
            print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}")
            Stew.draw(grid, pitch, roll, yaw, coll)
            
            label = f"{time.time()-t0:.3f}"
            cvgraph.draw_button(grid.canvas, label, 100, 40, .5)


        #re-indent to make part of while loop
        grid.display()
        inkey = cv2.pollKey()
        #inkey = cv2.waitKey(1)
        #break
        #inkey  = cv2.waitKey( 30)
        if inkey == 27:
            sys.exit(0)
            
   
def run():
    #test_recorded()
    test_controller()
    sys.exit()

if __name__ == "__main__":
    run()
