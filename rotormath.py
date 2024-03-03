#!/usr/bin/env python

import sys
import math as m
import time
import numpy as np
import cv2

import controllers as ctrl
import igraph

from linear import xaxis, yaxis, zaxis, rotate, vector
import linear as lin

from swashplate import Swashplate

decode_report = ctrl.decode_taranis
gamepad = ctrl.open_taranis()


# convert from 3-space to 2-space
twod = lambda p: np.array([p[0], p[1]])
   
class Rotor(Swashplate):
            
    def validate(self, cf, cp, cs, coll, pitch, roll, collpct):
        # validate
        cfcp = lin.vmag(cp-cf)
        cfcs = lin.vmag(cs-cf)
        cpcs = lin.vmag(cp-cs)
    
        front = lin.vmag(cf - self.Ff)
        port = lin.vmag(cp - self.Fp)
        star = lin.vmag(cs - self.Fs)
        col_l = lin.vmag(coll - self.P0)
    
        cfc = lin.vmag(cf - coll)
        cpc = lin.vmag(cp - coll)
        csc = lin.vmag(cs - coll)
    
        print(f"pitch: {pitch}, roll: {roll}, collpct: {collpct}")
        print("dist betweeen points:")
        print(f"cfcp: {cfcp}, cfcs: {cfcs}, cpcs: {cpcs}")
        print("blade lengths:")
        print(f"cfc: {cfc}, cpc: {cpc}, csc: {csc}")
        print(f"front: {front}, port: {port}, starboard: {star}, collective: {col_l}")
                
    def drawline(self, grid, p1, p2, color, width):
        grid.line(twod(p1), twod(p2), color, width)
        
    def drawframe(self, grid, pitch, roll, collpct, scene_zrot):
        zrr = m.radians(scene_zrot)
        screen_r = m.radians(-90)
        try:
            (cf, cp, cs, coll) = self.solve(pitch, roll, collpct)
            self.old_Cf = cf
            self.old_Cp = cp
            self.old_Cs = cs
            self.old_Vmast = coll
        except ValueError as e:
            label = f"Range Error P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} {e}"
            print(label)
            igraph.draw_button(grid.canvas, label, 100, 70, .5)
            cf = self.old_Cf
            cp = self.old_Cp
            cs = self.old_Cs
            coll = self.old_Vmast
        
        #cylinder lengths in mm    
        cfc = lin.vmag(cf - self.Ff)
        cpc = lin.vmag(cp - self.Fp)
        csc = lin.vmag(cs - self.Fs)
        label = f"P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} CF: {cfc:{4}.{4}} CP: {cpc:{4}.{4}} CS: {csc:{4}.{4}}"

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
        self.drawline(grid, scoll_s, coll_s, igraph.white, 1)
        
        # draw feet
        self.drawline(grid, scoll_s, cyl_front_r, igraph.red, 2)
        self.drawline(grid, scoll_s, cyl_port_r, igraph.green, 2)
        self.drawline(grid, scoll_s, cyl_star_r, igraph.blue, 2)
        
        # draw arms
        self.drawline(grid, coll_s, cf_r, igraph.red, 2)
        self.drawline(grid, coll_s, cp_r, igraph.green, 2)
        self.drawline(grid, coll_s, cs_r, igraph.blue, 2)
        
        # draw cylinders
        self.drawline(grid, cyl_front_r, cf_r, igraph.red, 1)
        self.drawline(grid, cyl_port_r, cp_r, igraph.green, 1)
        self.drawline(grid, cyl_star_r, cs_r, igraph.blue, 1)
        
        igraph.draw_button(grid.canvas, label, 100, 40, .5)
        
    
def test_recorded():
    # animate with fake data
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
            rot.drawframe(grid, pitch, roll, coll, scenerot)
            
            grid.display()
            inkey  = cv2.waitKey( 30)
            if inkey == 27:
                sys.exit(0)

def test_controller():
    # test with input from game controller
    
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
            scale = 256.0
            coll_in = rd['coll']/scale
            roll_in = rd['roll']/scale          
            pitch_in = rd['pitch']/scale
            yaw_in = rd['yaw']/scale
            #print(f"c: {coll_in:{3.3}}, r: {roll_in:{3.3}}, p: {pitch_in:{3.3}}")
            
            coll = coll_in
            roll = (roll_in - 0.5) * 40
            pitch = (-pitch_in + .5) * 40
            #print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}")
            rot.drawframe(grid, pitch, roll, coll, scenerot)
        
            grid.display()
            inkey = cv2.waitKey(1)
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