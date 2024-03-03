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

decode_report = ctrl.decode_taranis
gamepad = ctrl.open_taranis()


# convert from 3-space to 2-space
twod = lambda p: np.array([p[0], p[1]])

    
class Rotor(object):
    # coordinate system is +z:up +x:forward +y:port
    def __init__(self, rotor_rad, c_min, c_max):
        self.Cmin = c_min
        self.Cmax = c_max
        self.Crange = c_max - c_min
        self.Rsw = rotor_rad
        self.P0 = vector(0, 0, 0)
        print(f"Rotor Cmin: {self.Cmin} Cmax: {self.Cmax} Crange: {self.Crange} Rsw: {self.Rsw} ")
        
        # Feet 
        self.Ff = vector(rotor_rad, 0, 0) # Front 
        self.Fp = rotate(zaxis, m.radians(120), self.Ff)
        #self.Fp = vector(rotor_rad*m.sin(m.radians(120)), rotor_rad*m.cos(m.radians(120)), 0)
        self.Fs = rotate(zaxis, m.radians(-120), self.Ff)
        #self.Fs = vector(rotor_rad*m.sin(m.radians(-120)), rotor_rad*m.cos(m.radians(120)), 0)
        
        self.feet = [self.Ff, self.Fp, self.Fs]
        
        # default 50% collective, position after calibration
        tmat = np.array([0, 0, .5*self.Crange + self.Cmin])
        
        self.old_Vmast = tmat
        self.old_Cf = self.Ff + tmat
        self.old_Cp = self.Fp + tmat
        self.old_Cs = self.Fs + tmat
        print(f"front: {self.old_Cf}, port: {self.old_Cp}, startboard: {self.old_Cs}")
    def validate(self, cf, cp, cs, coll, pitch, roll, collpct):
        # validate
        cfcp = lin.vmag(cp-cf)
        cfcs = lin.vmag(cs-cf)
        cpcs = lin.vmag(cp-cs)
    
        front = lin.vmag(cf - self.old_Cf)
        port = lin.vmag(cp - self.old_Cp)
        star = lin.vmag(cs - self.old_Cs)
        col_l = lin.vmag(coll - self.old_Vmast)
    
        cfc = lin.vmag(cf - coll)
        cpc = lin.vmag(cp - coll)
        csc = lin.vmag(cs - coll)
    
        print(f"pitch: {pitch}, roll: {roll}, collpct: {collpct}")
        print("dist betweeen points:")
        print(f"cfcp: {cfcp}, cfcs: {cfcs}, cpcs: {cpcs}")
        print("blade lengths:")
        print(f"cfc: {cfc}, cpc: {cpc}, csc: {csc}")
        print(f"front: {front}, port: {port}, starboard: {star}, collective: {col_l}")
    def newsolve(self, pitch, roll, collpct):
        print(f"newsolve pitch: {pitch} roll: {roll}, coll%: {collpct}")
        Vp = rotate(yaxis, m.radians(pitch), vector(1, 0, 0))
        #Vp = vector(m.cos(m.radians(pitch)), 0, m.sin(m.radians(pitch)))
        Vr = rotate(xaxis, m.radians(roll), vector(0, 1, 0))
        #Vr = vector(0, m.cos(m.radians(roll)), m.sin(m.radians(roll)))
        Vdisk = np.cross(Vp, Vr) 
        Vdisk_n = lin.normalize(Vdisk)
        
        print(f"Vp: {lin.fv(Vp)}, Vr: {lin.fv(Vr)}, Vdisk_n: {lin.fv(Vdisk_n)}")
        Vmast = [0, 0, self.Cmin + collpct*self.Crange] # top of mast at collective setting
        arms = []
        for i, Fn in enumerate(self.feet):
            #print(f"i: {i}, cyl: {cyl}")
            # Vcn is the plane the cylinder rotates on its foot in, Foot X Mast
            Vcn = np.cross(Fn, Vmast)
            #cyl_norm_n = cyl_norm / np.linalg.norm(cyl_norm)
            Vcn_n = lin.normalize(Vcn)
            print(f"Cylinder: {i} Vcn_n: {lin.fv(Vcn_n)}")
            # Visect is the intersection of the Rotor Disk plane and the Cylinder rotation plane
            Visect = lin.cross(Vdisk_n, Vcn_n) # should be plane intersection
            Visect_n = lin.normalize(Visect)
            print(f"Visect_n: {lin.fv(Visect_n)}")
            #print(f"cylinder: {i}, cyl_norm_n: {cyl_norm_n}, intersection_n: {isect_n}")
            # Va is the arm vector as rotated in the cylinder rotation plane
            Va = (self.Rsw * Visect_n) + Vmast
            print(f"Va: {lin.fv(Va)}")
            
            arms.append(Va)
            cyl_len = lin.vmag(Va - Fn)
            print(f"Cyl len: {cyl_len}")
            if cyl_len < self.Cmin:
                raise ValueError(f"too short! Cyl: {cyl_len:{4}.{4}} min: {self.Cmin:{4}}")
            elif cyl_len > self.Cmax:
                raise ValueError(f"too long! Cyl: {cyl_len:{4}.{4}} max: {self.Cmax:{4}}")
        
        (Cf, Cp, Cs) = arms        
        #self.validate(Cf, Cp, Cs, Vmast, pitch, roll, collpct)
        return (Cf, Cp, Cs, Vmast)
            
        
    def draw2(self, grid, p1, p2, color, width):
        grid.line(twod(p1), twod(p2), color, width)
        
    def draw(self, grid, pitch, roll, collpct, scene_zrot):
        zrr = m.radians(scene_zrot)
        screen_r = m.radians(-90)
        try:
            (cf, cp, cs, coll) = self.newsolve(pitch, roll, collpct)
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

        # rotate cylinders for animation
        cf_r = rotate(zaxis, zrr, cf)
        cs_r = rotate(zaxis, zrr, cs)
        cp_r = rotate(zaxis, zrr, cp)
        
        # rotate cylinders to screen
        cf_r = rotate(xaxis, screen_r, cf_r)
        cs_r = rotate(xaxis, screen_r, cs_r)
        cp_r = rotate(xaxis, screen_r, cp_r)
        
        # don't need to rotate center point/coll about itself
        cyl_front_r = rotate(zaxis, zrr, self.Ff)
        cyl_port_r = rotate(zaxis, zrr, self.Fp)
        cyl_star_r = rotate(zaxis, zrr, self.Fs)
        
        cyl_front_r = rotate(xaxis, screen_r, cyl_front_r)
        cyl_port_r = rotate(xaxis, screen_r, cyl_port_r)
        cyl_star_r = rotate(xaxis, screen_r, cyl_star_r)
        
        scoll_s = rotate(xaxis, screen_r, self.P0)
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
            scale = 256.0
            coll_in = rd['coll']/scale
            roll_in = rd['roll']/scale          
            pitch_in = rd['pitch']/scale
            yaw_in = rd['yaw']/scale
            print(f"c: {coll_in:{3.3}}, r: {roll_in:{3.3}}, p: {pitch_in:{3.3}}")
            
            coll = coll_in
            roll = (roll_in - 0.5) * 40
            pitch = (-pitch_in + .5) * 40
            #print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}")
            rot.draw(grid, pitch, roll, coll, scenerot)
        
            grid.display()
            inkey = cv2.waitKey(1)
            #break
            #inkey  = cv2.waitKey( 30)
            if inkey == 27:
                sys.exit(0)
            
   
def run():
    #testrot3()
    testrot5()
    sys.exit()

            
if __name__ == "__main__":
    run()