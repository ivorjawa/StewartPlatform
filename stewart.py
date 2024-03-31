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

def euler_rotation_matrix(alpha,beta,gamma):
    """
    https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-3d/
    Generate a full three-dimensional rotation matrix from euler angles
 
    Input
    :param alpha: The roll angle (radians) - Rotation around the x-axis
    :param beta: The pitch angle (radians) - Rotation around the y-axis
    :param alpha: The yaw angle (radians) - Rotation around the z-axis
 
    Output
    :return: A 3x3 element matix containing the rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
 
    """
    # First row of the rotation matrix
    r00 = np.cos(gamma) * np.cos(beta)
    r01 = np.cos(gamma) * np.sin(beta) * np.sin(alpha) - np.sin(gamma) * np.cos(alpha)
    r02 = np.cos(gamma) * np.sin(beta) * np.cos(alpha) + np.sin(gamma) * np.sin(alpha)
     
    # Second row of the rotation matrix
    r10 = np.sin(gamma) * np.cos(beta)
    r11 = np.sin(gamma) * np.sin(beta) * np.sin(alpha) + np.cos(gamma) * np.cos(alpha)
    r12 = np.sin(gamma) * np.sin(beta) * np.cos(alpha) - np.cos(gamma) * np.sin(alpha)
     
    # Third row of the rotation matrix
    r20 = -np.sin(beta)
    r21 = np.cos(beta) * np.sin(alpha)
    r22 = np.cos(beta) * np.cos(alpha)
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def quaternion_rotation_matrix(Q):
    """
    https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
        
class DrawList(object):
    def __init__(self):
        self.points = []
        self.colors = []
        self.widths = []
        self.compoints = None
    def drawline(self, p1, p2, color, width):
        self.points.append(p1)
        self.points.append(p2)
        self.colors.append(color)
        self.widths.append(width)
    def compile(self):
        self.compoints = np.array(self.points)
    def draw(self, grid, compoints = []): # can draw on a transformed array
        if (len(compoints) == 0):
            compoints = self.compoints
        for i in range(len(self.colors)):
            grid.line(twod(compoints[2*i]), twod(compoints[2*i + 1]), self.colors[i], self.widths[i])
            
class Stewart(object): # millimeters
    def __init__(self, inner_r, outer_r, footprint, min_cyl, max_cyl):
        self.Cmin = min_cyl
        self.Cmax = max_cyl
        self.Crange = max_cyl - min_cyl
        self.inner_r = inner_r
        self.outer_r = outer_r
        self.p0 = lin.vector(0, 0, 0)
        #big_line = lin.vector(outer_r, 0, 0)  
        small_line = lin.vector(inner_r, 0, 0)
        fp_line = lin.vector(footprint/2.0, 0, 0)
        
        # lowest and highest platform can get
        min_height_hyp = m.sqrt( min_cyl**2 - (footprint/2.0)**2 )
        self.min_height = m.sqrt( min_height_hyp**2 - (outer_r-inner_r)**2 )
        max_height_hyp = m.sqrt( max_cyl**2 - (footprint/2.0)**2  )
        self.max_height = m.sqrt( max_height_hyp**2 - (outer_r-inner_r)**2  )
        self.plat_range = self.max_height - self.min_height
        print(f"min_cyl: {min_cyl} max_cyl: {max_cyl} inner_r: {inner_r} outer_r: {outer_r}")
        print(f"min_height_hyp: {min_height_hyp} min_height: {self.min_height}")
        print(f"max_height_hyp: {max_height_hyp} max_height: {self.max_height}")
        #sys.exit(0)

        
        # three spokes of inner wheel are sA, sB, sC ccw
        # three spokes of outer wheel are s1-s6
        # three feet of outer wheel are fA, fB, fC
        
        rotoff = -90 # align with roll and pitch
        self.sA = rotate(zaxis, m.radians(30+rotoff), small_line)
        self.sB = rotate(zaxis, m.radians(150+rotoff), small_line)
        self.sC = rotate(zaxis, m.radians(-90+rotoff), small_line)
        self.old_sA = self.sA
        self.old_sB = self.sB
        self.old_sC = self.sC
        self.old_coll_v = self.p0 *(self.min_height + .5 * self.plat_range)
        
        self.s1 = rotate(zaxis, m.radians(-60+rotoff), fp_line) + outer_r*lin.normalize(self.sA)
        self.s2 = rotate(zaxis, m.radians(120+rotoff), fp_line) + outer_r*lin.normalize(self.sA)
        self.s3 = rotate(zaxis, m.radians(60+rotoff), fp_line) + outer_r*lin.normalize(self.sB)
        self.s4 = rotate(zaxis, m.radians(240+rotoff), fp_line) + outer_r*lin.normalize(self.sB)
        self.s5 = rotate(zaxis, m.radians(180+rotoff), fp_line) + outer_r*lin.normalize(self.sC)
        self.s6 = rotate(zaxis, m.radians(0+rotoff), fp_line) + outer_r*lin.normalize(self.sC)
        

        # circle(self, center, radius, color, lw=1):   
        
        
    def draw(self, grid, roll, pitch, yaw, coll, glyph):
        # TODO make mode where plate moves in sphere and mode where plate is kept level
        
        """
        https://stackoverflow.com/questions/26289972/use-numpy-to-multiply-a-matrix-across-an-array-of-points
        
        transforming multiple points:
        >> mat = lin.rmat(lin.xaxis, m.radians(90))
        >> pts = np.random.random((5,3))
        >> pts
        array([[0.73548668, 0.82505642, 0.24109958],
               [0.16282707, 0.05095367, 0.48493043],
               [0.86938809, 0.17692427, 0.47028215],
               [0.7015419 , 0.59625183, 0.30894065],
               [0.71625289, 0.5231511 , 0.45795695]])
        >> lin.matmul(pts, mat.T)
        array([[ 0.73548668, -0.24109958,  0.82505642],
               [ 0.16282707, -0.48493043,  0.05095367],
               [ 0.86938809, -0.47028215,  0.17692427],
               [ 0.7015419 , -0.30894065,  0.59625183],
               [ 0.71625289, -0.45795695,  0.5231511 ]])
        
        """
        white = cvgraph.white
        red = cvgraph.red
        green = cvgraph.green

        Vp = lin.vector(m.cos(m.radians(pitch)), 0, m.sin(m.radians(pitch)))
        Vr = lin.vector(0, m.cos(m.radians(roll)), m.sin(m.radians(roll)))
        Vq = lin.normalize(Vp + Vr)
        #print(f"Pitch: {pitch} Vp: {Vp} roll: {roll} Vr: {Vr} Vq: {Vq}")
        
        # Normal of rotor disk
        Vdisk = lin.cross(Vp, Vr) 
        Vdisk_n = lin.normalize(Vdisk)
        
        coll_p = coll*self.plat_range+self.min_height
        Vjesus = lin.vector(0, 0, coll_p) # position of Jesus Nut before rotation
        coll_v = coll_p * Vdisk_n         # position of Jesus Nut after rotation
        
        if (glyph & ctrl.cSC): 
            print("using cup motion") 
            modelabel = f"cup motion {glyph}"     
            oily = euler_rotation_matrix(m.radians(-roll),m.radians(pitch),0) # cup motion
        else:
            modelabel = f"sphere motion {glyph}"
            print("using sphere motion")
            oily = euler_rotation_matrix(m.radians(roll),m.radians(-pitch),0) # sphere motion
        
        flatmode = False
        if (glyph & ctrl.cSD):
            flatmode = True
            print("using flat motion")
            coll_v = lin.vector(coll_v[0], coll_v[1], coll_p)
            modelabel = f"flat motion {glyph}"
            
        cvgraph.draw_button(grid.canvas, modelabel, 500, 60, 2)
        
        if flatmode:
            sa = rotate(zaxis, m.radians(yaw), self.sA)
            sb = rotate(zaxis, m.radians(yaw), self.sB)
            sc = rotate(zaxis, m.radians(yaw), self.sC)
            sa = sa+coll_v
            sb = sb+coll_v
            sc = sc+coll_v          
        else:
            sa = lin.matmul(oily, self.sA)+coll_v
            #print(f"sA: {self.sA+Vjesus} sa: {sa}")
            sb = lin.matmul(oily, self.sB)+coll_v
            #print(f"sB: {self.sB+Vjesus} sb: {sb}")
            sc = lin.matmul(oily, self.sC)+coll_v
            #print(f"sC: {self.sC+Vjesus} sc: {sc}")
            sa = rotate(Vdisk_n, m.radians(yaw), sa)
            sb = rotate(Vdisk_n, m.radians(yaw), sb)
            sc = rotate(Vdisk_n, m.radians(yaw), sc)
        

        
        
        triads = [
            [sa, self.s1, self.s2],
            [sb, self.s3, self.s4],
            [sc, self.s5, self.s6],
        ]
        for (top, left, right) in triads:
            c1 = lin.vmag(top-left)
            c2 = lin.vmag(top-right)
            
            if min(c1, c2, self.Cmin) != self.Cmin:
                print(f"cylinder too short: c1: {c1} c2: {c2} Cmin: {self.Cmin}")
                sa = self.old_sA
                sb = self.old_sB
                sc = self.old_sC
                coll_v = self.old_coll_v
                #return
                
            if max(c1, c2, self.Cmax) != self.Cmax:
                print(f"cylinder too long: c1: {c1} c2: {c2} Cax: {self.Cmax}")
                sa = self.old_sA
                sb = self.old_sB
                sc = self.old_sC
                coll_v = self.old_coll_v
                
                #return
        
                   
        dl = DrawList()
        
        dl.drawline(1.5*Vjesus, Vp*50+1.5*Vjesus, cvgraph.orange, 2)
        dl.drawline(1.5*Vjesus, Vr*50+1.5*Vjesus, cvgraph.blue, 2)
        dl.drawline(1.5*Vjesus, Vq*50+1.5*Vjesus, cvgraph.gray1, 2)
        
        dl.drawline(self.p0, coll_v, cvgraph.purple1, 2)
        
        dl.drawline(coll_v, sa, white, 2)   
        dl.drawline(coll_v, sb, white, 2)       
        dl.drawline(coll_v, sc, white, 2) 
        
        dl.drawline(self.s1, self.s2, green, 2)      
        dl.drawline(self.s3, self.s4, green, 2) 
        dl.drawline(self.s5, self.s6, green, 2)  
        
        dl.drawline(self.s1, sa, red, 2) 
        dl.drawline(self.s2, sa, red, 2) 
        dl.drawline(self.s3, sb, red, 2) 
        dl.drawline(self.s4, sb, red, 2)
        dl.drawline(self.s5, sc, red, 2) 
        dl.drawline(self.s6, sc, red, 2)  
        
        dl.compile()
        dl.draw(grid, dl.compoints - xaxis * 150)
        grid.circle(twod(xaxis*-150), self.outer_r, green, lw=1)
        
        offset = xaxis * 250 + yaxis * -200
        
        rm = lin.rmat(xaxis, m.radians(-90))
        screenpts = lin.matmul(dl.compoints, rm.T) + offset
        
        dl.draw(grid, screenpts)
        
        self.old_sA = sa
        self.old_sB = sb
        self.old_sC = sc
        self.old_coll_v = coll_v
        
         
    #def drawline(self, grid, p1, p2, color, width):
    #    grid.line(twod(p1), twod(p2), color, width)
    
      
def test_controller():
    # test with input from game controller
    
    circle, colors = get_circle(200)
    Stew = Stewart(40, 120, 120, 240, 308) #inner, outer radius, footprint, min, max cylinder extension
    
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
        grid = cvgraph.SimGrid(1200, 1200, 1.5)
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
            pitch = (pitch_in - .5) * 40
            yaw = (-yaw_in + 0.5) * 90 
            #print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}")
            Stew.draw(grid, roll, pitch, yaw, coll, glyph)
            
            label = f"{time.time()-t0:.3f}"
            cvgraph.draw_button(grid.canvas, label, 100, 40, 1)


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
