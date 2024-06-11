#!/usr/bin/env python

import pyglet
import numpy as np
import math as m
import time

from linear import xaxis, yaxis, zaxis, rotate, vector
import linear as lin
import slerp

from StewartPlatform import StewartPlatform

# convert from 3-space to 2-space
#twod = lambda p: np.array([p[0], p[1]])
twod = lambda p: [int(p[0]), int(p[1])]
 
#from pyglet.gl import *

#joysticks = pyglet.input.get_joysticks()
#assert joysticks, 'No joystick device is connected'
#joystick = joysticks[0]
#joystick.open()


window = pyglet.window.Window(width=800, height=800)
#oldbatch = pyglet.graphics.Batch()


# Labels
#pyglet.text.Label("Buttons:", x=15, y=window.height - 25, font_size=14, batch=oldbatch)
#pyglet.text.Label("D Pad:", x=window.width - 125, y=window.height - 25, font_size=14, batch=oldbatch)

#c = lambda b, g, r: A([b, g, r], dtype='int32')
c = lambda b, g, r: np.array([r, g, b], dtype='int32') # converts bgr to rgb
red = c(0, 0, 255)
red1 = c(*red/2)
green = c(0, 255, 0)
green1 = c(*green/2)
yellow = c(0, 255, 255)
yellow1 = c(*yellow/2)
white = c(255, 255, 255)
gray1 = c(*white/2)
purple = c(255, 0, 255)
purple1 = c(*purple/2)
blue = c(255, 0, 0)
blue1 = c(*blue/2)
orange = c(0, 165, 255)
orange1 = c(*orange/2)

cSA = 24
cSB = 40
cSC = 72
cSD = 68

t0 = time.time()

outlog = []
outcount = 0
fastcount = 0

class DrawList(object):
    def __init__(self):
        self.points = []
        self.colors = []
        self.widths = []
        self.compoints = None
        self.batch = pyglet.graphics.Batch()
        self.lines = []
    def drawline(self, p1, p2, color, width):
        self.points.append(p1)
        self.points.append(p2)
        self.colors.append(color)
        self.widths.append(width)
    def compile(self):
        self.compoints = np.array(self.points)
    def draw(self, compoints = []): # can draw on a transformed array
        self.batch = pyglet.graphics.Batch()
        self.lines = []
        if (len(compoints) == 0):
            compoints = self.compoints
        for i in range(len(self.colors)):
            #grid.line(twod(compoints[2*i]), twod(compoints[2*i + 1]), self.colors[i], self.widths[i])
            #pyglet.shapes.Line(p1[0], p1[1], p2[0], p2[1], width=width, color=color, batch=self.batch)
            #print(*twod(compoints[2*i]), *twod(compoints[2*i + 1]), self.widths[i], self.colors[i])
            l = pyglet.shapes.Line(*twod(compoints[2*i]), *twod(compoints[2*i + 1]), self.widths[i], self.colors[i], batch=self.batch)
            self.lines.append(l)
        self.batch.draw()


def hsv82bgr(h, s, v):
    (r, g, b) = colorsys.hsv_to_rgb(h/255.0, s/255.0, v/255.0)
    return bgr(b*255, g*255, r*255)   
       
def get_circle(radius, n = 128):
    circle = []
    colors = []
    for i in range(n):
        theta = (i/float(n)) * 2*m.pi
        circle.append(
            [radius*m.cos(theta),
            radius*m.sin(theta),
            0])
        color = hsv82bgr((i/float(n))*255, 255, 255)
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


class Stewart(StewartPlatform):
     
    def __init__(self, inner_r, outer_r, footprint, min_cyl, max_cyl):
        super().__init__(inner_r, outer_r, footprint, min_cyl, max_cyl)
        self.last_time = time.time()
        self.times = [time.time() for i in range(5)] 
        self.textbatch = pyglet.graphics.Batch()
        self.labels = []
         
    def draw(self, coll_v, sa, sb, sc):  
        self.textbatch = pyglet.graphics.Batch() 
        self.labels = []        
        dl = DrawList()
                
        dl.drawline(self.p0, coll_v, purple, 2)
        
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
        #print("top")
        dl.draw(dl.compoints + xaxis*300  + yaxis*400)
        #grid.circle(twod(xaxis*-150), self.outer_r, green, lw=1)
        #dl.draw()
        
        offset = xaxis*600 + yaxis* 400
        
        screen_rm = lin.rmat(xaxis, m.radians(-90))
        screenpts = lin.matmul(dl.compoints, screen_rm.T) + offset
        
        #print("side")
        dl.draw(screenpts)
        

        
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        self.times.append(dt)
        self.times.pop(0)
        dtavg = sum(self.times) / len(self.times)
        
        dtlabel = f"{dtavg*1000:.3f} ms/frame"
        #draw_button(grid.canvas, dtlabel, 600, 950, 1) 
        #draw_button(grid.canvas, self.modelabel, 500, 60, 2)
        
"""button_labels = []
button_shapes = []

for i in range(len(joystick.buttons)):
    rows = len(joystick.buttons) // 2
    y = window.height - 50 - 25 * (i % rows)
    x = 35 + 60 * (i // rows)
    label = pyglet.text.Label(f"{i}:", x=x, y=y, font_size=14, anchor_x='right', batch=batch)
    button_labels.append(label)
    shape = pyglet.shapes.Rectangle(x + 10, y + 1, 10, 10, color=(255, 0, 0), batch=batch)
    button_shapes.append(shape)


joystick_rect = pyglet.shapes.Rectangle(window.width // 2, window.height // 2, 10, 10, color=(255, 0, 255), batch=batch)
joystick_rect.anchor_position = joystick_rect.width // 2, joystick_rect.height // 2
d_pad_rect = pyglet.shapes.Rectangle(window.width - 75, window.height - 100, 10, 10, color=(0, 0, 255), batch=batch)"""


#Stew = Stewart(40, 120, 120, 240, 308) #inner, outer radius, footprint, min, max cylinder extension
Stew = Stewart(57, 98, 120, 250, 314) #inner, outer radius, footprint, min, max cylinder extension

#line2 = pyglet.shapes.Line(0, 0, window.width/2, window.height/2, width=4, color=(200, 20, 20), batch=textbatch)

#@window.event
def on_drawl():
    #global textbatch
    window.clear()
    #oldbatch.draw()
    
    #print(window.width, window.height)
    axes = joystick.device.get_controls()[24:]
    
    coll_in = axes[2].value/2047.0
    roll_in = axes[0].value/1024.0 - 1
    pitch_in = axes[1].value/1024.0 - 1
    yaw_in = axes[3].value/1024.0 - 1
    
    #print(f"ci: {coll_in:{3.3}}, ri: {roll_in:{3.3}}, pi: {pitch_in:{3.3}}, yi: {yaw_in:{3.3}}")
    
    glyph = 0
    if (axes[4].value == 1024): glyph |= cSA
    if (axes[5].value == 1024): glyph |= cSB
    if (axes[6].value == 1024): glyph |= cSC
    if (axes[7].value == 1024): glyph |= cSD

    
    disk_def = 10 # degrees 
    coll = coll_in 
    roll = (-roll_in) * disk_def
    pitch = (pitch_in) * disk_def
    yaw = (yaw_in) * 35 
    

    
    #print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}, y: {yaw:{3.3}} g: {glyph}")
    (coll_v, sa, sb, sc) = Stew.solve(roll, pitch, yaw, coll, glyph)
    Stew.draw(coll_v, sa, sb, sc)
    
    l = pyglet.text.Label(f"coll: {coll: 3.1f}", x=15, y=window.height - 25, font_size=14, batch=Stew.textbatch)
    Stew.labels.append(l)
    l = pyglet.text.Label(f"roll: {roll: 3.1f}", x=15, y=window.height - 45, font_size=14, batch=Stew.textbatch)
    Stew.labels.append(l)    
    l = pyglet.text.Label(f"pitch: {pitch: 3.1f}", x=15, y=window.height - 65, font_size=14, batch=Stew.textbatch)
    Stew.labels.append(l)    
    l = pyglet.text.Label(f"yaw: {yaw: 3.1f}", x=15, y=window.height - 85, font_size=14, batch=Stew.textbatch)
    Stew.labels.append(l)
    l = pyglet.text.Label(f"mode: {Stew.modelabel}", x=15, y=window.height - 105, font_size=14, batch=Stew.textbatch)
    Stew.labels.append(l)
    
    for i, cyl in enumerate(Stew.cyls):
        l = pyglet.text.Label(f"Cyl {i}: {cyl: 3.1f}mm", x=15, y=window.height - (140+i*20), font_size=14, batch=Stew.textbatch)
        Stew.labels.append(l)
    Stew.textbatch.draw()
    
    """x = round((.5 * joystick.x + 1), 2) * window.width / 2
    y = round((-.5 * joystick.y + 1), 2) * window.height / 2
    rx = (.5 * joystick.rx + 1) * 60
    ry = (-.5 * joystick.ry + 1) * 60
    z = joystick.z * 50
    #print(f"joystick.z * 1024: {joystick.z*1024}, {z}")
    #print(f"x: {x:0.3f}, y: {y:0.3f}, rx: {rx:0.3f}, ry: {ry:0.3f}, z: {z:0.3f}")
    # Axes
    joystick_rect.position = x, y
    joystick_rect.anchor_position = joystick_rect.width // 2, joystick_rect.height // 2
    joystick_rect.width = 10 + rx + z
    joystick_rect.height = 10 + ry + z

    #sa: 4, sb: 5, sc: 6, sd: 7
    #lx/yaw: 3, ly/coll: 2, rx/roll: 0, ry/pitch: 1"""
    
    #print([f"{i}: {x.value}" for (i, x) in enumerate(joystick.device.get_controls()[24:])])
    """# Buttons
    for i in range(len(joystick.buttons)):
        rect = button_shapes[i]
        rect.color = (0, 255, 0) if joystick.buttons[i] else (255, 0, 0)"""

import dataforslerp as slerpdata

@window.event
def on_draw():
    #global textbatch
    window.clear()
    #oldbatch.draw()
    
    """ 

           
    """

    #x, y, z
    cube = slerpdata.scube

    # heading, pitch, roll
    rcube = slerpdata.rcube
    #rcube = cube * 15 # 15 or so
    #scube = cube - lin.vector(.5, .5, 0) # center it
    #print(f"rcube.shape: {rcube.shape}")
    #rcube = rcube[12:]
    #scube = cube[12:]
    scube = cube
    #scube = scube * .75 # scale it down a bit.
    #scube = scube + lin.vector(0, 0, 1) # give it room to move at 0 collective
    
    now = time.time()
    inow = int(now)
    cubedex1 = inow % len(cube)
    cubedex2 = (inow + 1) % len(cube)
    fnow = now - inow
    #framedex = (int(fnow*11) % 11)/10.0 # should be 0.0..1.0
    framedex = fnow
    
    #if(framedex == 0) and (cubedex1 == 0):
    #    print("-"*40)
    
    rcube1 = rcube[cubedex1]
    rcube2 = rcube[cubedex2]
    
    #euler_quat(heading, pitch, roll) yaw, pitch, roll
    rotor1 = slerp.euler_quat(m.radians(rcube1[0]), m.radians(rcube1[1]), m.radians(rcube1[2]))
    rotor2 = slerp.euler_quat(m.radians(rcube2[0]), m.radians(rcube2[1]), m.radians(rcube2[2]))

    scube1 = scube[cubedex1]
    scube2 = scube[cubedex2]
    cubefract = (scube2-scube1)
    
    rotor = slerp.slerp(rotor1, rotor2, framedex)
    (r, p, y) = slerp.to_euler(rotor) # (roll, pitch, yaw)
    roll = m.degrees(r)
    pitch = m.degrees(p)
    yaw = m.degrees(y)
    #print(f"cubedex1: {cubedex1:5}, cubedex2: {cubedex2:5}, framedex: {framedex:5.2f} r: {r:5.2f} p: {p:5.2f} y: {y:5.2f}", end='\n')
    #print(f"r1: {slerp.fq(rotor1)}\nr2: {slerp.fq(rotor2)}\nrr: {slerp.fq(rotor)}")
    #print(f"r1: ({rotor1.x:5.3f}, {rotor1.y:5.3f}, {rotor1.z:5.3f}), r2: ({rotor2.x:5.3f}, {rotor2.y:5.3f}, {rotor2.z:5.3f}), framedex: {framedex:5.2f} rx: {rotor.x:5.3f} ry: {rotor.y:5.3f} rz: {rotor.z:5.3f}", end='\n')
    
    lerpcube = scube1 + framedex*cubefract
    colspokes = Stew.solve4(rotor, *lerpcube)
    if len(colspokes) == 4:
        coll_v, sa, sb, sc = colspokes
        Stew.draw(coll_v, sa, sb, sc)

        l = pyglet.text.Label(f"coll_v: ({coll_v[0]: 3.1f}, {coll_v[1]: 3.1f}, {coll_v[2]: 3.1f})", x=15, y=window.height - 25, font_size=14, batch=Stew.textbatch)
        Stew.labels.append(l)
        l = pyglet.text.Label(f"roll: {roll: 3.1f}", x=15, y=window.height - 45, font_size=14, batch=Stew.textbatch)
        Stew.labels.append(l)    
        l = pyglet.text.Label(f"pitch: {pitch: 3.1f}", x=15, y=window.height - 65, font_size=14, batch=Stew.textbatch)
        Stew.labels.append(l)    
        l = pyglet.text.Label(f"yaw: {yaw: 3.1f}", x=15, y=window.height - 85, font_size=14, batch=Stew.textbatch)
        Stew.labels.append(l)
        for i, cyl in enumerate(Stew.cyls):
            l = pyglet.text.Label(f"Cyl {i}: {cyl: 3.1f}mm", x=15, y=window.height - (140+i*20), font_size=14, batch=Stew.textbatch)
            Stew.labels.append(l)
    Stew.textbatch.draw()
    
pyglet.app.run()
