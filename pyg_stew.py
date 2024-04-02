import pyglet
import numpy as np
import math as m
import time

from linear import xaxis, yaxis, zaxis, rotate, vector
import linear as lin

# convert from 3-space to 2-space
#twod = lambda p: np.array([p[0], p[1]])
twod = lambda p: [int(p[0]), int(p[1])]
 
#from pyglet.gl import *

joysticks = pyglet.input.get_joysticks()
assert joysticks, 'No joystick device is connected'
joystick = joysticks[0]
joystick.open()


window = pyglet.window.Window(width=800, height=800)
oldbatch = pyglet.graphics.Batch()


# Labels
pyglet.text.Label("Buttons:", x=15, y=window.height - 25, font_size=14, batch=oldbatch)
pyglet.text.Label("D Pad:", x=window.width - 125, y=window.height - 25, font_size=14, batch=oldbatch)

#c = lambda b, g, r: A([b, g, r], dtype='int32')
c = lambda b, g, r: np.array([r, g, b], dtype='int32') # converts bgr to rgb
red = c(0, 0, 255)
#red1 = red/2
green = c(0, 255, 0)
#green1 = green/2
yellow = c(0, 255, 255)
#yellow1 = yellow/2
white = c(255, 255, 255)
#gray1 = white/2
purple = c(255, 0, 255)
#purple1 = purple/2
blue = c(255, 0, 0)
#blue1 = blue/2
orange = c(0, 165, 255)
#orange1 = orange/2

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

      
class Stewart(object): # millimeters
    def __init__(self, inner_r, outer_r, footprint, min_cyl, max_cyl):
        self.Cmin = min_cyl
        self.Cmax = max_cyl
        self.Crange = max_cyl - min_cyl
        self.inner_r = inner_r
        self.outer_r = outer_r
        self.p0 = lin.vector(0, 0, 0)
        self.last_time = time.time()
        self.times = [time.time() for x in range(5)]
        self.modelabel = "init"
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
        self.old_Vp = lin.vector(0, 0, 0)
        self.old_Vr = lin.vector(0, 0, 0)
        self.old_Vq = lin.vector(0, 0, 0)
        
        self.s1 = rotate(zaxis, m.radians(-60+rotoff), fp_line) + outer_r*lin.normalize(self.sA)
        self.s2 = rotate(zaxis, m.radians(120+rotoff), fp_line) + outer_r*lin.normalize(self.sA)
        self.s3 = rotate(zaxis, m.radians(60+rotoff), fp_line) + outer_r*lin.normalize(self.sB)
        self.s4 = rotate(zaxis, m.radians(240+rotoff), fp_line) + outer_r*lin.normalize(self.sB)
        self.s5 = rotate(zaxis, m.radians(180+rotoff), fp_line) + outer_r*lin.normalize(self.sC)
        self.s6 = rotate(zaxis, m.radians(0+rotoff), fp_line) + outer_r*lin.normalize(self.sC)
        

        # circle(self, center, radius, color, lw=1):   
        
        
    def solve(self, roll, pitch, yaw, coll, glyph): 
        global outcount, fastcount, outlog       
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
        #white = white
        #red = red
        #green = green

        Vp = lin.vector(m.cos(m.radians(pitch)), 0, m.sin(m.radians(pitch)))
        Vr = lin.vector(0, m.cos(m.radians(roll)), m.sin(m.radians(roll)))
        Vq = lin.normalize(Vp + Vr)
        #print(f"Pitch: {pitch} Vp: {Vp} roll: {roll} Vr: {Vr} Vq: {Vq}")
        
        # Normal of rotor disk
        Vdisk = lin.cross(Vp, Vr) 
        Vdisk_n = lin.normalize(Vdisk)
        
        coll_p = coll*self.plat_range+self.min_height
        coll_v = coll_p * Vdisk_n         
        
        flatmode = False
        if (glyph & cSD) == cSD:
            flatmode = True
            #print("using flat motion")
            coll_v = lin.vector(coll_v[0], coll_v[1], coll_p)
            self.modelabel = f"flat motion {glyph}"
        
        if not flatmode:
            if (glyph & cSC) == cSC: 
                #print("using cup motion") 
                self.modelabel = f"cup motion {glyph}"     
                oily = euler_rotation_matrix(m.radians(-roll),m.radians(pitch),0) # cup motion
            else:
                self.modelabel = f"sphere motion {glyph}"
                #print("using sphere motion")
                oily = euler_rotation_matrix(m.radians(roll),m.radians(-pitch),0) # sphere motion
                
        
        if flatmode:
            sa = rotate(zaxis, m.radians(yaw), self.sA)
            sb = rotate(zaxis, m.radians(yaw), self.sB)
            sc = rotate(zaxis, m.radians(yaw), self.sC)
            sa = sa+coll_v
            sb = sb+coll_v
            sc = sc+coll_v          
        else:
            sa = lin.matmul(oily, self.sA)+coll_v
            sb = lin.matmul(oily, self.sB)+coll_v
            sc = lin.matmul(oily, self.sC)+coll_v
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
                #print(f"cylinder too short: c1: {c1} c2: {c2} Cmin: {self.Cmin}")
                sa = self.old_sA
                sb = self.old_sB
                sc = self.old_sC
                coll_v = self.old_coll_v
                #return
                
            if max(c1, c2, self.Cmax) != self.Cmax:
                #print(f"cylinder too long: c1: {c1} c2: {c2} Cax: {self.Cmax}")
                sa = self.old_sA
                sb = self.old_sB
                sc = self.old_sC
                coll_v = self.old_coll_v
                
                #return
                
            self.old_sA = sa
            self.old_sB = sb
            self.old_sC = sc
            self.old_coll_v = coll_v
            self.old_Vr = Vr
            self.old_Vp = Vp
            self.old_Vq = Vq
            
            return (coll_v, sa, sb, sc)
        
    def draw(self, coll_v, sa, sb, sc):           
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

line2 = pyglet.shapes.Line(0, 0, window.width/2, window.height/2, width=4, color=(200, 20, 20), batch=oldbatch)

Stew = Stewart(40, 120, 120, 240, 308) #inner, outer radius, footprint, min, max cylinder extension

@window.event
def on_draw():
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

        
    coll = coll_in 
    roll = (-roll_in) * 40
    pitch = (pitch_in) * 40
    yaw = (yaw_in) * 90 
    
    #print(f"c: {coll:{3.3}}, r: {roll:{3.3}}, p: {pitch:{3.3}}, y: {yaw:{3.3}} g: {glyph}")
    (coll_v, sa, sb, sc) = Stew.solve(roll, pitch, yaw, coll, glyph)
    Stew.draw(coll_v, sa, sb, sc)
    
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




pyglet.app.run()