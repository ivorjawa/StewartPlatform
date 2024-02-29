#!/usr/bin/env python3

import sys
import numpy as np
A = np.array
import math as m
import cv2
#import cv2.cv as cv

rad2deg = m.degrees #lambda x: (x/m.pi)*180
deg2rad = m.radians #lambda d: (d*m.pi)/180

def proj(r, theta):
    return A([r*m.cos(theta), r*m.sin(theta)])

vlen = lambda v: np.linalg.norm(v)
nmz = lambda a: np.array(a) / vlen(a) #np.linalg.norm(a) # normalize wactor
ituple = lambda t: tuple([int(x) for x in t])

def vline(img, p1, p2, c, width):
    cv2.line(img, ituple(p1), ituple(p2), ituple(c), width)

pt = lambda x, y: A([x, y])
c = lambda b, g, r: A([b, g, r], dtype='int32')
bgr = c

def rmat(degs):
    theta = deg2rad(degs)
    rotMatrix = np.array([[np.cos(theta), -np.sin(theta)],
                             [np.sin(theta),  np.cos(theta)]])
    return rotMatrix


red = c(0, 0, 255)
red1 = red/2
green = c(0, 255, 0)
green1 = green/2
yellow = c(0, 255, 255)
yellow1 = yellow/2
white = c(255, 255, 255)
gray1 = white/2
purple = c(255, 0, 255)
purple1 = purple/2
blue = c(255, 0, 0)
blue1 = blue/2
orange = c(0, 165, 255)
orange1 = orange/2

class SimGrid(object):
    def __init__(self, width=300, height=300, scale=1):
        self.w = width
        self.h = height
        self.canvas = np.zeros((width, height, 3), np.uint8)
        self.c = pt(width//2, height//2 + -40)
        self.scale = scale
        self.flip = pt(1, -1)
    def line(self, p1, p2, color, lw=1):
        P1 = self.scale*self.flip*p1 + self.c
        P2 = self.scale*self.flip*p2 + self.c
        vline(self.canvas, P1, P2, color, lw)
    def circle(self, center, radius, color, lw=1):
        center = self.scale*self.flip*center + self.c
        radius = self.scale*radius
        cv2.circle(self.canvas, ituple(center), int(radius), ituple(color), lw)
    def draw_convex(self, points, color):
        p = [pt(*x)*self.scale*self.flip+self.c for x in points]
        points = A([[int(a[0]), int(a[1])] for a in p])
        #print points
        print("red: ", repr(red))
        tr = tuple([int(x) for x in red])
        print("TR: ", repr(tr))
        print("tr0: ", type(tr[0]))
        cv2.polylines(self.canvas,[ points], True,  color=tr, thickness=2)
    def display(self):
        #cv_im = cv.fromarray(self.canvas)
        #pi = Image.fromstring("RGB", cv.GetSize(cv_im), cv_im.tostring())
        #fig, axes = plt.subplots(figsize=(self.h/100,self.w/100), dpi=100)
        #axes.imshow(np.asarray(pi))
        cv2.imshow("output", self.canvas)

from scipy.spatial import ConvexHull

class Winder(object):
    def reinit(self):
        self.leg_int_front = []
    def __init__(self, max_h, min_h):
        self.max_h = max_h
        self.min_h = min_h
        self.exts = [[],[]]
        self.grid = None
        self.reinit()
    def display(self):
        hullpoints = A(self.exts[0] + self.exts[1])

        hull = ConvexHull(hullpoints); points = [hull.points[x] for x in hull.vertices]
        #print "cooked hull: ", points

        self.grid.draw_convex(points, red)
        self.grid.display()
        #print hullpoints
    def windup(self, grid, degs, ltrack, rtrack, id):
        self.exts[id] = []
        if id == 1:
            _green = green
            _red = red
            _yellow = yellow
            _white = white
            _purple = purple
            _orange = orange
        else:
            _green = green1
            _red = red1
            _yellow = yellow1
            _white = gray1
            _purple = purple1
            _orange = orange1

        theta = deg2rad(degs)

        #basic measurements
        leg_top_r = 46.42 # from center of cam follower
        leg_bot_r = 76.54
        cam_r = 5.89 # distance between motor and center of cam
        cam_r /= 2 # erry day I'm shufflin'
        follower_r = 6.5 # was 9, can use a smaller follow, calculate this exactly later
        pin_r = 2.14 # used for both cam and
        pcb_motor_dist = 53.73 # for positioning hall effect sensor
        #cam_shoulder_dist= 46.42
        motor = pt(0, 0) # motor is center of coordinate system
        frame_pin = motor + pt(0, 37.28)
        pcb_point = motor + pt(0, pcb_motor_dist)
        cam_p = proj(cam_r, theta) + motor
        foot_width = 38.31
        thigh_radius = 12
        thigh_length = 45 # down from shoulder
        chest_radius = 38
        hip_radius = 60

        cam_center = cam_p # cam center aligns with follower center on leg body
        leg_top_dir = nmz(frame_pin - cam_center)
        leg_top = leg_top_dir * leg_top_r + cam_center
        self.exts[id].append(leg_top)
        leg_bot = -1 * leg_top_dir * leg_bot_r + cam_center
        foot_dir = np.dot(rmat(90), leg_top_dir)

        self.exts[id].append(foot_width * foot_dir + leg_bot)
        self.exts[id].append(-foot_width * foot_dir + leg_bot)

        if id == 1:
            foot_p1 = foot_width * .45 * foot_dir + leg_bot
            foot_p2 = -foot_width * foot_dir + leg_bot
        else:
            foot_p1 = foot_width * foot_dir + leg_bot
            foot_p2 = -foot_width * .45 * foot_dir + leg_bot

        ltrack.append(foot_p1)
        rtrack.append(foot_p2)

        hip_chest_r = m.sqrt(thigh_radius**2 + hip_radius**2)
        hip_chest_angle = m.atan2(thigh_radius, hip_radius)
        mod_hip_r = hip_radius * m.sin(hip_chest_angle)
        #print "hip_chest_r: %0.2f, angle: %0.2f, mod_hip_r: %0.2f" %( hip_chest_r, rad2deg(hip_chest_angle), mod_hip_r)
        rib_center = frame_pin + pt(-mod_hip_r, 0)
        rib_bot = rib_center + pt(0, -70)
        grid.line(rib_center, rib_bot, white, 2)
        min_h_1 = self.min_h + pt(0, 70)
        max_h_1 = self.max_h + pt(0, 70)
        grid.line(self.max_h, max_h_1, gray1, 2)
        grid.line(self.min_h, min_h_1, gray1, 2)


        thigh_front = -thigh_radius * foot_dir + cam_center
        thigh_back = thigh_radius * foot_dir + cam_center
        thigh_front_p1 = thigh_front + thigh_length * leg_top_dir
        thigh_front_p2 = thigh_front + thigh_length * -leg_top_dir
        thigh_back_p1 = thigh_back + thigh_length * leg_top_dir
        thigh_back_p2 = thigh_back + thigh_length * -leg_top_dir
        self.leg_int_front.append(thigh_back_p2)
        for p in self.leg_int_front:
            grid.line(p, p, orange, 3)

        grid.line(thigh_front_p1, thigh_front_p2, _orange, 2)
        grid.line(thigh_back_p1, thigh_back_p2, _orange, 2)

        grid.circle(cam_center, follower_r, _purple, 2)
        grid.circle(motor, pin_r, blue, 2)
        grid.circle(frame_pin, pin_r, blue, 2)

        # figure out hall effect height
        mag_cube = 1.587 # mm
        mag_height = (0, mag_cube)
        mag_dist = pcb_point - (leg_top + mag_height)

        sh_width = 9.345 / 2
        sh_p1 = sh_width * foot_dir + leg_top
        sh_p2 = -sh_width * foot_dir + leg_top
        pcbo = pt(sh_width, 0)
        grid.line(pcb_point-pcbo, pcb_point+pcbo, green, 2)
        grid.line(sh_p1, sh_p2, _green, 2)
        if id == 1:
            grid.line(pcb_point, leg_top+mag_height, red, 2)
            draw_button(grid.canvas, "%0.3f mm" % vlen(mag_dist))

        grid.line(motor, cam_p, _red, 2)
        grid.line(leg_top, leg_bot, _green, 2)
        grid.line(foot_p1, foot_p2, _white, 2)
        for i in range(len(ltrack)):
            grid.line(ltrack[i], ltrack[i], _yellow, 10)
            grid.line(rtrack[i], rtrack[i], _yellow, 10)

def draw_button(cv_img, text, x=200, y=200, scale=1):
    T = lambda a: tuple(a)
    fnt = cv2.FONT_HERSHEY_SIMPLEX
    clr = (255, 255, 255)
    coords = (x, y)
    #scale = 1
    thick = 1
    ((width, height), baseline) = cv2.getTextSize(text, fnt, scale, thick)
    rec_geom = (width, (height-baseline))
    #offs = np.array([width//2, height//2])
    cv2.rectangle(cv_img,
                  T(A(coords) - (0, height)),
                  T(A(coords) + rec_geom),
                  (125, 125, 125), -1)
    cv2.putText(cv_img, text, coords, fnt, scale, clr, thick, cv2.LINE_AA)

def run():
    max_h = pt(0, 0)
    min_h = pt(0, 0)
    while 1:
        ltrack1 = []
        rtrack1 = []
        ltrack2 = []
        rtrack2 = []
        winder = Winder(min_h, max_h)
        #rpm = 150 # motor @ 4.5v
        rpm = 20
        rps = rpm / 60 # revs / seconds
        fps = 30 # frames / seconds
        rpf = rps / fps # revs / seconds * 1/ frames / seconds = revs / frames
        dpf = rpf * 360 # degrees per frame
        print("rpm: %d rps: %0.2f fps: %d rpf: %0.2f dpf: %0.2f" % (rpm, rps, fps, rpf, dpf))
        #sys.exit(0)
        for i in range(0, 360, int(dpf)):
            grid = SimGrid(800, 800, 5)
            winder.grid = grid
            winder.windup(grid, i, ltrack1, rtrack1, 0)
            winder.windup(grid, i+180, ltrack2, rtrack2, 1)
            winder.display()
            inkey  = cv2.waitKey( 30)
            if inkey == 27:
                sys.exit(0)
        if len(winder.leg_int_front) > 0:
            min_h = min(winder.leg_int_front, key=lambda x:x[0])
            max_h = max(winder.leg_int_front, key=lambda x:x[0])
            print("min_h: %s, max_h: %s" % (min_h, max_h))


if __name__ == "__main__":
    run()
