#!/usr/bin/env python3

# convenience wrapper around opencv drawing functions.

import sys
import numpy as np
A = np.array
import math as m
import colorsys
import cv2


ituple = lambda t: tuple([int(x) for x in t])

def vline(img, p1, p2, c, width):
    cv2.line(img, ituple(p1), ituple(p2), ituple(c), width)

pt = lambda x, y: A([x, y])
c = lambda b, g, r: A([b, g, r], dtype='int32')
bgr = c
def hsv82bgr(h, s, v):
    (r, g, b) = colorsys.hsv_to_rgb(h/255.0, s/255.0, v/255.0)
    return bgr(b*255, g*255, r*255)

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

