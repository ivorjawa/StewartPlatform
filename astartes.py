#!/usr/bin/env python

import time, random
import numpy as np
import cv2

from PIL import ImageFont, ImageDraw, Image

from drawplate import SquareBoard, black, yellow, cyan, red, green
from statemachine import StateMachine

class MoveSM(StateMachine):
    def __init__(self):
        super().__init__()
        self.build("movestates", ['select', 'sleep'])
        self.starttime = time.time()
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.interval = 1 # second
    def select(self):
        self.x1 = random.randrange(0, 8)
        self.y1 = random.randrange(0, 6)
        self.x2 = random.randrange(0, 8)
        self.y2 = random.randrange(0, 6)
        self.starttime = time.time()
        self.state = self.states.sleep
    def sleep(self):
        if time.time() > (self.starttime + self.interval):
            self.state = self.states.select
            
class astartes(object):
    def __init__(self):
        self.movesm = MoveSM()
        print("created astartus")
        
        self.earth_font = ImageFont.truetype("fonts/future-earth.ttf", 32)
        self.chic_font = ImageFont.truetype("fonts/chicago.ttf", 32)

    def go(self):
        while(1):
            self.movesm.tick()
            self.draw()
            if cv2.waitKey(30) == 27:
                print("bye")
                return
                
    def draw(self):
        grid_mm = 183
        grid = SquareBoard(800, 700, grid_mm)
        
        center = np.array((grid_mm/2, grid_mm/2))
        
        grid.rectangle((0,0), (grid_mm, grid_mm), black, -1, layer=grid.print_layer)
        ocr = 166/2 # outer circle
        grid.circle(center, ocr, cyan, -1, layer=grid.print_layer) # outer circle
        #grid.circle(center, ocr+2, yellow, 1, layer=grid.ring_cut_layer) # outer circle
        #grid.rectangle((0,0), (grid_mm, grid_mm), yellow, layer=grid.cut_layer)
        
        # checker board 
        xc = grid_mm/2 # center
        yc = grid_mm/2
        cbw = (105/2) # width
        cbh = (75/2) # height
        cs = (15) # square side in pixels
        cbxo = xc-cbw # x origin
        cbyo = yc-cbh # y originm
        cbxm = xc+cbw # x max
        cbym = yc+cbh # y max
        
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
        
        ccr = 2 # center of lego pin holes
        as_start = ((self.movesm.x1 * cs) + cbxo, (self.movesm.y1 * cs) + cbyo)
        as_end = ((self.movesm.x2 * cs) + cbxo, (self.movesm.y2 * cs) + cbyo)
        grid.circle(as_start, ccr, red, -1, layer=grid.print_layer)
        grid.circle(as_end, ccr, green, -1, layer=grid.print_layer)
        
        
        img_pil = Image.fromarray(grid.canvas)
        draw = ImageDraw.Draw(img_pil)
        b,g,r,a = 0,0,255,0
        draw.text((150, 100),  "Hasta la Vista, Baby!", font = self.earth_font, fill = (b, g, r, a))
        draw.text((150, 300),  "38911 BASIC BYTES FREE", font = self.chic_font, fill = (b, g, r, a))
        
        grid.canvas = np.array(img_pil)
        
        grid.crosshairs() 
        grid.show()
        
                
                
if __name__ == "__main__":
    astartes().go()