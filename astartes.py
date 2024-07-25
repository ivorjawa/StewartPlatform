#!/usr/bin/env python

import time, random
import math as m
import heapq

import numpy as np
import cv2
from PIL import ImageFont, ImageDraw, Image

from drawplate import SquareBoard, black, yellow, cyan, red, green, white, blue
from statemachine import StateMachine

# A*, Dijkstra
# https://www.redblobgames.com/pathfinding/a-star/introduction.html
# https://www.redblobgames.com/pathfinding/a-star/implementation.html
# https://youtu.be/CgW0HPHqFE8
# https://youtu.be/A60q6dcoCjw
# https://youtu.be/pVfj6mxhdMw

#GRIDX = 8
#GRIDY = 6
#GRIDPIX = 15

#GRIDX = 12
#GRIDY = 10
#GRIDPIX = 10

GRIDX = 24
GRIDY = 18
GRIDPIX = 5

class PriorityQueue:
    def __init__(self):
        self.elements = []
    def empty(self):
        return not self.elements
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    def get(self):
        return heapq.heappop(self.elements)[1]
        
def reconstruct_path(came_from, start, goal):
    current = goal # : Location 
    path = [] # : list[Location]
    if goal not in came_from: # no path was found
        print(f"goal {goal} not in came_from")
        return []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path
    
def cbcost(p1, p2):
    # just straight line distance
    #return m.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
    return abs(p2[0]-p1[0])+ abs(p2[1]-p1[1])

def cbneighbors(x,y,xsize=GRIDX,ysize=GRIDY):
    # returns a list of valid neighbors in a checkerboard of size xsize by ysize
    even = lambda n: n % 2 == 0
    
    maxx = xsize-1
    maxy = ysize-1
    
    if sum([(x > maxx), (x < 0), (y > maxy), (y < 0)]) > 0:
        raise IndexError(f"x: {x} maxx: {maxx} y: {y} maxy: {maxy}")
        
    up = (x, y + 1)
    down = (x, y - 1)
    left = (x - 1, y)
    right = (x + 1, y)
    
    #print(f"x: {x} y: {y} up: {up} down: {down} left: {left} right: {right}")
    xparts = []
    yparts = []
    
    # general, easy case, not on any edge
    if sum([(x < maxx), (x > 0), (y < maxy), (y > 0)]) == 4:
        return [up, down, left, right]
    
    # all edge.  don't you hate edgelords?
    if x == 0:
        xparts = [right]
        if even(y):
            yparts = [up]
        else:
            yparts = [down]
    else: # x == maxx:
        xparts = [left]
        if even(y):
            yparts = [up]
        else:
            yparts = [down]
    
    if y == 0:
        yparts = [up]
        if even(x):
            xparts = [right]
        else:
            xparts = [left]
    else: # y == maxy: 
        yparts = [down]
        if even(x):
            xparts = [right]
        else:
            xparts = [left]
        
    return xparts+yparts

class aster(object):
    def __init__(self):
        self.frontier = None
        self.current = None
        self.came_from = {}
    def astar_gen(self, start = (0, 0), goal = (4, 4)):
        frontier = PriorityQueue()
        self.frontier = frontier
        frontier.put(start, 0)
        #came_from = {} # : dict[Location, Optional[Location]] 
        cost_so_far = {} # : dict[Location, float]
        self.came_from[start] = None
        cost_so_far[start] = 0
        visited = {}
    
        while not frontier.empty():
            current = frontier.get() # : Location
            self.current = current
            #print(f"current: {current} frontier: {dir(frontier)}")
            # put frontier instead of "visited" here?
            yield reconstruct_path(self.came_from, start, goal)
            if current == goal:
                print(f"found goal {goal} after {len(self.came_from.keys())} steps")
                return
                #break
        
            for next in cbneighbors(*current):
                visited[current] = True
                new_cost = cost_so_far[current] + cbcost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + cbcost(next, goal)
                    frontier.put(next, priority)
                    self.came_from[next] = current
                    
    
class MoveSM(StateMachine):
    def __init__(self):
        super().__init__()
        self.build("movestates", ['select', 'path', 'sleep', 'freeze'])
        self.pausetime = time.time()
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.interval = 0 # second
    def select(self):
        self.x1 = random.randrange(0, GRIDX)
        self.y1 = random.randrange(0, GRIDY)
        self.x2 = random.randrange(0, GRIDX)
        self.y2 = random.randrange(0, GRIDY)
        self.pausetime = time.time() + self.interval
        self.aster = aster()
        self.pather = self.aster.astar_gen((self.x1, self.y1), (self.x2, self.y2))
        self.visited = set()
        self.iternum = 0
        self.pathlen = 0
        self.newpath = []
        self.camefrom = {}
        self.foundpath = False
        self.currentpoint = (0, 0)
        self.state = self.states.path
    def path(self):
        try:
            self.newpath = next(self.pather)
            self.camefrom = {}
            self.currentpoint = self.aster.current
            self.pathlen = len(self.newpath)
            self.iternum += 1
            self.visited.add(self.currentpoint)
            #for point in self.newpath:
            #    self.visited.add(point)
            self.pausetime = time.time()+self.interval
            self.state = self.states.sleep
        except StopIteration:
            self.foundpath = True
            self.state = self.states.freeze
            self.restarttime = time.time() + 3
    def sleep(self):
        if time.time() > self.pausetime:
            self.state = self.states.path
    def freeze(self):
        if time.time() > self.restarttime:
            self.state = self.states.select
            
class astartes(object):
    def __init__(self):
        self.movesm = MoveSM()
        print("created astartus")
        
        self.grid_mm = 183
        self.grid = SquareBoard(800, 700, self.grid_mm)
        
        self.earth_font = ImageFont.truetype("fonts/future-earth.ttf", 20)
        self.chic_font = ImageFont.truetype("fonts/chicago.ttf", 20)

    def go(self):
        while(1):
            self.movesm.tick()
            self.draw()
            if cv2.waitKey(1) == 27:
                print("bye")
                return
                
    def draw(self):
        grid = self.grid
        grid_mm = self.grid_mm
        
        center = np.array((grid_mm/2, grid_mm/2))
        
        grid.rectangle((0,0), (grid_mm, grid_mm), black, -1, layer=grid.print_layer)
        ocr = 166/2 # outer circle
        grid.circle(center, ocr, cyan, -1, layer=grid.print_layer) # outer circle

        
        # checker board 
        xc = grid_mm/2 # center
        yc = grid_mm/2
        cbw = (105/2) # width
        cbh = (75/2) # height
        cs = (GRIDPIX) # square side in pixels
        cbxo = xc-cbw # x origin
        cbyo = yc-cbh # y originm
        cbxm = xc+cbw # x max
        cbym = yc+cbh # y max

        for x in range(0, GRIDX, 2):
            for y in range(0, GRIDY, 2):
                x1 = cbxo+x*cs
                y1 = cbyo+y*cs
                x2 = cbxo+(x+1)*cs
                y2 = cbyo+(y+1)*cs
                #print(f"x: {x}, y: {y}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
                grid.rectangle((x1, y1), (x2, y2), black, -1, layer=grid.print_layer)    
        for x in range(1, GRIDX-1, 2):
            for y in range(1, GRIDY-1, 2):
                x1 = cbxo+x*cs
                y1 = cbyo+y*cs
                x2 = cbxo+(x+1)*cs
                y2 = cbyo+(y+1)*cs
                #print(f"x: {x}, y: {y}, x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
                grid.rectangle((x1, y1), (x2, y2), black, -1, layer=grid.print_layer)
        
        maxdist = m.sqrt(GRIDX**2 + GRIDY**2)
        cmult = 255/maxdist
        ccr = 1.5 # dot radius
        gridpix = lambda x, y: ((x * cs) + cbxo, (y * cs) + cbyo)
        for x in range(GRIDX):
            for y in range(GRIDY):
                # calculating A* potential.  We eventually want to make
                # this so we can only move on black.
                
                # sqrt(8*8 + 6*6) = 10
                dist = m.sqrt((self.movesm.x2-x)**2 + (self.movesm.y2-y)**2)
                gv = int(cmult * (maxdist-dist))
                c = (gv, gv, gv)
                distdot = gridpix(x, y)
                grid.circle(distdot, ccr, c, -1, layer=grid.print_layer)
        
        for point in self.movesm.visited:
            grid.circle(gridpix(*point), ccr, yellow, -1, layer=grid.print_layer) 
        
        # the path we just tried
        # reconstruct_path(came_from, start, goal)
        history = reconstruct_path(
            self.movesm.aster.came_from, 
            (self.movesm.x1, self.movesm.y1), 
            self.movesm.currentpoint
            )   
        print(self.movesm.camefrom) 
        for point in history:
            grid.circle(gridpix(*point), ccr, (0, 128, 255), -1, layer=grid.print_layer)  
            
        if(self.movesm.foundpath):
            pathcolor = green
        else:
            pathcolor = (255, 0, 255)
            
        for point in self.movesm.newpath:
            grid.circle(gridpix(*point), ccr, pathcolor, -1, layer=grid.print_layer)
        
           
        #as_start = ((self.movesm.x1 * cs) + cbxo, (self.movesm.y1 * cs) + cbyo)
        #as_end = ((self.movesm.x2 * cs) + cbxo, (self.movesm.y2 * cs) + cbyo)
        as_start = gridpix(self.movesm.x1, self.movesm.y1)
        as_end = gridpix(self.movesm.x2, self.movesm.y2)
        grid.circle(as_start, ccr+.5, green, 2, layer=grid.print_layer)
        grid.circle(as_start, ccr, red, -1, layer=grid.print_layer)
        grid.circle(as_end, ccr+.5, green, 2, layer=grid.print_layer)
        grid.circle(as_end, ccr, blue, -1, layer=grid.print_layer)
        
        #https://stackoverflow.com/questions/37191008/load-truetype-font-to-opencv
        
        img_pil = Image.fromarray(grid.canvas)
        draw = ImageDraw.Draw(img_pil)
        b,g,r,a = 0,0,255,128
        draw.text(
            (200, 60),  
            f"Iteration: {self.movesm.iternum}, Path Length: {self.movesm.pathlen}", 
            font = self.earth_font, fill = (b, g, r, a))
        draw.text(
            (200, 725),  
            f"Examining {self.movesm.currentpoint} Found: {self.movesm.foundpath} History: {len(history)}", 
            font = self.chic_font, fill = (b, g, r, a))
        
        grid.canvas = np.array(img_pil)
        
        grid.crosshairs() 
        grid.show(pdf=False)
        
                
                
if __name__ == "__main__":
    astartes().go()