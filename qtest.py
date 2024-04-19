#!/usr/bin/env python
import sys
import datetime
import logging, logging.handlers
import multiprocessing as mp
import asyncio
import time

import cv2
import uvc 

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

from joycode import JoyProtocol
from statemachine import BasicSM, Transition, State
from rotorbase import LoggingBricksHub

class SeekingPose(State):
    def __init__(self, sm):
        super().__init__()
        self.sm = sm
    def work(self):
        #print("SeekPose.work()")
        if self.sm.moved:
            self.sm.moved = False
            print("processed movement")
            
    def reset(self):
        pass
        
class MovingPlatform(State):
    def __init__(self, sm):
        super().__init__()
        self.sm = sm
    def work(self):
        #print("MovingPlatform.work()")
        if len(self.sm.pose) > 0: #edge trigger
            self.sm.pose.clear()
            print("processed pose")
    def reset(self):
        pass
        
class CalibSM(BasicSM):
    """
    command full left roll
    wait for completion
    calculate time
    seek pose
    command full right roll
    wait for completion 
    calculate time
    seek pose 
    ...
    """
    def __init__(self):
        super().__init__()
        self.pose = []
        self.moved = False
        self.seeking_pose = SeekingPose(self)
        self.moving_plat = MovingPlatform(self)
        self.t_pose_found = Transition(lambda: len(self.pose) >0, self.moving_plat)
        self.t_plat_moved = Transition(lambda: self.moved == True, self.seeking_pose)
        
        self.moving_plat.set_transitions([self.t_plat_moved])
        self.moving_plat.set_transitions([self.t_plat_moved])
        
        self.states = [self.seeking_pose, self.moving_plat]
        self.cur_state = self.seeking_pose
        self.default_state = self.seeking_pose
                
def find_mode(cap, width, height, fps):
    """
    for mode in cap.available_modes:
        print(
            f"MODE: {mode.width} x {mode.height} @ {mode.fps} ({mode.format_name}) {mode.__class__}"
        )
    """
    for mode in cap.available_modes:
        if (mode.width == width) and (mode.height == height) and (mode.fps == fps):
            return mode
            
def load_img(fromq, toq):
    #print(image)
    width = 640
    height = 480
    device = uvc.device_list()[0]
    cap = uvc.Capture(device["uid"]) 
    cap.frame_mode = find_mode(cap, width, height, 30)
    csm = CalibSM()
    csm.start()
    #cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")
    #im = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    #_, inv = cv2.threshold(im, 150, 255, cv2.THRESH_BINARY_INV)
    #cv2.GaussianBlur(inv, (3, 3), 0)
    count = 0
    while 1:
        #ret, frame = cam.read()
        #if not ret:
        #    break
        frame = cap.get_frame().bgr
        cv2.line(frame, (int(0),int(height/2)), (int(width), int(height/2)), (0, 0, 255), 1)
        cv2.line(frame, (int(width/2),int(0)), (int(width/2), int(height)), (0, 0, 255), 1)
        cv2.imshow('Async test', frame)
        count += 1
        csm.pose = [count]
        #print(f"frame {count}")
        cdict = {'roll': 255, 'pitch': 8, 'yaw': 16, 'coll': 32, 'glyph': 64}
        toq.put_nowait(cdict)

        if cv2.pollKey() == 27:
            break
        try:
            token = fromq.get_nowait()
            #fromq.task_done()
            csm.moved = True
            print(f"got token {token} csm.moved: {csm.moved}")
            
            #break
        except Exception as e:
            print(e)
        print(f"pretick: csm.pose: {csm.pose}, csm.moved: {csm.moved}, csm.state:  {csm.cur_state}")
        csm.tick()
        print(f"posttick: csm.pose: {csm.pose}, csm.moved: {csm.moved}, csm.state:  {csm.cur_state}")
    cv2.destroyAllWindows()
    return

class  LoggingQueuedBricksHub(PybricksHub):
    """
        A PybricksHub that can act on things received by _line_handler.
        <report> (lines) </report> will create a log file from 
        logfilename, ideally comma-separated values.
        <goodbye/> is the disconnect single from the hub.
    """
    def __init__(self, logfilename, toq):
        super().__init__()
        self.csvfile = None
        self.csv_stemname = logfilename
        self.toq = toq
        
    def _line_handler(self, line: bytes) -> None:
        try:
            l = line.decode()
            logging.info(f"Hub Sent:  {l}")
            
            if l == "<report>":
                fn = "%s_%s.csv" % (self.csv_stemname,
                                    str(datetime.datetime.now().isoformat()))
                self.csvfile = open(fn, "w")
                logging.info("logging to ", fn)
                return
            elif l == "</report>":
                if(self.csvfile):
                    self.csvfile.close()
                    self.csvfile = None
                    logging.info("done logging")
            elif l == "<goodbye/>":
                sys.exit(1)
            elif l == "<taskdone/>":
                self.toq.put_nowait(l) 
                print("queued taskdone")           
            if(self.csvfile):
                print(l, file=self.csvfile)
        except Exception as e:
            logging.error(f"_line_handler error: {e}")
        
 
class BaseStation(object):
    def __init__(self, fromq, toq):
        self.fromq = fromq
        self.toq = toq
        self.last_sent = time.time()
        #self.gamepad = ctrl.TaranisX9d()
    
    async def send_data(self, hub):
        lastout = ""
        wvars = ['roll', 'pitch', 'yaw', 'coll', 'glyph']
        wirep = JoyProtocol(wvars, 2, None, sys.stdin)
        logging.info("starting main loop")
        while 1:
        
            #report = self.gamepad.report()
            report = {}
            try:
                cdict = self.fromq.get_nowait()
                #report["framenum"] = framenum
                #print(f"got frame framenum")
            except Exception as e:
                pass
            if len(cdict) > 0:  
                #print(report)          
                output = wirep.encode(cdict)
                #print(output)
                if (output != lastout) or (((time.time()-self.last_sent)*1000) > 16):
                    lastout = output
                    logging.debug(output)
                    try:
                        await hub.write(bytearray(output, 'ascii'))
                        self.last_sent = time.time()
                    except bleak.exc.BleakError as e:
                        logging.debug(f"BLE communication error: {e}")
                    except Exception as e:
                        logging.error(f"Other error in hub.write(): {e}")
                        #sys.exit(0)
    
    async def go(self, logbasename, brickaddress, pyprog):
        hub = LoggingQueuedBricksHub(logbasename, self.toq)
        address = await find_device(brickaddress)
        await hub.connect(address)   

        try:
            await hub.run(pyprog, wait=False)
            await asyncio.gather(self.send_data(hub))

        except Exception as e:
            logging.error("script gather failed:  ", e)

        await hub.disconnect()
    
    def engage(self, logbasename, brickaddress, pyprog):
        asyncio.run(self.go(logbasename, brickaddress, pyprog))

def slurnk(fromq, toq):
    bubblebase = BaseStation(fromq, toq)
    bubblebase.engage("bubble", "bubble", "bubble.py")
      
               
if __name__ == '__main__':
    mp.set_start_method('spawn')
    cvq = mp.Queue()
    brickq = mp.Queue()
    #q = mp.Queue()
    p = mp.Process(target=load_img, args=(brickq, cvq))
    p.start()
    p2 = mp.Process(target=slurnk, args=(cvq, brickq))
    p2.start()
    #print(q.get())
    p.join()
    p2.join()