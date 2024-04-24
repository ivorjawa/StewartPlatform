#!/usr/bin/env python
import sys
import datetime
import logging, logging.handlers
import multiprocessing as mp
import asyncio
import time
from enum import Enum

import numpy as np
import cv2
import uvc 

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

from joycode import JoyProtocol
#from statemachine import BasicSM, Transition, State
from rotorbase import LoggingBricksHub


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

class AEnum(object):
    # this is good enough for state machines in micropython
    def __init__(self, enumname, enums, start=1):
        self.__enumname = enumname
        for i, e in enumerate(enums):
            setattr(self, e, i+start)
        
class CalibSM(object):
    """
    Generate pose and signal
    Wait for movement to complete
    Scan for board and save if found
    Loop until points is 50
    """
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
    def __init__(self, fromq, toq):
        self.fromq = fromq
        self.toq = toq

        self.start_time = 0
        self.end_time = 0
        self.woke = False
        self.moving = False
        self.poses = []
        
        self.width = 640
        self.height = 480
        self.device = uvc.device_list()[0]
        self.cap = uvc.Capture(self.device["uid"]) 
        self.cap.frame_mode = find_mode(self.cap, self.width, self.height, 30)
        
        self.states = Enum('cstates', ['wait_start', 'gen_sig', 'wait_move', 'scan', 'count', 'done'])
        self.state = self.states.wait_start
    
    def recognize(self):
        return True, True
        
    def tick(self):
        #print(f"tick(): self.state: {self.state} |{self.state == self.states.wait_start}|")
        if self.state == self.states.wait_start:
            #print("asleep")
            if self.woke:
                self.state = self.states.gen_sig
                print("generating signal")
        elif self.state == self.states.gen_sig:
            cdict = {'roll': 255, 'pitch': 8, 'yaw': 16, 'coll': 32, 'glyph': 64}
            self.toq.put_nowait(cdict)
            self.moving = True
            self.start_time = time.time()
            print("sent command")
            self.state = self.states.wait_move
        elif self.state == self.states.wait_move:
            if self.moving == False:
                self.end_time = time.time()
                self.state = self.states.scan
                print(f"got movement, took {self.end_time-self.start_time:3.3f}s")
        elif self.state == self.states.scan:
            ret, pose = self.recognize()
            if ret:
                self.poses.append(pose)
                print(f"appended pose {len(self.poses)}")
            self.state = self.states.count
        elif self.state == self.states.count:
            if len(self.poses) > 50:
                print("enough poses found")
                self.state = self.states.done
            else:
                self.state = self.states.gen_sig           
        else:
            print(f"sufficient poses found {self.state}")
            pass
    
    def loop(self):
        while 1:
            frame = self.cap.get_frame().bgr
            cv2.line(frame, np.intp((0, self.height/2)), np.intp((self.width, self.height/2)), (0, 0, 255), 1)
            cv2.line(frame, np.intp((self.width/2, 0)), np.intp((self.width/2, self.height)), (0, 0, 255), 1)
            cv2.imshow('Async test', frame)
            try:
                token = self.fromq.get_nowait()
                print(f"got token {token}")
                if token == "<awake/>":
                    self.woke = True
                elif token == "<taskdone/>":
                    self.moving = False
                else:
                    print(f"got unknown token: {token}")
            except Exception as e:
                pass
            self.tick()
            if cv2.pollKey() == 27:
                break
        cv2.destroyAllWindows()
        return        


def load_img(fromq, toq):
    csm = CalibSM(fromq, toq)
    csm.loop()
    
def old_load_img(fromq, toq):
    #print(image)
    width = 640
    height = 480
    device = uvc.device_list()[0]
    cap = uvc.Capture(device["uid"]) 
    cap.frame_mode = find_mode(cap, width, height, 30)
    #csm = CalibSM()
    #csm.start()
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
            #print(e)
            pass
        #print(f"pretick: csm.pose: {csm.pose}, csm.moved: {csm.moved}, csm.state:  {csm.cur_state}")
        csm.tick()
        #print(f"posttick: csm.pose: {csm.pose}, csm.moved: {csm.moved}, csm.state:  {csm.cur_state}")
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
            elif l == "<awake/>":
                self.toq.put_nowait(l)
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
            cdict = {'roll': 0, 'pitch': 0, 'yaw': 0, 'coll': 0, 'glyph': 255} # keepawake
            try:
                cdict = self.fromq.get_nowait()
                #report["framenum"] = framenum
                #print(f"got frame framenum")
            except Exception as e:
                pass
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