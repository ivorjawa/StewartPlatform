#!/usr/bin/env python

import sys
import os
import datetime
import logging, logging.handlers
import multiprocessing as mp
import asyncio
import time
import random
import pickle

import numpy as np
import cv2

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

from rich import print as rprint

from joycode import JoyProtocol

from statemachine import StateMachine
import StewartPlatform
from pose_est import Recognizer

#"maps -1.0..1.0 to 0..255"
one28 = lambda x: int(((x+1)/2)*255)

     
class TrackerSM(StateMachine):
    def __init__(self, fromq, toq):
        super().__init__()
        self.build('cstates', ['wait_start', 'gen_sig', 'wait_move', 'scan', 'count', 'done'])
        
        self.fromq = fromq
        self.toq = toq

        self.start_time = 0
        self.end_time = 0
        self.woke = False
        self.moving = False
        
        self.width = 640
        self.height = 480

        rec = Recognizer(self.height, self.width)
        
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cam.set(cv2.CAP_PROP_FPS, 30)

    
        os.system("./uvc-util -I 0 -s auto-focus=0")
        os.system("./uvc-util -I 0 -s focus-abs=0")
        os.system("./uvc-util -I 0 -s auto-exposure-mode=1")
        os.system("./uvc-util -I 0 -s exposure-time-abs=150")

    def recognize(self):
        print("recognizer")
        ret, pose = self.calibrator.calibrate_frame(self.frame)
        return ret, pose
        
    
    def wait_start(self):
        #print("asleep")
        if self.woke:
            self.state = self.states.gen_sig
            print("generating signal")
            
    def gen_sig(self):
        #StewartPlatform.cSD = Flatmode
        #StewartPlatform.cSC set is cup motion, otherwise dome motion
        #.cSB is kill switch
        modeglyph = StewartPlatform.cSC|StewartPlatform.cSD
        cdict = {
            'roll': one28(random.random()-.5), 
            'pitch': one28(random.random()-.5), 
            'yaw': one28(random.random()-.5), 
            'coll': one28(random.random()-.5), 
            'glyph': modeglyph
        }
        #print(f"cdict: {cdict}")
        #self.cur_pitch += 1/self.posegoal
        self.toq.put_nowait(cdict)
        self.moving = True
        self.start_time = time.time()
        print("sent command")
        self.state = self.states.wait_move
        
    def wait_move(self):
        now = time.time()
        if self.moving == True:
            if (now - self.start_time > 5):
                print("Timeout waiting for robot to move, kicking")
                self.state = self.states.gen_sig
        elif self.moving == False:
            self.end_time = now
            self.state = self.states.scan
            print(f"got movement, took {self.end_time-self.start_time:3.3f}s")
            
    def scan(self):
        ret, pose = self.recognize()
        if ret:
            self.poses.append(pose)
            print(f"appended pose {len(self.poses)}")
        self.state = self.states.count
        
    def count(self):
        if len(self.poses) >= self.posegoal:
            print("enough poses found")
            self.calibrator.calculate()
            self.state = self.states.done
        else:
            self.state = self.states.gen_sig           

    
    def loop(self):
        while 1:
            #self.frame = self.cap.get_frame().bgr
            ret, self.frame = self.cam.read()
            if not ret:
                print("eof?")
                break

            cv2.line(self.frame, np.intp((0, self.height/2)), np.intp((self.width, self.height/2)), (0, 0, 255), 1)
            cv2.line(self.frame, np.intp((self.width/2, 0)), np.intp((self.width/2, self.height)), (0, 0, 255), 1)
            cv2.imshow('calibrator', self.frame)
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

def tracker(fromq, toq):
    tsm = TrackerSM(fromq, toq)
    tsm.loop()

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
            logging.debug(f"Hub Sent:  {l}")
            
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
        #wvars = ['coll', 'roll', 'pitch', 'yaw', 'glyph']
        from stewart_wvars import wvars
        wirep = JoyProtocol(wvars, 2, None, sys.stdin)
        logging.info("starting main loop")
        while 1:
        
            #report = self.gamepad.report()
            #for x in [24, 40, 72, 68, 255]: print(f"{x:#010b}") 
            """
            >>> for x in [24, 40, 72, 68, 128, 129, 130, 131, 255]: print(f"{x:#010b}")
            ... 
            0b00011000
            0b00101000
            0b01001000
            0b01000100
            0b10000000
            0b10000001
            0b10000010
            0b10000011
            0b11111111
            
            if we treat bit 7 as a command flag, we've got 2 bits for modes
            (3 if we consider keepawake a mode, 4 if keepawake is handled separately)
            Glyph still has to die.
            129 will signify "slerp to next pose"
            130 will be "twitch (no interpolation) to next pose" 
            """
            cdict = {'roll': 0, 'pitch': 0, 'yaw': 0, 'coll': 0, 'glyph': 255} # keepawake
            # get command from opencv recognizer process
            try:
                cdict = self.fromq.get_nowait()
            except Exception as e:
                pass
            #print(report)  
            output = wirep.encode(cdict)
            
            # may need this throttling, not sure yet, recognizer locked to 30 hz
            #dtms = (time.time() - self.last_sent) * 1000
            #if (dtms > 16): # only send 60 fps
            
            if (output != lastout) or (((time.time()-self.last_sent)*1000) > 16):
                lastout = output
                logging.debug(output)
                #print(f"cdict before send: {cdict}")        
                #print(f"output: {output}")
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

def robotlink(fromq, toq):
    robotbase = BaseStation(fromq, toq)
    robotbase.engage('stewbase', 'jawaspike', 'stuart.py') # make it so
    #robotbase.engage("bubble", "bubble", "bubble.py") # make it so on a dummy machine
      
               
if __name__ == '__main__':
    mp.set_start_method('spawn')
    
    cvq = mp.Queue()
    brickq = mp.Queue()
    
    p = mp.Process(target=tracker, args=(brickq, cvq))
    p.start()
    p2 = mp.Process(target=robotlink, args=(cvq, brickq))
    p2.start()
    
    p.join()
    p2.join()